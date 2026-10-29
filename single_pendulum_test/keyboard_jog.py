import os
os.environ["SDL_VIDEODRIVER"] = "dummy"
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

import time
import pygame
import odrive
from odrive.utils import dump_errors

AXIS = 0
INVERT = 1
DEADZONE = 0.10
VEL_MAX = 50.0
RAMP_RATE = 150.0     # was 5.0 -- the only aggressive change in this run
MAX_TRAVEL = 25.0
STOP_BUTTON = 0

pygame.init()
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    raise SystemExit("No gamepad found. Pair/connect it first.")
pad = pygame.joystick.Joystick(0)
pad.init()
print(f"Gamepad: {pad.get_name()}  axes={pad.get_numaxes()} buttons={pad.get_numbuttons()}")

odrv = odrive.find_any(timeout=20)
ax = odrv.axis0
print(f"ODrive: {odrv.serial_number:X}")


def clear_errors(odrv, verbose=True):
    """Set IDLE and clear all errors. Returns True if axis0 is clean."""
    ax = odrv.axis0
    ax.requested_state = 1
    try:
        ax.config.enable_watchdog = False
    except Exception:
        pass
    time.sleep(0.2)
    for a in (odrv.axis0, odrv.axis1):
        for obj in (a, a.motor, a.encoder, a.controller):
            try:
                obj.error = 0
            except Exception:
                pass
    time.sleep(0.2)
    left = {"axis": ax.error, "motor": ax.motor.error,
            "encoder": ax.encoder.error, "controller": ax.controller.error}
    clean = all(v == 0 for v in left.values())
    if verbose:
        print("axis0 errors after clear:",
              "none" if clean else {k: hex(v) for k, v in left.items() if v})
        print(f"state={ax.current_state}  is_calibrated={ax.motor.is_calibrated}  "
              f"encoder.is_ready={ax.encoder.is_ready}")
    return clean


# ---- clear BEFORE enabling ----
if not clear_errors(odrv):
    dump_errors(odrv)
    raise SystemExit("Errors would not clear. Stopped.")

ax.controller.config.control_mode = 2        # VELOCITY_CONTROL
ax.controller.config.input_mode = 1          # VEL_RAMP
ax.controller.config.vel_ramp_rate = RAMP_RATE
if ax.controller.config.vel_limit < VEL_MAX * 1.2:
    ax.controller.config.vel_limit = VEL_MAX * 1.2
ax.controller.input_vel = 0
start_pos = ax.encoder.pos_estimate

ax.requested_state = 8                       # CLOSED_LOOP
time.sleep(0.5)

# ---- only READ after enabling (do not clear) ----
if ax.current_state != 8:
    print("--- errors after enabling ---")
    dump_errors(odrv)
    ax.requested_state = 1
    raise SystemExit(f"Not in closed loop (state={ax.current_state}). Stopped.")

print("Stick = velocity, release = ramp to 0, button 0 = stop, Ctrl+C = exit")

try:
    last_cmd = None
    next_t = time.perf_counter()
    last_print = 0.0
    while True:
        lost = False
        for e in pygame.event.get():
            if e.type == pygame.JOYDEVICEREMOVED:
                lost = True
        if lost:
            ax.controller.input_vel = 0
            raise RuntimeError("Gamepad disconnected")

        if ax.error != 0:
            print(f"\nFAULT: axis 0x{ax.error:X} ctrl 0x{ax.controller.error:X} "
                  f"motor 0x{ax.motor.error:X}")
            break

        raw = pad.get_axis(AXIS) * INVERT
        if abs(raw) < DEADZONE:
            raw = 0.0
        target = raw * VEL_MAX
        if pad.get_button(STOP_BUTTON):
            target = 0.0

        pos = ax.encoder.pos_estimate - start_pos
        state = ""
        if pos > MAX_TRAVEL and target > 0:
            target, state = 0.0, "LIMIT +"
        elif pos < -MAX_TRAVEL and target < 0:
            target, state = 0.0, "LIMIT -"

        if target != last_cmd:
            ax.controller.input_vel = target
            last_cmd = target

        now = time.perf_counter()
        if now - last_print > 0.1:
            print(f"\rtarget={target:+.2f} actual={ax.encoder.vel_estimate:+.2f} "
                  f"Iq_set={ax.motor.current_control.Iq_setpoint:+.2f}A "
                  f"Iq_meas={ax.motor.current_control.Iq_measured:+.2f}A "
                  f"pos={pos:+.2f} {state:<8}", end="", flush=True)
            last_print = now

        next_t += 0.01
        time.sleep(max(0, next_t - time.perf_counter()))

except KeyboardInterrupt:
    pass
except Exception as exc:
    print(f"\nERROR: {type(exc).__name__}: {exc}")
finally:
    try:
        ax.controller.input_vel = 0
        t0 = time.time()
        while abs(ax.encoder.vel_estimate) > 0.05 and time.time() - t0 < 3:
            time.sleep(0.05)
        ax.requested_state = 1
        print("\nSet to IDLE")
        dump_errors(odrv)
    except Exception:
        print("\nConnection lost. Power off or use the emergency stop.")
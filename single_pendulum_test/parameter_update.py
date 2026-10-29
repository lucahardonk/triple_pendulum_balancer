#!/usr/bin/env python3
"""parameter_update.py - ODrive 0.5.x axis0 velocity-loop tuning only.
No calibration, no erase, no reboot, no closed-loop request.
  python3 parameter_update.py          (RAM only)
  python3 parameter_update.py --save   (also save to flash)
"""
import sys
import time
import odrive
from odrive.utils import dump_errors

IDLE = 1

# (path on axis0, value, note) -- starting guesses, NOT tuned for your rail
PARAMS = [
    ("controller.config.control_mode",           2,      "2 = velocity control"),
    ("controller.config.input_mode",             2,      "2 = VEL_RAMP"),
    ("controller.config.vel_ramp_rate",          100.0,   "turn/s^2, acceleration knob"),
    ("controller.config.vel_gain",               2.0,   "Nm/(turn/s)"),
    ("controller.config.vel_integrator_gain",    0.1,    "0 = no windup; raise only once it moves"),
    ("controller.config.vel_limit",              10.0,    "turn/s"),
    ("controller.config.vel_limit_tolerance",    2.0,    "overspeed trips at vel_limit * this"),
    ("controller.config.enable_vel_limit",       True,   ""),
    ("controller.config.enable_overspeed_error", True,   ""),
    ("motor.config.torque_constant",             0.0486, "8.27 / 170 kV"),
    ("motor.config.current_lim",                 10.0,   "A (your original was 15)"),
    ("encoder.config.bandwidth",                 1000.0, "your original value"),
    ("config.startup_closed_loop_control",       False,  "no motion at boot"),
    ("config.enable_watchdog",                   False,  "the jog script enables it itself"),
    ("config.watchdog_timeout",                  0.5,    "s"),
]


def get(obj, path):
    for p in path.split("."):
        obj = getattr(obj, p)
    return obj


def put(obj, path, value):
    *parents, last = path.split(".")
    for p in parents:
        obj = getattr(obj, p)
    setattr(obj, last, value)


print("Looking for ODrive...", flush=True)
odrv = odrive.find_any(timeout=20)
ax = odrv.axis0
print(f"Connected: {odrv.serial_number:X}")

if ax.current_state != IDLE:
    ax.requested_state = IDLE
    time.sleep(0.3)
    if ax.current_state != IDLE:
        sys.exit("Axis is not IDLE. Nothing changed.")

# --- read-only motor / encoder check ---
m = ax.motor.config
print("\n--- motor check (read only) ---")
print(f"vbus {odrv.vbus_voltage:.2f} V | is_calibrated {ax.motor.is_calibrated} "
      f"| encoder.is_ready {ax.encoder.is_ready}")
print(f"R {m.phase_resistance:.4f} ohm | L {m.phase_inductance*1e6:.1f} uH "
      f"| pole_pairs {m.pole_pairs} | type {m.motor_type}")
print(f"current_lim {m.current_lim} A | calib_current {m.calibration_current} A "
      f"| requested_current_range {m.requested_current_range} A")
print(f"current_control_bandwidth {m.current_control_bandwidth} Hz")
if m.phase_resistance == 0:
    print("WARNING: phase_resistance is 0, so the motor was never measured.")

print("\nErrors before update (not cleared):")
dump_errors(odrv)

print(f"\n{'parameter':48}{'old':>10}{'new':>10}   note")
bad, skipped = [], []
for path, new, note in PARAMS:
    try:
        old = get(ax, path)
        put(ax, path, new)
        now = get(ax, path)
    except AttributeError:
        skipped.append(path)
        print(f"{path:48}{'-':>10}{'-':>10}   NOT FOUND, skipped")
        continue
    miss = abs(float(now) - float(new)) > 1e-4
    if miss:
        bad.append(path)
    print(f"{path:48}{float(old):>10.4g}{float(new):>10.4g}   {note}"
          f"{'  <-- MISMATCH' if miss else ''}")
print(f"\nSkipped: {skipped or 'none'} | Mismatched: {bad or 'none'}")

# --- axis1 (unused): idle, no startup actions, clear stale errors ---
a1 = odrv.axis1
a1.requested_state = IDLE
for flag in ("startup_motor_calibration", "startup_encoder_offset_calibration",
             "startup_closed_loop_control", "startup_sensorless_control",
             "startup_homing"):
    try:
        setattr(a1.config, flag, False)
    except AttributeError:
        pass
time.sleep(0.2)
a1.error = 0
a1.motor.error = 0
a1.encoder.error = 0
a1.controller.error = 0

if bad:
    sys.exit("Mismatch found. Not saving.")

if "--save" in sys.argv:
    try:
        odrv.save_configuration()
        print("Saved to flash.")
    except Exception as exc:
        print(f"Save sent; connection dropped as expected ({type(exc).__name__}).")
else:
    print("RAM only. Run with --save to keep.")
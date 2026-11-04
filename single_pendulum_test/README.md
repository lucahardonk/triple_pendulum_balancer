# Single Pendulum Balancer

Single inverted-pendulum balancing test using a belt-driven linear cart, an MKS XDrive Mini running ODrive firmware, and pendulum encoder data streamed over USB by an ESP32-C6.

## Files

- `2_PID_balancer.py` — main program: sensor initialization, automatic angle zeroing, rail homing, centering, and cascaded PID balancing.
- `esp_32_firmware_encoders/` — encoder-reading firmware, provided as an `.ino` sketch inside this directory.
- `keyboard_jog_motor_test.py` — keyboard-based motor control test.
- `parameter_update.py` — helper for loading multiple ODrive parameters at once in `odrivetool`.
- `odrive-legacy/` — Python virtual environment used for the legacy ODrive setup.

## Activate the environment

From this subsection's directory:

```bash
source odrive-legacy/bin/activate
```

## Run the main program

Start with the pendulum hanging down and stationary: the program uses this position for automatic angle zeroing. Homing moves the cart toward both mechanical end stops, measures the travel and turns-per-meter ratio, checks encoder signs, then centers the cart. In `READY`, balancing starts automatically when the pendulum enters the capture window around the upright position.

The inner PID converts pendulum angle error into cart velocity. The outer PID converts cart position error into a requested pendulum tilt. Software travel limits stop operation if exceeded.

Command used for the current setup:

```bash
python3 2_PID_balancer.py --rail-meters 1.06 \
  --kp-in 5.0 --ki-in 30.0 --kd-in 0.15 \
  --kp-out 0.4 --ki-out 0.05 --kd-out 0.1 \
  --max-tilt 6.0 --max-vel 35.0 --catch-angle 10.0
```

These are the current test settings, not a guarantee of stable balancing but tested.

## Main parameters

| Parameter | Purpose | Value in the command |
| --- | --- | --- |
| `--rail-meters` | Actual cart travel between end stops; used to calculate turns/m | `1.06` m |
| `--kp-in`, `--ki-in`, `--kd-in` | Inner angle-to-velocity PID gains | `5.0`, `30.0`, `0.15` |
| `--kp-out`, `--ki-out`, `--kd-out` | Outer position-to-tilt PID gains | `0.4`, `0.05`, `0.1` |
| `--max-tilt` | Maximum tilt requested by the outer loop | `6.0` degrees |
| `--max-vel` | Maximum commanded motor velocity | `35.0` turns/s |
| `--catch-angle` | Maximum deviation from upright for capture | `10.0` degrees |

Other useful options (defaults unless specified):

- `--port /dev/ttyACM0`, `--baud 921600`: ESP32 serial connection.
- `--axis 0`: ODrive axis.
- `--fall-angle 35.0`: leave balancing when the pendulum falls beyond this angle.
- `--soft-fraction 0.80`: software travel limit as a fraction of the half-travel.
- `--homing-vel 4.0`: homing velocity in turns/s.
- `--dt 0.002`: requested control-loop period in seconds; actual frequency depends on execution time.
- `--log tuning.csv`: save control data for tuning.
- `--skip-homing`: requires `--rail-meters`, `--turns-per-meter`, and `--angle-sign`; the current cart position becomes the center. Use only with the cart physically centered and a previously measured ratio/sign.

For all options and units:

```bash
python3 2_PID_balancer.py --help
```

## Controls and safety

- **Space**: request an emergency stop, then put the motor in `IDLE`.
- **r**: re-enable the motor after an emergency; does not restart incomplete homing.
- **b**: return from manual mode to `READY` for automatic capture.
- **m**: switch to manual mode.
- **q / Esc / Ctrl+C**: exit.
- **Joystick axis 0**: manual cart velocity, or position-setpoint adjustment while balancing.
- **Joystick button 0**: stop manual velocity / set the balancing position target to the current position.
- **Joystick button 1**: toggle balancing/manual operation.

Homing deliberately reaches the mechanical end stops. Keep the travel area clear and use a physical emergency stop: terminal controls and software limits are not a substitute for hardware protection.

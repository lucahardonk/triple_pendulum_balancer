# ODrive USB ROS 2 Driver

Single-node Python ROS 2 driver using the ODrive Python library over USB. All USB access is serialized by USBWorker; USBBackend handles device operations and SI conversion; ODriveDriver handles ROS interfaces.

## Installation

Keep the currently working ODrive Python library version. Build and run in a Python environment that can import both rclpy and odrive. Source your ROS 2 installation and activate your existing ODrive virtual environment first.

Package layout:
- setup.py
- setup.cfg
- package.xml
- README.md
- resource/odrive_driver (empty marker)
- odrive_driver/__init__.py
- odrive_driver/driver_node.py

In setup.py register this console entry point:
```python
entry_points={'console_scripts': ['driver_node = odrive_driver.driver_node:main']}
```

Include README.md beside package.xml in the share/odrive_driver data_files entry. Declare ament_python as a buildtool dependency and rclpy, std_msgs, std_srvs as dependencies in package.xml. Replace the maintainer email placeholder with your actual email. If the token-like value previously pasted as an email was real, revoke it.

setup.cfg:
```ini
[develop]
script_dir=$base/lib/odrive_driver

[install]
install_scripts=$base/lib/odrive_driver
```

Build:
```bash
python3 -c "import rclpy, odrive; print('Imports OK')"
cd ~/Documents/triple_pendulum_balancer/ros2_ws
python3 -m colcon build --packages-select odrive_driver --symlink-install
source install/setup.bash
ros2 pkg executables odrive_driver
```

The build environment needs colcon available to that Python interpreter. In new terminals source ROS 2 and the workspace overlay; use the ODrive environment to run the driver. Installing this package does not launch it at boot.

## Start

```bash
ros2 run odrive_driver driver_node --ros-args \
  -p serial_number:="'316B315D3335'" \
  -p axis:=0 -p rate_hz:=100.0 -p log_hz:=1.0
```

Connection reads existing control/input modes; it does not automatically select a mode or request closed-loop operation.

## Startup parameters

| Parameter | Default | Meaning |
|---|---|---|
| serial_number | empty | Empty selects any discovered device |
| axis | 0 | Axis 0 or 1 |
| connect_timeout_s | 20.0 | Connection timeout |
| rate_hz | 100.0 | Target worker and publication timer rate |
| log_hz | 1.0 | Console feedback rate; 0 disables logging |
| telemetry_json | position and velocity | Telemetry selection |
| unit_rules_json | {} | Additional numeric unit rules |

Parameters are startup-only. USB, Python and ROS scheduling do not provide hard real-time guarantees.

## Topics

| Topic | Type | Purpose |
|---|---|---|
| /odrive_driver/command | std_msgs/msg/Float64 | SI motion setpoint |
| /odrive_driver/request | std_msgs/msg/String | JSON administrative request |
| /odrive_driver/response | std_msgs/msg/String | JSON response |
| /odrive_driver/feedback | std_msgs/msg/String | JSON SI telemetry |

Command and feedback use best-effort QoS, depth 1. Request and response use reliable QoS, depth 32. Names assume the default node name.

Command units depend on selected control mode: absolute radians for position, rad/s for velocity, N*m for torque. The worker consumes the latest pending value; intermediate commands can be superseded. The last device setpoint persists until changed.

## Velocity example

Secure the mechanism and keep an independent stop available. Check each service result before continuing.

```bash
ros2 service call /odrive_driver/set_idle std_srvs/srv/Trigger '{}'
ros2 service call /odrive_driver/set_vel_mode std_srvs/srv/Trigger '{}'
ros2 service call /odrive_driver/set_closed_loop std_srvs/srv/Trigger '{}'
ros2 topic pub --once /odrive_driver/command std_msgs/msg/Float64 '{data: 0.2}'
```

Stream commands at 100 Hz:
```bash
ros2 topic pub --rate 100 /odrive_driver/command std_msgs/msg/Float64 '{data: 0.2}'
```

Publish changing Float64 values on this topic from your controller node. Stopping a publisher does not stop the motor.

## Stop

Stop active command publishers first. In velocity mode only, command zero velocity:
```bash
ros2 topic pub --once /odrive_driver/command std_msgs/msg/Float64 '{data: 0.0}'
```

Request IDLE:
```bash
ros2 service call /odrive_driver/set_idle std_srvs/srv/Trigger '{}'
```

IDLE disables active control; it is not controlled braking. Position zero means an absolute target of zero radians, not a stop command.

## Services

All services use std_srvs/srv/Trigger under /odrive_driver/.

| Service | Action |
|---|---|
| set_idle | Request IDLE |
| set_closed_loop | Request closed-loop control |
| set_vel_mode | Velocity/passthrough |
| set_position_mode | Position/passthrough |
| set_torque_mode | Torque/passthrough |
| set_passthrough | Passthrough for current mode |
| set_ramped | Velocity ramp, position trajectory, or torque ramp |
| set_position_filter | Position filter; position mode required |
| get_status | Modes, actual state, component errors |

Mode and shaping changes require IDLE. Mode selection initializes position from the current encoder angle and clears velocity/torque feedforward. Shaping selection does not configure rates or trajectory limits.

State service success means the state request was written, not that the axis reached the state. Verify actual state and errors:
```bash
ros2 service call /odrive_driver/get_status std_srvs/srv/Trigger '{}'
```

## Position trajectories and relative moves

Configure suitable trajectory limits while IDLE before motion. Select set_idle, set_position_mode, set_ramped, then set_closed_loop, checking every result. In this configuration the command topic specifies an absolute target in radians.

Relative move example:
```bash
ros2 topic pub --once /odrive_driver/request std_msgs/msg/String \
  '{data: "{\"id\":20,\"op\":\"move_relative\",\"value\":0.1,\"from_goal\":false}"}'
```

value is a signed increment in radians. from_goal=false references the current position; true references the previous goal. Requires position/trajectory mode and firmware exposing controller.move_incremental.

## Property get/set

Watch responses in another terminal:
```bash
ros2 topic echo /odrive_driver/response
```

Read velocity limit:
```bash
ros2 topic pub --once /odrive_driver/request std_msgs/msg/String \
  '{data: "{\"id\":1,\"op\":\"get\",\"path\":\"axis0.controller.config.vel_limit\"}"}'
```

Set velocity limit to 1 rad/s while IDLE:
```bash
ros2 topic pub --once /odrive_driver/request std_msgs/msg/String \
  '{data: "{\"id\":2,\"op\":\"set\",\"path\":\"axis0.controller.config.vel_limit\",\"value\":1.0}"}'
```

Numeric commands, telemetry and get/set use registered SI conversion. Unknown numeric properties are rejected until explicitly registered. Writes support configuration properties only, require affected axes IDLE, and return readback with verified and persistent fields. Check both ok and verified. Writes are not saved to flash. Select control/input modes through services, not property writes.

## Telemetry

```bash
ros2 topic echo /odrive_driver/feedback --qos-reliability best_effort
```

Feedback includes position/velocity by default, unit labels, axis state/error, modes, host monotonic timestamp, read duration and worker timing. Properties are read sequentially, not simultaneously. The timestamp is recorded after sampling and is not a ROS timestamp. Adding telemetry increases USB work. log_hz only affects terminal logging. Publication uses new cached snapshots and may not deliver every worker sample.

Custom telemetry example:
```bash
ros2 run odrive_driver driver_node --ros-args \
  -p serial_number:="'316B315D3335'" \
  -p telemetry_json:='{"position_rad":{"path":"encoder.pos_estimate"},"velocity_rad_s":{"path":"encoder.vel_estimate"},"bus_voltage":{"scope":"device","path":"vbus_voltage"}}'
```

Each alias specifies a path and optional scope: axis (default, relative to selected axis) or device. Additional unit_rules_json entries use exact property paths with native_to_si and unit fields. SI equals native multiplied by native_to_si. Factors must be positive and built-in rules cannot be overridden.

## Safety and limitations

- No automatic calibration, error clearing, flash saving or reconnection.
- No implemented watchdog feeding, host command timeout or magnitude limit.
- Last setpoint persists after publisher disconnection.
- Shutdown attempts IDLE; process termination or USB failure can prevent it.
- Use independent hardware safety measures.
- Do not change modes concurrently from another application: command interpretation uses cached mode selection.

## Troubleshooting

- Executable missing: check console entry point and setup.cfg; rebuild and source overlay.
- Import failure: verify the Python environment can import both rclpy and odrive; rebuild with the intended interpreter.
- Axis must be IDLE: request IDLE and verify actual state before retrying.
- No motion: inspect actual state, component errors, selected modes and command value.
- Unsupported property/enum: check library and firmware compatibility.
- USB worker stopped: inspect the logged exception; restart only after addressing the cause.

Build and hardware behavior have not been tested by the documentation author in this session.

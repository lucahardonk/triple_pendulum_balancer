#!/usr/bin/env python3
"""ROS 2 ODrive driver: SI commands, selected telemetry, mode services."""

import json
import math
import re
import threading
import time
from collections import deque
from dataclasses import dataclass, field

import odrive
import odrive.enums as enums
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger


PATH = re.compile(r"[A-Za-z][A-Za-z0-9_]*(?:\.[A-Za-z][A-Za-z0-9_]*)*")

MODES = {
    "position": ("CONTROL_MODE_POSITION_CONTROL", "input_pos", math.tau),
    "velocity": ("CONTROL_MODE_VELOCITY_CONTROL", "input_vel", math.tau),
    "torque": ("CONTROL_MODE_TORQUE_CONTROL", "input_torque", 1.0),
}

INPUTS = {
    "passthrough": ("INPUT_MODE_PASSTHROUGH", {"position", "velocity", "torque"}),
    "velocity_ramp": ("INPUT_MODE_VEL_RAMP", {"velocity"}),
    "trajectory": ("INPUT_MODE_TRAP_TRAJ", {"position"}),
    "torque_ramp": ("INPUT_MODE_TORQUE_RAMP", {"torque"}),
    "position_filter": ("INPUT_MODE_POS_FILTER", {"position"}),
}

UNIT_RULES = {
    "encoder.pos_estimate": (math.tau, "rad"),
    "encoder.vel_estimate": (math.tau, "rad/s"),
    "controller.input_pos": (math.tau, "rad"),
    "controller.input_vel": (math.tau, "rad/s"),
    "controller.input_torque": (1.0, "N*m"),
    "controller.config.vel_limit": (math.tau, "rad/s"),
    "controller.config.vel_ramp_rate": (math.tau, "rad/s^2"),
    "controller.config.torque_ramp_rate": (1.0, "N*m/s"),
    "controller.config.input_filter_bandwidth": (1.0, "1/s"),
    "controller.config.vel_gain": (1.0 / math.tau, "N*m*s/rad"),
    "controller.config.vel_integrator_gain": (1.0 / math.tau, "N*m/rad"),
    "controller.config.vel_limit_tolerance": (1.0, "1"),
    "controller.config.control_mode": (1.0, "enum"),
    "controller.config.input_mode": (1.0, "enum"),
    "trap_traj.config.vel_limit": (math.tau, "rad/s"),
    "trap_traj.config.accel_limit": (math.tau, "rad/s^2"),
    "trap_traj.config.decel_limit": (math.tau, "rad/s^2"),
    "motor.config.torque_constant": (1.0, "N*m/A"),
    "motor.config.current_lim": (1.0, "A"),
    "motor.config.calibration_current": (1.0, "A"),
    "motor.config.requested_current_range": (1.0, "A"),
    "motor.config.phase_resistance": (1.0, "ohm"),
    "motor.config.phase_inductance": (1.0, "H"),
    "motor.config.pole_pairs": (1.0, "1"),
    "motor.config.motor_type": (1.0, "enum"),
    "config.watchdog_timeout": (1.0, "s"),
    "current_state": (1.0, "enum"),
    "error": (1.0, "bitmask"),
    "motor.error": (1.0, "bitmask"),
    "encoder.error": (1.0, "bitmask"),
    "controller.error": (1.0, "bitmask"),
    "vbus_voltage": (1.0, "V"),
}


def finite(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError("Expected a finite number")
    return float(value)


def unit_key(path):
    parts = path.split(".")
    return ".".join(parts[1:]) if parts[0] in ("axis0", "axis1") else path


def resolve(root, path):
    if not isinstance(path, str) or not PATH.fullmatch(path):
        raise ValueError("Invalid property path")
    parts = path.split(".")
    for part in parts[:-1]:
        root = getattr(root, part)
    return root, parts[-1]


@dataclass
class Job:
    action: str
    done: threading.Event = field(default_factory=threading.Event)
    success: bool = False
    message: str = ""


class USBWorker:
    """Only this thread accesses the ODrive connection."""

    def __init__(self, params, notify, logger):
        self.params = params
        self.notify = notify
        self.logger = logger

        self.condition = threading.Condition()
        self.stopping = False
        self.ready = False
        self.fault = None
        self.transitioning = False

        self.target = None
        self.jobs = deque()
        self.output = None

        self.thread = threading.Thread(
            target=self.run, name="odrive_usb", daemon=True
        )

    def command(self, value):
        with self.condition:
            if self.ready and not self.stopping and not self.transitioning:
                self.target = value
                self.condition.notify()

    def submit(self, action):
        job = Job(action)
        with self.condition:
            if not self.ready or self.stopping:
                raise RuntimeError(self.fault or "USB worker not ready")
            if len(self.jobs) >= 8:
                raise RuntimeError("Service queue full")

            # Discard the old setpoint before every explicit transition.
            self.transitioning = True
            self.target = None
            if action == "idle":
                self.jobs.appendleft(job)
            else:
                self.jobs.append(job)
            self.condition.notify()
        return job

    def take_output(self):
        with self.condition:
            result = self.output
            self.output = None
        return result

    def configure_telemetry(self):
        self.units = dict(UNIT_RULES)
        extensions = json.loads(self.params["unit_rules_json"])
        if not isinstance(extensions, dict):
            raise ValueError("unit_rules_json must be an object")

        for path, rule in extensions.items():
            if not PATH.fullmatch(path):
                raise ValueError("Invalid unit-rule path")
            key = unit_key(path)
            if key in self.units:
                raise ValueError(f"Cannot override built-in rule: {key}")
            factor = finite(rule["native_to_si"])
            unit = rule["unit"]
            if factor <= 0 or not isinstance(unit, str) or not unit:
                raise ValueError("Expected positive factor and nonempty unit")
            self.units[key] = (factor, unit)

        spec = json.loads(self.params["telemetry_json"])
        if not isinstance(spec, dict):
            raise ValueError("telemetry_json must be an object")

        self.telemetry = []
        for alias, item in spec.items():
            if not isinstance(item, dict) or "scale" in item:
                raise ValueError("Invalid telemetry entry; use unit_rules_json")
            scope = item.get("scope", "axis")
            if scope not in ("axis", "device"):
                raise ValueError("Telemetry scope must be axis or device")
            path = item["path"]
            root = self.axis if scope == "axis" else self.device
            parent, name = resolve(root, path)
            value = getattr(parent, name)
            self.to_si(path, value)
            self.telemetry.append((alias, parent, name, path))

    def to_si(self, path, value):
        if type(value) is bool:
            return {"value": value, "unit": "bool"}
        if type(value) is str:
            return {"value": value, "unit": "string"}
        if type(value) not in (int, float):
            raise ValueError(f"Non-scalar telemetry: {path}")

        key = unit_key(path)
        if key not in self.units:
            raise ValueError(f"Register {path} in unit_rules_json")
        factor, unit = self.units[key]
        converted = value if factor == 1.0 else value * factor
        if isinstance(converted, float) and not math.isfinite(converted):
            converted = str(converted)
        return {"value": converted, "unit": unit}

    def read_mode(self):
        actual = self.controller.config.control_mode
        for name, (symbol, _, _) in MODES.items():
            if actual == getattr(enums, symbol):
                return name
        raise ValueError(f"Unsupported control mode: {actual}")

    def set_mode(self, mode, shaping):
        if self.axis.current_state != enums.AXIS_STATE_IDLE:
            raise RuntimeError("Axis must be IDLE")
        if mode not in INPUTS[shaping][1]:
            raise ValueError("Incompatible control/input mode")

        control = getattr(enums, MODES[mode][0])
        input_mode = getattr(enums, INPUTS[shaping][0])

        position = self.axis.encoder.pos_estimate
        self.controller.input_vel = 0.0
        self.controller.input_torque = 0.0
        self.controller.input_pos = position
        self.controller.config.control_mode = control
        self.controller.config.input_mode = input_mode

        self.mode = self.read_mode()
        actual_input = self.controller.config.input_mode
        if self.mode != mode or actual_input != input_mode:
            raise RuntimeError("Mode readback mismatch")
        return f"mode={mode}, input={shaping}"

    def execute(self, action):
        if action == "idle":
            self.axis.requested_state = enums.AXIS_STATE_IDLE
            return "IDLE requested"

        if action == "closed_loop":
            # Do not intentionally re-enable with a stale velocity/torque.
            if self.axis.current_state != enums.AXIS_STATE_IDLE:
                raise RuntimeError("Axis must be IDLE before enabling")
            self.controller.input_vel = 0.0
            self.controller.input_torque = 0.0
            self.controller.input_pos = self.axis.encoder.pos_estimate
            self.axis.requested_state = enums.AXIS_STATE_CLOSED_LOOP_CONTROL
            return "CLOSED_LOOP_CONTROL requested"

        if action in MODES:
            return self.set_mode(action, "passthrough")

        if action == "ramped":
            shaping = {
                "position": "trajectory",
                "velocity": "velocity_ramp",
                "torque": "torque_ramp",
            }[self.mode]
        else:
            shaping = action
        return self.set_mode(self.mode, shaping)

    def write_due_command(self):
        now = time.monotonic()
        if now < self.command_due:
            return

        with self.condition:
            value = None if self.transitioning else self.target

        if value is not None:
            _, name, factor = MODES[self.mode]
            setattr(self.controller, name, value / factor)

        # Skip missed slots instead of sending catch-up bursts.
        finished = time.monotonic()
        slots = max(
            1, int((finished - self.command_due) / self.command_period) + 1
        )
        self.command_due += slots * self.command_period

    def clear_startup_errors(self):
        # All device access stays inside the USB worker thread.
        self.axis.requested_state = enums.AXIS_STATE_IDLE
        deadline = time.monotonic() + 2.0

        while self.axis.current_state != enums.AXIS_STATE_IDLE:
            if self.stopping:
                raise RuntimeError("Shutdown during startup")
            if time.monotonic() >= deadline:
                raise RuntimeError("Startup: IDLE not confirmed; errors not cleared")
            time.sleep(0.02)

        error_objects = (
            ("axis", self.axis),
            ("motor", self.axis.motor),
            ("encoder", self.axis.encoder),
            ("controller", self.axis.controller),
        )

        before = {
            name: int(obj.error) for name, obj in error_objects
        }
        self.logger.info(
            "Startup errors before clear: "
            + ", ".join(f"{name}=0x{value:X}" for name, value in before.items())
        )

        for _, obj in error_objects:
            obj.error = 0

        after = {
            name: int(obj.error) for name, obj in error_objects
        }
        self.logger.info(
            "Startup errors after clear: "
            + ", ".join(f"{name}=0x{value:X}" for name, value in after.items())
        )

        if any(after.values()):
            raise RuntimeError("Startup: errors remain after clear")

        self.logger.info("Startup: errors cleared; axis left in IDLE")

    def run(self):
        self.device = None
        active_job = None
        try:
            self.device = odrive.find_any(
                serial_number=self.params["serial_number"] or None,
                timeout=self.params["connect_timeout_s"],
            )
            self.axis = getattr(self.device, f"axis{self.params['axis']}")
            self.controller = self.axis.controller
            self.clear_startup_errors()
            self.mode = self.read_mode()
            self.configure_telemetry()

            self.command_period = 1.0 / self.params["rate_hz"]
            telemetry_period = 1.0 / self.params["telemetry_hz"]
            self.command_due = time.monotonic()
            telemetry_due = self.command_due

            values = {}
            index = 0
            sampling = False

            with self.condition:
                self.ready = True
            self.logger.info("ODrive connected")

            while True:
                with self.condition:
                    if self.stopping:
                        break
                    active_job = self.jobs.popleft() if self.jobs else None

                if active_job is not None:
                    try:
                        active_job.message = self.execute(active_job.action)
                        active_job.success = True
                    except Exception as exc:
                        active_job.message = str(exc)
                    finally:
                        with self.condition:
                            self.target = None
                            self.transitioning = bool(self.jobs)
                        active_job.done.set()
                        active_job = None
                    # Do not mix samples from before and after a transition.
                    sampling = False
                    values = {}
                    continue

                self.write_due_command()
                now = time.monotonic()

                if not sampling and now >= telemetry_due:
                    slots = max(
                        1, int((now - telemetry_due) / telemetry_period) + 1
                    )
                    telemetry_due += slots * telemetry_period
                    values = {}
                    index = 0
                    sampling = bool(self.telemetry)

                if sampling:
                    # One synchronous read, then reconsider command deadline.
                    alias, parent, name, path = self.telemetry[index]
                    value = getattr(parent, name)
                    values[alias] = self.to_si(path, value)
                    index += 1

                    self.write_due_command()

                    if index == len(self.telemetry):
                        with self.condition:
                            self.output = values
                        self.notify()
                        sampling = False
                    continue

                deadline = min(self.command_due, telemetry_due)
                with self.condition:
                    if not self.stopping and not self.jobs:
                        self.condition.wait(
                            timeout=max(0.0, deadline - time.monotonic())
                        )

        except Exception as exc:
            self.fault = f"{type(exc).__name__}: {exc}"
            self.logger.error(f"USB stopped: {self.fault}")

        finally:
            with self.condition:
                self.ready = False
                self.target = None
                remaining = list(self.jobs)
                self.jobs.clear()

            if active_job is not None:
                remaining.insert(0, active_job)
            for job in remaining:
                job.success = False
                job.message = self.fault or "USB worker stopped"
                job.done.set()

            if self.device is not None and hasattr(self, "axis"):
                try:
                    self.axis.requested_state = enums.AXIS_STATE_IDLE
                except Exception as exc:
                    self.logger.error(f"Shutdown IDLE failed: {exc}")

    def close(self):
        with self.condition:
            self.stopping = True
            self.target = None
            self.condition.notify_all()
        self.thread.join()


class ODriveDriver(Node):
    def __init__(self):
        super().__init__("odrive_driver")
        defaults = {
            "serial_number": "",
            "axis": 0,
            "connect_timeout_s": 20.0,
            "rate_hz": 500.0,
            "telemetry_hz": 100.0,
            "unit_rules_json": "{}",
            "telemetry_json": json.dumps({
                "position_rad": {"path": "encoder.pos_estimate"},
                "velocity_rad_s": {"path": "encoder.vel_estimate"},
                "current_limit_a": {"path": "motor.config.current_lim"},
                "torque_constant_nm_per_a": {
                    "path": "motor.config.torque_constant"
                },
                "velocity_limit_rad_s": {
                    "path": "controller.config.vel_limit"
                },
            }),
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        params = {
            name: self.get_parameter(name).value for name in defaults
        }

        for name in ("rate_hz", "telemetry_hz", "connect_timeout_s"):
            params[name] = finite(params[name])
            if params[name] <= 0:
                raise ValueError(f"{name} must be positive")
        if type(params["axis"]) is not int or params["axis"] not in (0, 1):
            raise ValueError("axis must be 0 or 1")

        self.command_group = MutuallyExclusiveCallbackGroup()
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.publish_group = MutuallyExclusiveCallbackGroup()

        latest = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.feedback_pub = self.create_publisher(String, "~/feedback", latest)
        self.feedback_guard = self.create_guard_condition(
            self.publish_feedback, callback_group=self.publish_group
        )
        self.worker = USBWorker(
            params, self.feedback_guard.trigger, self.get_logger()
        )
        self.command_sub = self.create_subscription(
            Float64, "~/command", self.on_command, latest,
            callback_group=self.command_group,
        )

        actions = {
            "set_idle": "idle",
            "set_closed_loop": "closed_loop",
            "set_vel_mode": "velocity",
            "set_position_mode": "position",
            "set_torque_mode": "torque",
            "set_passthrough": "passthrough",
            "set_ramped": "ramped",
            "set_position_filter": "position_filter",
        }
        self._driver_services = [
            self.create_service(
                Trigger, "~/" + name, self.make_service(action),
                callback_group=self.service_group,
            )
            for name, action in actions.items()
        ]
        self.worker.thread.start()

    def on_command(self, message):
        # Ignore invalid commands without flooding logs at command rate.
        if math.isfinite(message.data):
            self.worker.command(message.data)

    def make_service(self, action):
        def callback(request, response):
            try:
                job = self.worker.submit(action)
                while not job.done.wait(0.1):
                    if not rclpy.ok():
                        response.success = False
                        response.message = "ROS shutdown; state not confirmed"
                        return response
                response.success = job.success
                response.message = job.message
            except Exception as exc:
                response.success = False
                response.message = str(exc)
            return response
        return callback

    def publish_feedback(self):
        values = self.worker.take_output()
        if values is not None:
            self.feedback_pub.publish(
                String(data=json.dumps(values, allow_nan=False))
            )


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=3)
    try:
        node = ODriveDriver()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.worker.close()
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
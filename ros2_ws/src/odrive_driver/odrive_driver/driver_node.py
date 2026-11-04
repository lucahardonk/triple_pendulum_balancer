#!/usr/bin/env python3
"""Single-node ROS 2 ODrive USB driver.

Topics:
  ~/command   std_msgs/Float64: rad, rad/s or N*m, according to mode
  ~/request   std_msgs/String: JSON administrative requests
  ~/response  std_msgs/String: JSON request results
  ~/feedback  std_msgs/String: JSON SI telemetry

Services (std_srvs/Trigger):
  ~/set_idle
  ~/set_closed_loop
  ~/set_vel_mode
  ~/set_position_mode
  ~/set_torque_mode
  ~/set_passthrough
  ~/set_ramped       velocity ramp / position trajectory / torque ramp
  ~/set_position_filter
  ~/get_status

All USB access belongs to one worker thread.
No automatic calibration, error clearing, flash saving or reconnection.
No host command timeout or magnitude limit.
"""

import json
import math
import queue
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
    "position": ("CONTROL_MODE_POSITION_CONTROL", "input_pos", math.tau, "rad"),
    "velocity": ("CONTROL_MODE_VELOCITY_CONTROL", "input_vel", math.tau, "rad/s"),
    "torque": ("CONTROL_MODE_TORQUE_CONTROL", "input_torque", 1.0, "N*m"),
}

INPUTS = {
    "passthrough": ("PASSTHROUGH", {"position", "velocity", "torque"}),
    "velocity_ramp": ("VEL_RAMP", {"velocity"}),
    "position_filter": ("POS_FILTER", {"position"}),
    "trajectory": ("TRAP_TRAJ", {"position"}),
    "torque_ramp": ("TORQUE_RAMP", {"torque"}),
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


def resolve(root, path):
    if not isinstance(path, str) or not PATH.fullmatch(path):
        raise ValueError("Invalid dotted property path")
    parts = path.split(".")
    for part in parts[:-1]:
        root = getattr(root, part)
    return root, parts[-1]


def unit_key(path):
    parts = path.split(".")
    return ".".join(parts[1:]) if parts[0] in ("axis0", "axis1") else path


def input_enum(name):
    # Legacy odrive.enums exposes INPUT_MODE_* constants.
    symbol = "INPUT_MODE_" + INPUTS[name][0]
    if not hasattr(enums, symbol):
        raise ValueError(f"Installed odrive library does not expose {symbol}")
    return getattr(enums, symbol)


class USBBackend:
    """Constructed and used exclusively by USBWorker."""

    def __init__(self, params):
        self.units = dict(UNIT_RULES)
        extensions = json.loads(params["unit_rules_json"])
        if not isinstance(extensions, dict):
            raise ValueError("unit_rules_json must be an object")

        for path, rule in extensions.items():
            if not PATH.fullmatch(path):
                raise ValueError("Invalid unit-rule path")
            key = unit_key(path)
            if key in self.units:
                raise ValueError(f"Cannot override built-in unit rule: {path}")
            factor = finite(rule["native_to_si"])
            unit = rule["unit"]
            if factor <= 0 or not isinstance(unit, str) or not unit:
                raise ValueError("Expected positive factor and nonempty unit")
            self.units[key] = (factor, unit)

        self.device = odrive.find_any(
            serial_number=params["serial_number"] or None,
            timeout=params["connect_timeout_s"],
        )
        self.axis = getattr(self.device, f"axis{params['axis']}")
        self.controller = self.axis.controller
        self.serial = f"{self.device.serial_number:X}"

        # Discover actual configuration; do not modify it on connection.
        self.mode = self.read_mode()
        self.input_mode = self.read_input_mode()

        spec = json.loads(params["telemetry_json"])
        if not isinstance(spec, dict):
            raise ValueError("telemetry_json must be an object")
        self.telemetry = []
        for alias, item in spec.items():
            if "scale" in item:
                raise ValueError("Use unit_rules_json instead of telemetry scale")
            scope = item.get("scope", "axis")
            if scope not in ("axis", "device"):
                raise ValueError("Telemetry scope must be axis or device")
            path = item["path"]
            root = self.axis if scope == "axis" else self.device
            parent, name = resolve(root, path)
            self.to_si(path, getattr(parent, name))
            self.telemetry.append((alias, parent, name, path))

    def read_mode(self):
        actual = self.controller.config.control_mode
        for name, (symbol, _, _, _) in MODES.items():
            if actual == getattr(enums, symbol):
                return name
        raise ValueError(f"Unsupported device control mode: {actual}")

    def read_input_mode(self):
        actual = self.controller.config.input_mode
        for name in INPUTS:
            try:
                if actual == input_enum(name):
                    return name
            except ValueError:
                pass
        return "unsupported"

    def rule(self, path, value):
        if type(value) is bool:
            return 1.0, "bool"
        if type(value) is str:
            return 1.0, "string"
        if type(value) not in (int, float):
            raise ValueError("Only scalar properties are supported")
        key = unit_key(path)
        if key not in self.units:
            raise ValueError(f"Register {path} in unit_rules_json first")
        return self.units[key]

    def to_si(self, path, value):
        factor, unit = self.rule(path, value)
        if type(value) in (int, float):
            value = value if factor == 1.0 else value * factor
            if type(value) is float and not math.isfinite(value):
                value = str(value)
        return {"value": value, "unit": unit}

    def require_idle(self):
        if self.axis.current_state != enums.AXIS_STATE_IDLE:
            raise RuntimeError("Axis must be IDLE")

    def set_mode(self, mode, shaping="passthrough"):
        self.require_idle()
        if mode not in MODES or shaping not in INPUTS:
            raise ValueError("Unknown control or input mode")
        if mode not in INPUTS[shaping][1]:
            raise ValueError(f"{shaping} is incompatible with {mode}")

        control = getattr(enums, MODES[mode][0])
        input_value = input_enum(shaping)

        # Clear feedforward and initialize position at the current angle.
        # These writes happen only on explicit mode selection, while IDLE.
        position = self.axis.encoder.pos_estimate
        self.controller.input_vel = 0.0
        self.controller.input_torque = 0.0
        self.controller.input_pos = position
        self.controller.config.control_mode = control
        self.controller.config.input_mode = input_value

        # Read back rather than assuming configuration writes succeeded.
        self.mode = self.read_mode()
        self.input_mode = self.read_input_mode()
        if self.mode != mode or self.input_mode != shaping:
            raise RuntimeError("Mode configuration readback mismatch")
        return {"mode": self.mode, "input_mode": self.input_mode}

    def command(self, value):
        _, name, factor, _ = MODES[self.mode]
        setattr(self.controller, name, value / factor)

    def status(self):
        return {
            "serial": self.serial,
            "mode": self.mode,
            "input_mode": self.input_mode,
            "command_unit": MODES[self.mode][3],
            "state": self.axis.current_state,
            "errors": {
                "axis": self.axis.error,
                "motor": self.axis.motor.error,
                "encoder": self.axis.encoder.error,
                "controller": self.controller.error,
            },
        }

    def set_config(self, path, value):
        if not isinstance(path, str) or not PATH.fullmatch(path):
            raise ValueError("Invalid path")
        parts = path.split(".")
        if "config" not in parts[:-1]:
            raise ValueError("set supports configuration properties only")

        # Keep cached command semantics synchronized through set_mode only.
        key = unit_key(path)
        if key in (
            "controller.config.control_mode",
            "controller.config.input_mode",
        ):
            raise ValueError("Use set_mode or the mode-selection services")

        affected = (
            (parts[0],) if parts[0] in ("axis0", "axis1")
            else ("axis0", "axis1")
        )
        for name in affected:
            axis = getattr(self.device, name, None)
            if axis is not None and axis.current_state != enums.AXIS_STATE_IDLE:
                raise RuntimeError(f"{name} must be IDLE")

        parent, name = resolve(self.device, path)
        old = getattr(parent, name)
        factor, unit = self.rule(path, old)
        if type(old) is bool:
            if type(value) is not bool:
                raise ValueError("Expected boolean")
            native = value
        elif type(old) is str:
            if type(value) is not str:
                raise ValueError("Expected string")
            native = value
        else:
            if isinstance(value, str) and unit == "enum":
                value = getattr(enums, value)
            native = finite(value) / factor
            if type(old) is int:
                if not native.is_integer():
                    raise ValueError("Expected integer after SI conversion")
                native = int(native)

        setattr(parent, name, native)
        actual = getattr(parent, name)
        verified = (
            math.isclose(actual, native, rel_tol=1e-5, abs_tol=1e-7)
            if type(native) is float else actual == native
        )
        return {
            **self.to_si(path, actual),
            "verified": verified,
            "persistent": False,
        }

    def execute(self, request):
        op = request["op"]
        if op == "idle":
            self.axis.requested_state = enums.AXIS_STATE_IDLE
            return {"requested": "IDLE"}

        if op == "closed_loop":
            self.axis.requested_state = enums.AXIS_STATE_CLOSED_LOOP_CONTROL
            return {"requested": "CLOSED_LOOP_CONTROL"}

        if op == "set_mode":
            return self.set_mode(
                request["mode"], request.get("input_mode", "passthrough")
            )

        if op == "set_input":
            return self.set_mode(self.mode, request["input_mode"])

        if op == "ramped":
            shaping = {
                "position": "trajectory",
                "velocity": "velocity_ramp",
                "torque": "torque_ramp",
            }[self.mode]
            return self.set_mode(self.mode, shaping)

        if op == "move_relative":
            if self.mode != "position" or self.input_mode != "trajectory":
                raise RuntimeError("Relative moves require position/trajectory")
            value = finite(request["value"])
            from_goal = request.get("from_goal", False)
            if type(from_goal) is not bool:
                raise ValueError("from_goal must be boolean")
            move = getattr(self.controller, "move_incremental", None)
            if not callable(move):
                raise RuntimeError("Device does not expose move_incremental")
            move(value / math.tau, from_goal)
            return {"increment_rad": value, "from_goal": from_goal}

        if op == "status":
            return self.status()

        if op == "get":
            path = request["path"]
            parent, name = resolve(self.device, path)
            return self.to_si(path, getattr(parent, name))

        if op == "set":
            return self.set_config(request["path"], request["value"])

        raise ValueError(f"Unsupported operation: {op}")

    def sample(self):
        began = time.monotonic()
        values = {
            alias: self.to_si(path, getattr(parent, name))
            for alias, parent, name, path in self.telemetry
        }
        state = self.axis.current_state
        error = self.axis.error
        finished = time.monotonic()
        return {
            "monotonic_s": finished,
            "read_duration_s": finished - began,
            "mode": self.mode,
            "input_mode": self.input_mode,
            "command_unit": MODES[self.mode][3],
            "state": state,
            "axis_error": error,
            "values": values,
        }


@dataclass
class Job:
    request: dict
    service: bool = False
    done: threading.Event = field(default_factory=threading.Event)
    result: dict = field(default_factory=dict)


class USBWorker:
    """Only this object's thread may access USBBackend."""

    def __init__(self, params, logger):
        self.params = params
        self.logger = logger
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.jobs = deque()
        self.responses = queue.SimpleQueue()
        self.pending = None
        self.feedback = None
        self.ready = False
        self.fault = None
        self.thread = threading.Thread(target=self.run, daemon=True)

    def submit(self, request, service=False):
        job = Job(request, service)
        with self.lock:
            if self.fault:
                raise RuntimeError(self.fault)
            if not self.ready or self.stop.is_set():
                raise RuntimeError("USB worker is not ready")
            if len(self.jobs) >= 32:
                raise RuntimeError("Administrative queue is full")
            if request.get("op") == "idle":
                self.pending = None
                self.jobs.appendleft(job)
            else:
                self.jobs.append(job)
        return job

    def command(self, value):
        with self.lock:
            if self.fault:
                raise RuntimeError(self.fault)
            if not self.ready or self.stop.is_set():
                raise RuntimeError("USB worker is not ready")
            self.pending = value

    def complete(self, job, ok, **data):
        job.result = {"id": job.request.get("id"), "ok": ok, **data}
        if not job.service:
            self.responses.put(job.result)
        job.done.set()

    def run(self):
        backend = None
        current = None
        try:
            self.logger.info("Connecting to ODrive...")
            backend = USBBackend(self.params)
            with self.lock:
                self.ready = True
            self.logger.info(
                f"Connected {backend.serial}; "
                f"mode={backend.mode}; input={backend.input_mode}"
            )

            period = 1.0 / self.params["rate_hz"]
            deadline = time.monotonic()
            cycles = overruns = 0
            max_work = 0.0

            while not self.stop.is_set():
                began = time.monotonic()
                with self.lock:
                    current = self.jobs.popleft() if self.jobs else None
                    command = self.pending
                    self.pending = None

                # Administrative transitions form a command barrier.
                # Do not reinterpret an old float after changing its units.
                barrier = current is not None and current.request["op"] in (
                    "idle", "set_mode", "set_input", "ramped", "move_relative"
                )

                if command is not None and not barrier:
                    backend.command(command)

                if current is not None:
                    try:
                        result = backend.execute(current.request)
                        self.complete(current, True, result=result)
                    except Exception as exc:
                        self.complete(current, False, error=str(exc))
                    finally:
                        if barrier:
                            with self.lock:
                                self.pending = None
                        current = None

                snapshot = backend.sample()
                work = time.monotonic() - began
                cycles += 1
                overruns += int(work > period)
                max_work = max(max_work, work)
                snapshot["timing"] = {
                    "cycles": cycles,
                    "work_s": work,
                    "max_work_s": max_work,
                    "work_overruns": overruns,
                }
                with self.lock:
                    self.feedback = snapshot

                deadline += period
                now = time.monotonic()
                if deadline < now:
                    deadline = now
                self.stop.wait(max(0.0, deadline - now))

        except Exception as exc:
            with self.lock:
                self.fault = f"{type(exc).__name__}: {exc}"
            self.logger.error(f"USB worker stopped: {self.fault}")

        finally:
            with self.lock:
                self.ready = False
                remaining = list(self.jobs)
                self.jobs.clear()
                reason = self.fault or "USB worker stopped"
            if current is not None and not current.done.is_set():
                self.complete(current, False, error=reason)
            for job in remaining:
                self.complete(job, False, error=reason)
            if backend is not None:
                try:
                    backend.execute({"op": "idle"})
                except Exception as exc:
                    self.logger.error(
                        f"Shutdown IDLE failed: {exc}; use independent stop"
                    )

    def close(self):
        self.stop.set()
        self.thread.join()


class ODriveDriver(Node):
    def __init__(self):
        super().__init__("odrive_driver")
        defaults = {
            "serial_number": "",
            "axis": 0,
            "connect_timeout_s": 20.0,
            "rate_hz": 100.0,
            "log_hz": 1.0,
            "unit_rules_json": "{}",
            "telemetry_json": json.dumps({
                "position_rad": {"path": "encoder.pos_estimate"},
                "velocity_rad_s": {"path": "encoder.vel_estimate"},
            }),
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        params = {
            name: self.get_parameter(name).value for name in defaults
        }
        params["rate_hz"] = finite(params["rate_hz"])
        params["connect_timeout_s"] = finite(params["connect_timeout_s"])
        self.log_hz = finite(params["log_hz"])
        if params["axis"] not in (0, 1):
            raise ValueError("axis must be 0 or 1")
        if (
            params["rate_hz"] <= 0
            or params["connect_timeout_s"] <= 0
            or self.log_hz < 0
        ):
            raise ValueError("Invalid rate or timeout")

        latest = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.response_pub = self.create_publisher(String, "~/response", 32)
        self.feedback_pub = self.create_publisher(String, "~/feedback", latest)
        self.create_subscription(Float64, "~/command", self.on_command, latest)
        self.create_subscription(String, "~/request", self.on_request, 32)

        # Service waiting cannot block command reception/publication.
        # Only one administrative service callback waits at a time.
        self.service_group = MutuallyExclusiveCallbackGroup()
        self._driver_services = []
        actions = {
            "set_idle": {"op": "idle"},
            "set_closed_loop": {"op": "closed_loop"},
            "set_vel_mode": {"op": "set_mode", "mode": "velocity"},
            "set_position_mode": {"op": "set_mode", "mode": "position"},
            "set_torque_mode": {"op": "set_mode", "mode": "torque"},
            "set_passthrough": {
                "op": "set_input", "input_mode": "passthrough"
            },
            "set_ramped": {"op": "ramped"},
            "set_position_filter": {
                "op": "set_input", "input_mode": "position_filter"
            },
            "get_status": {"op": "status"},
        }
        for name, action in actions.items():
            self._driver_services.append(self.create_service(
                Trigger, "~/" + name, self.service_callback(action),
                callback_group=self.service_group,
            ))

        self.last_stamp = None
        self.last_log = 0.0
        self.worker = USBWorker(params, self.get_logger())
        self.create_timer(1.0 / params["rate_hz"], self.publish_cached)
        self.worker.thread.start()

    def service_callback(self, action):
        def callback(request, response):
            try:
                job = self.worker.submit(dict(action), service=True)
                # Worker completion includes USB execution, not just queuing.
                job.done.wait()
                response.success = job.result["ok"]
                response.message = json.dumps(job.result, allow_nan=False)
            except Exception as exc:
                response.success = False
                response.message = str(exc)
            return response
        return callback

    def on_command(self, message):
        try:
            self.worker.command(finite(message.data))
        except Exception as exc:
            self.get_logger().error(f"Command rejected: {exc}")

    def on_request(self, message):
        request = {}
        try:
            request = json.loads(message.data)
            if not isinstance(request, dict) or not isinstance(request.get("op"), str):
                raise ValueError("Expected a JSON object with string op")
            self.worker.submit(request)
        except Exception as exc:
            self.worker.responses.put({
                "id": request.get("id") if isinstance(request, dict) else None,
                "ok": False,
                "error": str(exc),
            })

    @staticmethod
    def publish_json(publisher, value):
        publisher.publish(String(data=json.dumps(value, allow_nan=False)))

    def publish_cached(self):
        for _ in range(32):
            try:
                response = self.worker.responses.get_nowait()
            except queue.Empty:
                break
            self.publish_json(self.response_pub, response)

        with self.worker.lock:
            snapshot = self.worker.feedback
        if snapshot is None or snapshot["monotonic_s"] == self.last_stamp:
            return

        self.last_stamp = snapshot["monotonic_s"]
        self.publish_json(self.feedback_pub, snapshot)
        now = time.monotonic()
        if self.log_hz and now - self.last_log >= 1.0 / self.log_hz:
            self.last_log = now
            self.get_logger().info(json.dumps(snapshot, allow_nan=False))


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
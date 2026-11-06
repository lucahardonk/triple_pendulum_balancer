#!/usr/bin/env python3
"""Joystick velocity/torque test bridge for the ODrive CAN driver (odrive_can)."""

import math

from odrive_can.msg import ControlMessage
from odrive_can.srv import AxisState
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy

AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP = 8
CONTROL_MODE = {"torque": 1, "velocity": 2}
INPUT_MODE_PASSTHROUGH = 1
STATE_NAMES = {AXIS_STATE_IDLE: "IDLE", AXIS_STATE_CLOSED_LOOP: "CLOSED_LOOP"}


class JoystickFirstTest(Node):
    def __init__(self):
        super().__init__("joystick_first_test_node")

        self.declare_parameter("max_velocity", 10.0)
        self.declare_parameter("max_torque", 10.0)
        self.declare_parameter("deadzone", 0.05)

        self.max_velocity = float(self.get_parameter("max_velocity").value)
        self.max_torque = float(self.get_parameter("max_torque").value)
        self.deadzone = float(self.get_parameter("deadzone").value)

        if not all(
            math.isfinite(x) and x > 0.0
            for x in (self.max_velocity, self.max_torque)
        ):
            raise ValueError("I massimi devono essere finiti e positivi")
        if not math.isfinite(self.deadzone) or not 0.0 <= self.deadzone < 1.0:
            raise ValueError("deadzone deve essere tra 0 incluso e 1 escluso")

        self.axis_value = 0.0
        self.previous_buttons = [False] * 4
        self.mode = None
        self.enabled = False
        self.idle_acknowledged = False
        self.busy = False
        self.pending_idle = False
        self.joy_seen = False
        self.invalid_joy_reported = False

        # Commands go to control_mux (priority 2, below the safety node),
        # never directly to /odrive_axis0/control_message.
        self.publisher = self.create_publisher(
            ControlMessage, "/control/joystick", 10
        )
        self.subscription = self.create_subscription(
            Joy, "/joy", self.on_joy, qos_profile_sensor_data
        )

        self.axis_client = self.create_client(
            AxisState, "/odrive_axis0/request_axis_state"
        )

        self.timer = self.create_timer(0.01, self.publish_command)

        self.get_logger().info(
            "PRONTO, invio disabilitato. "
            "Asse 0 | tasto 0: IDLE | 3: closed-loop | "
            "2: velocita | 1: coppia"
        )
        self.get_logger().info(
            f"Massimi: {self.max_velocity:g} rad/s, "
            f"{self.max_torque:g} N*m | deadzone: {self.deadzone:g}. "
            "Nessun timeout joystick."
        )
        self.get_logger().info(
            "Sequenza: 0 IDLE -> 2 velocita o 1 coppia -> "
            "stick al centro -> 3 closed-loop."
        )

    def on_joy(self, msg):
        if not self.joy_seen:
            self.joy_seen = True
            self.get_logger().info("Joystick ricevuto su /joy")

        buttons = [
            bool(msg.buttons[i]) if i < len(msg.buttons) else False
            for i in range(4)
        ]
        pressed = [
            current and not previous
            for current, previous in zip(buttons, self.previous_buttons)
        ]
        self.previous_buttons = buttons

        valid_axis = bool(msg.axes) and math.isfinite(msg.axes[0])
        if valid_axis:
            raw = max(-1.0, min(1.0, float(msg.axes[0])))
            self.axis_value = 0.0 if abs(raw) <= self.deadzone else raw
            self.invalid_joy_reported = False
        else:
            self.axis_value = 0.0
            if not self.invalid_joy_reported:
                self.get_logger().error(
                    "Asse 0 mancante/non finito: comando impostato a zero"
                )
                self.invalid_joy_reported = True

        # IDLE has priority, including while another service is pending.
        if pressed[0]:
            self.request_idle()
            return

        # Holding IDLE prevents all other button actions.
        if buttons[0]:
            return

        if not any(pressed[1:]):
            return

        if self.busy:
            self.get_logger().warning(
                "Servizio in corso: pressione ignorata; ripremere dopo il risultato"
            )
            return

        # Reject ambiguous simultaneous mode/enable requests.
        if sum(pressed[1:]) > 1:
            self.get_logger().warning("Premere un solo tasto di modalita/avvio")
            return

        if pressed[2]:
            self.select_mode("velocity")
        elif pressed[1]:
            self.select_mode("torque")
        elif pressed[3]:
            self.request_closed_loop(valid_axis)

    def send(self, mode, value):
        msg = ControlMessage(
            control_mode=CONTROL_MODE[mode],
            input_mode=INPUT_MODE_PASSTHROUGH,
        )
        if mode == "velocity":
            msg.input_vel = value / math.tau  # rad/s -> giri/s
        else:
            msg.input_torque = value  # N*m
        self.publisher.publish(msg)

    def send_zero(self):
        self.send(self.mode or "velocity", 0.0)

    def request_idle(self):
        self.enabled = False
        self.idle_acknowledged = False
        self.send_zero()

        if self.busy:
            self.pending_idle = True
            self.get_logger().warning(
                "Invio disabilitato; IDLE accodato dopo il servizio in corso"
            )
            return

        self.call_service(AXIS_STATE_IDLE, self.on_idle_success)

    def on_idle_success(self):
        self.idle_acknowledged = True
        self.get_logger().info(
            "IDLE confermato dall'heartbeat; invio disabilitato."
        )

    def select_mode(self, mode):
        if self.enabled or not self.idle_acknowledged:
            self.get_logger().warning(
                "Prima premere 0 e attendere il risultato IDLE"
            )
            return

        # With odrive_can the control mode travels with every setpoint:
        # a zero setpoint while IDLE configures the mode on the ODrive.
        self.mode = mode
        self.send_zero()
        unit = "rad/s" if mode == "velocity" else "N*m"
        self.get_logger().info(
            f"Modalita selezionata: {mode}, comandi in {unit}. "
            "Centrare lo stick e premere 3."
        )

    def request_closed_loop(self, valid_axis):
        if self.enabled:
            self.get_logger().info("Richiesta closed-loop gia accettata")
            return

        if not self.idle_acknowledged or self.mode is None:
            self.get_logger().warning(
                "Prima richiedere IDLE e selezionare velocita o coppia"
            )
            return

        if not valid_axis or self.axis_value != 0.0:
            self.get_logger().warning(
                "Closed-loop rifiutato dal bridge: centrare lo stick"
            )
            return

        self.send_zero()
        self.call_service(AXIS_STATE_CLOSED_LOOP, self.on_closed_loop_success)

    def on_closed_loop_success(self):
        # Stick may have moved while the service was pending.
        if self.axis_value != 0.0:
            self.get_logger().warning(
                "Stick mosso durante l'attivazione: richiesta IDLE"
            )
            self.pending_idle = True
            return

        self.idle_acknowledged = False
        self.enabled = True
        self.get_logger().info(
            "CLOSED-LOOP confermato dall'heartbeat, invio a 100 Hz abilitato."
        )

    def call_service(self, state, on_success):
        name = STATE_NAMES[state]
        if not self.axis_client.service_is_ready():
            self.get_logger().error(
                "Servizio /odrive_axis0/request_axis_state non disponibile; "
                "ripremere"
            )
            return

        self.busy = True
        self.get_logger().info(f"Richiesta: {name}")

        try:
            future = self.axis_client.call_async(
                AxisState.Request(axis_requested_state=state)
            )
        except Exception as exc:
            self.busy = False
            self.get_logger().error(f"{name}: {exc}")
            return

        def done(result_future):
            try:
                result = result_future.result()
                info = (
                    f"axis_state={result.axis_state}, "
                    f"errori=0x{result.active_errors:X}"
                )
                if result.axis_state == state:
                    self.get_logger().info(f"{name}: OK | {info}")
                    if not self.pending_idle:
                        on_success()
                else:
                    self.get_logger().error(f"{name}: FALLITO | {info}")
            except Exception as exc:
                self.get_logger().error(f"{name}: errore | {exc}")
            finally:
                self.busy = False
                if self.pending_idle:
                    self.pending_idle = False
                    self.request_idle()

        future.add_done_callback(done)

    def publish_command(self):
        if not self.enabled or self.busy or self.mode is None:
            return

        scale = (
            self.max_velocity if self.mode == "velocity" else self.max_torque
        )
        self.send(self.mode, self.axis_value * scale)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickFirstTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
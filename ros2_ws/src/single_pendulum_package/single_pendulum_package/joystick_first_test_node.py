#!/usr/bin/env python3
"""Joystick velocity/torque test bridge for the ODrive driver."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from std_srvs.srv import Trigger


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

        self.publisher = self.create_publisher(
            Float64, "/odrive_driver/command", qos_profile_sensor_data
        )
        self.subscription = self.create_subscription(
            Joy, "/joy", self.on_joy, qos_profile_sensor_data
        )

        self._service_clients = {
            name: self.create_client(Trigger, f"/odrive_driver/{name}")
            for name in (
                "set_idle",
                "set_closed_loop",
                "set_vel_mode",
                "set_torque_mode",
            )
        }

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

    def send_zero(self):
        self.publisher.publish(Float64(data=0.0))

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

        self.call_service("set_idle", self.on_idle_success)

    def on_idle_success(self):
        self.idle_acknowledged = True
        self.get_logger().info(
            "Richiesta IDLE accettata; invio disabilitato. "
            "Lo stato fisico non e verificato da questo bridge."
        )

    def select_mode(self, mode):
        if self.enabled or not self.idle_acknowledged:
            self.get_logger().warning(
                "Prima premere 0 e attendere il risultato IDLE"
            )
            return

        self.mode = None
        service = "set_vel_mode" if mode == "velocity" else "set_torque_mode"

        def success():
            self.mode = mode
            unit = "rad/s" if mode == "velocity" else "N*m"
            self.get_logger().info(
                f"Modalita selezionata: {mode}, comandi in {unit}. "
                "Centrare lo stick e premere 3."
            )

        self.call_service(service, success)

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

        self.call_service("set_closed_loop", self.on_closed_loop_success)

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
            "Richiesta CLOSED-LOOP accettata, invio a 100 Hz abilitato. "
            "Il raggiungimento dello stato non e verificato."
        )

    def call_service(self, name, on_success):
        client = self._service_clients[name]
        if not client.service_is_ready():
            self.get_logger().error(
                f"Servizio /odrive_driver/{name} non disponibile; ripremere"
            )
            return

        self.busy = True
        self.get_logger().info(f"Richiesta: {name}")

        try:
            future = client.call_async(Trigger.Request())
        except Exception as exc:
            self.busy = False
            self.get_logger().error(f"{name}: {exc}")
            return

        def done(result_future):
            try:
                result = result_future.result()
                if result.success:
                    self.get_logger().info(f"{name}: OK | {result.message}")
                    if not self.pending_idle:
                        on_success()
                else:
                    self.get_logger().error(
                        f"{name}: FALLITO | {result.message}"
                    )
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
        self.publisher.publish(Float64(data=self.axis_value * scale))


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
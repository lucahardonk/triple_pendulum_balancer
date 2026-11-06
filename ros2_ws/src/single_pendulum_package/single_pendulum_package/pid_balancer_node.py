#!/usr/bin/env python3
"""pid_balancer_node: PID in cascata per il pendolo inverso singolo (porting di 2_PID_balancer.py).

Ingressi
  /encoders_state   encoders_package/EncoderState  encoder del pendolo (campo `bottom`), ~1 kHz
  /pendulum_status  pendulum_safety_package/PendulumStatus  carrello [m] dal centro, 200 Hz
                    (pubblicato da overrun_node solo a calibrazione finita)
  /control_mux/active  std_msgs/String  chi comanda il motore in questo momento
Uscita
  /control/controller  odrive_can/ControlMessage, controllo in VELOCITA' [giri/s]
                       (priorita' 3 del control_mux: safety e joystick vincono sempre)

Homing, centro, barriere e IDLE sono compito di overrun_node: questo nodo non chiama mai
il servizio request_axis_state e non parla direttamente con l'ODrive.

Stati
  ZEROING   -> pendolo APPESO e fermo per 0.5 s: quell'angolo diventa 0 (su = 180 deg)
  WAIT_CART -> aspetta /pendulum_status (overrun_node in VIGILANT)
  READY     -> silenzioso; cattura quando l'asta e' entro catch_angle dalla verticale
  BALANCING -> inner: inclinazione -> velocita' carrello; outer: posizione -> inclinazione
  STOPPING  -> velocita' 0 per stop_time s, poi silenzio (il mux torna a coppia 0)
  FALLEN    -> silenzioso; torna READY quando il pendolo e' di nuovo appeso
"""

import math
import time

from encoders_package.msg import EncoderState
from odrive_can.msg import ControlMessage
from pendulum_safety_package.msg import PendulumStatus
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String

CONTROL_MODE_VELOCITY = 2
INPUT_MODE_PASSTHROUGH = 1
STALE_S = 0.1  # dato piu' vecchio di cosi' = sensore perso
MUX_GRACE_S = 0.3  # tempo concesso al mux per passare a "controller"


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def wrap_pi(a):
    """Angolo in [-pi, pi)."""
    return (a + math.pi) % math.tau - math.pi


class PID:
    """PID con derivata sulla misura e anti-windup a clamping (identico allo script)."""

    def __init__(self, kp, ki, kd, out_limit):
        self.kp, self.ki, self.kd, self.out_limit = kp, ki, kd, out_limit
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def update(self, error, meas_rate, dt):
        u_unsat = self.kp * error + self.ki * self.integral + self.kd * meas_rate
        u = clamp(u_unsat, -self.out_limit, self.out_limit)
        if self.ki != 0.0 and (u == u_unsat or (u_unsat > 0) != (error > 0)):
            self.integral += error * dt
        return u


class PidBalancer(Node):
    def __init__(self):
        super().__init__("pid_balancer_node")
        p = self.declare_parameter
        # Default = comando usato con lo script (README di single_pendulum_test)
        self.kp_in = p("kp_in", 5.0).value  # [m/s / rad]
        self.ki_in = p("ki_in", 30.0).value  # [m/s^2 / rad], deve essere > 9.81
        self.kd_in = p("kd_in", 0.15).value  # [m / rad]
        self.kp_out = p("kp_out", 0.4).value  # [rad / m]
        self.ki_out = p("ki_out", 0.05).value  # [rad / (m*s)]
        self.kd_out = p("kd_out", 0.1).value  # [rad / (m/s)]
        self.max_tilt = math.radians(p("max_tilt_deg", 6.0).value)
        self.catch = math.radians(p("catch_angle_deg", 10.0).value)
        self.fall = math.radians(p("fall_angle_deg", 35.0).value)
        self.max_vel = p("max_vel", 10.0).value  # [giri/s] <= vel_limit dell'ODrive!
        self.m_per_turn = p("meters_per_turn", 0.06).value  # HTD 5M x 12 denti
        self.soft_limit = p("soft_limit_m", 0.35).value  # < barriere overrun_node (~0.43)
        self.recenter = p("recenter_rate_m_s", 0.018).value
        self.rate_hz = p("rate_hz", 200.0).value
        self.outer_div = int(p("outer_div", 2).value)  # outer a rate_hz/outer_div
        self.angle_sign = int(p("angle_sign", 0).value)  # +1/-1, 0 = da determinare
        self.thetadot_sign = int(p("thetadot_sign", 1).value)
        self.stop_time = p("stop_time", 0.3).value
        log_file = p("log_file", "").value

        if self.ki_in <= 9.81:
            raise ValueError(
                "ki_in deve essere > 9.81: in velocita' e' il termine che regge la gravita'"
            )
        if self.angle_sign not in (-1, 0, 1) or self.thetadot_sign not in (-1, 1):
            raise ValueError("angle_sign in {-1,0,1}, thetadot_sign in {-1,1}")
        if self.outer_div < 1 or self.rate_hz <= 0:
            raise ValueError("outer_div >= 1 e rate_hz > 0")

        self.inner = PID(self.kp_in, self.ki_in, self.kd_in, float("inf"))
        self.outer = PID(self.kp_out, self.ki_out, self.kd_out, self.max_tilt)

        # misure
        self.raw = None  # angolo encoder grezzo [rad]
        self.raw_dot = 0.0
        self.enc_t = 0.0
        self.pos = None  # carrello [m] dal centro
        self.vel = 0.0
        self.cart_t = 0.0
        self.mux_active = "?"
        self.mux_t = 0.0
        self.zero = 0.0
        self.zero_buf = []  # campioni per l'azzeramento automatico
        self.check = [0, 0]  # verifica segno thetadot: [concordi, discordi]
        self.prev = None

        # stato
        self.state = None
        self.state_t = time.monotonic()
        self.pos_sp = 0.0
        self.outer_corr = 0.0
        self.vel_cmd = 0.0  # ultimo comando [giri/s]
        self.loop_n = 0
        self.last_t = None
        self.last_log = 0.0
        self.log = None
        if log_file:
            self.log = open(log_file, "w")
            self.log.write("t,state,phi_deg,phi_dot,pos,vel,pos_sp,tilt_deg,vel_cmd\n")

        self.pub = self.create_publisher(ControlMessage, "/control/controller", 10)
        self.create_subscription(
            EncoderState, "/encoders_state", self.on_encoder, qos_profile_sensor_data
        )
        self.create_subscription(
            PendulumStatus,
            "/pendulum_status",
            self.on_cart,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.create_subscription(
            String,
            "/control_mux/active",
            self.on_mux,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_timer(0.05, self.watchdog)

        self.get_logger().info(
            f"inner Kp={self.kp_in} Ki={self.ki_in} Kd={self.kd_in} | outer Kp={self.kp_out} "
            f"Ki={self.ki_out} Kd={self.kd_out} | "
            f"tilt max {math.degrees(self.max_tilt):.1f} deg | {self.rate_hz:.0f} Hz | "
            f"max_vel {self.max_vel} giri/s | soft limit +/-{self.soft_limit} m"
        )
        self.go("ZEROING", "lascia il pendolo APPESO e FERMO")

    # ------------------------------------------------------------------ ingressi
    def on_encoder(self, m):
        now = time.monotonic()
        self.raw, self.raw_dot, self.enc_t = m.bottom.position, m.bottom.velocity, now
        # Fuori dal balance: velocity ha lo stesso segno di d(position)/dt?
        if self.state != "BALANCING" and self.prev is not None and abs(self.raw_dot) > 1.0:
            dt = now - self.prev[1]
            if 1e-4 < dt < 0.01:
                d = wrap_pi(self.raw - self.prev[0]) / dt
                if abs(d) > 0.5:
                    self.check[(d > 0) != (self.raw_dot > 0)] += 1
        self.prev = (self.raw, now)
        # Il controllo gira QUI, decimato a rate_hz: niente timer veloce in competizione
        # con i messaggi a 1 kHz nello stesso executor (latenza minima, nessuna starvation)
        if self.last_t is None or now - self.last_t >= 0.9 / self.rate_hz:
            self.step()

    def watchdog(self):
        """Timer lento: se l'encoder tace, step() non viene piu' chiamato da on_encoder."""
        if self.raw is None or self.since(self.enc_t) >= STALE_S:
            self.step()

    def on_cart(self, m):
        self.pos, self.vel, self.cart_t = m.position_m, m.velocity_m_s, time.monotonic()

    def on_mux(self, m):
        self.mux_active, self.mux_t = m.data, time.monotonic()

    # ------------------------------------------------------------------ utilita'
    def go(self, state, info):
        self.get_logger().info(f"{self.state or '-'} -> {state} | {info}")
        self.state, self.state_t = state, time.monotonic()

    def since(self, t):
        return time.monotonic() - t

    def tilt(self):
        """Inclinazione dalla verticale alta [rad], segno dell'encoder (non ancora angle_sign)."""
        return wrap_pi(self.raw - self.zero - math.pi)

    def send_vel(self, turns_s):
        self.pub.publish(
            ControlMessage(
                control_mode=CONTROL_MODE_VELOCITY,
                input_mode=INPUT_MODE_PASSTHROUGH,
                input_vel=float(turns_s),
                input_torque=0.0,
            )
        )

    def throttle(self, period, text, warn=False):
        if self.since(self.last_log) >= period:
            self.last_log = time.monotonic()
            # rclpy vieta severita' diverse dalla stessa riga: due chiamate separate
            if warn:
                self.get_logger().warning(text)
            else:
                self.get_logger().info(text)

    # ------------------------------------------------------------------ macchina a stati
    def step(self):
        now = time.monotonic()
        dt = clamp(now - self.last_t, 0.0, 0.02) if self.last_t else 1.0 / self.rate_hz
        self.last_t = now
        enc_ok = self.raw is not None and self.since(self.enc_t) < STALE_S
        cart_ok = self.pos is not None and self.since(self.cart_t) < STALE_S

        if self.state == "ZEROING":
            if not enc_ok:
                return self.throttle(
                    1.0, "aspetto /encoders_state (encoders_driver avviato?)", True
                )
            if abs(self.raw_dot) > 0.3:
                self.zero_buf.clear()
                return self.throttle(
                    1.0, f"pendolo in movimento ({self.raw_dot:+.2f} rad/s): fermalo appeso", True
                )
            self.zero_buf.append(self.raw)
            if len(self.zero_buf) >= int(0.5 * self.rate_hz):
                sx = sum(math.cos(a) for a in self.zero_buf)
                sy = sum(math.sin(a) for a in self.zero_buf)
                self.zero = math.atan2(sy, sx) % math.tau
                self.go("WAIT_CART", f"zero: appeso = {math.degrees(self.zero):.2f} deg grezzi")
            return

        if not enc_ok and self.state == "BALANCING":
            return self.abort("encoder perso")
        if not enc_ok and self.state == "READY":
            return self.throttle(1.0, "encoder fermo da piu' di 0.1 s", True)

        if self.state == "WAIT_CART":
            if cart_ok:
                self.go(
                    "READY",
                    f"carrello a {self.pos:+.3f} m. Solleva l'asta: cattura entro "
                    f"+/-{math.degrees(self.catch):.0f} deg",
                )
            else:
                self.throttle(
                    2.0, "aspetto /pendulum_status (overrun_node deve arrivare in VIGILANT)"
                )

        elif self.state == "READY":
            self.ready(cart_ok)

        elif self.state == "BALANCING":
            self.balance(cart_ok, dt)

        elif self.state == "STOPPING":
            self.send_vel(0.0)
            if self.since(self.state_t) > self.stop_time:
                self.go("FALLEN", "silenzio sul mux. Riporta il pendolo APPESO per riarmare")

        elif self.state == "FALLEN":
            if enc_ok and abs(self.tilt()) > math.radians(150.0):
                self.go("READY", "pendolo appeso: pronto per una nuova cattura")

        self.write_log(dt)

    def ready(self, cart_ok):
        dev = self.tilt()
        if self.angle_sign == 0:  # aiuto per trovare il verso dell'encoder
            return self.throttle(
                0.5,
                f"angle_sign NON impostato. Tieni l'asta quasi verticale e inclinala verso il "
                f"lato + del carrello (/pendulum_status cresce): inclinazione = "
                f"{math.degrees(dev):+.1f} deg -> se positiva angle_sign:=1, "
                "se negativa angle_sign:=-1",
                True,
            )
        conc, disc = self.check
        if conc + disc >= 200 and disc > 0.9 * (conc + disc) and self.thetadot_sign == 1:
            self.throttle(
                2.0,
                f"velocity dell'encoder ha segno OPPOSTO a d(position)/dt ({disc}/{conc + disc}): "
                "rilancia con thetadot_sign:=-1",
                True,
            )
        if abs(dev) >= self.catch:
            return self.throttle(
                1.0, f"attendo cattura: |inclinazione| = {abs(math.degrees(dev)):.1f} deg"
            )
        if not cart_ok:
            return self.throttle(1.0, "asta in cattura ma /pendulum_status fermo: non parto", True)
        if self.mux_active in ("safety", "joystick"):
            return self.throttle(
                1.0,
                f"asta in cattura ma il motore e' di '{self.mux_active}': non parto "
                "(overrun_node in VIGILANT? joystick spento?)",
                True,
            )
        if abs(self.pos) > 0.8 * self.soft_limit:
            return self.throttle(
                1.0, f"carrello troppo lontano dal centro ({self.pos:+.3f} m): non parto", True
            )
        self.inner.reset()
        self.outer.reset()
        self.outer_corr = 0.0
        self.pos_sp = self.pos
        self.loop_n = 0
        self.go(
            "BALANCING", f"CATTURA a {math.degrees(dev):+.1f} deg, setpoint {self.pos_sp:+.3f} m"
        )

    def balance(self, cart_ok, dt):
        if not cart_ok:
            return self.abort("/pendulum_status perso")
        phi = self.angle_sign * self.tilt()  # >0 = punta verso +
        if abs(phi) > self.fall:
            return self.abort(f"asta caduta ({math.degrees(phi):+.1f} deg)")
        if abs(self.pos) > self.soft_limit:
            return self.abort(f"soft limit: carrello a {self.pos:+.3f} m")
        if self.since(self.state_t) > MUX_GRACE_S and self.mux_active != "controller":
            return self.abort(f"il mux ha dato il motore a '{self.mux_active}'")
        phi_dot = self.angle_sign * self.thetadot_sign * self.raw_dot

        # setpoint che rientra piano verso il centro
        step = self.recenter * dt
        self.pos_sp = clamp(
            self.pos_sp - clamp(self.pos_sp, -step, step),
            -0.8 * self.soft_limit,
            0.8 * self.soft_limit,
        )
        # OUTER: carrello lontano dal setpoint -> l'asta deve pendere verso il setpoint
        if self.loop_n % self.outer_div == 0:
            self.outer_corr = -self.outer.update(
                self.pos - self.pos_sp, self.vel, dt * self.outer_div
            )
        self.loop_n += 1
        # INNER: il carrello va sotto la punta. v = Kp*e + Ki*int(e) + Kd*phi_dot [m/s]
        e = phi - self.outer_corr
        u = self.inner.update(e, phi_dot, dt)
        v_lim = self.max_vel * self.m_per_turn
        if abs(u) > v_lim:  # anti-windup sulla saturazione
            self.inner.integral -= e * dt
            u = math.copysign(v_lim, u)
        self.vel_cmd = u / self.m_per_turn
        self.send_vel(self.vel_cmd)
        self.throttle(
            0.1,
            f"phi={math.degrees(phi):+6.2f} deg  pos={self.pos:+.3f} sp={self.pos_sp:+.3f} m  "
            f"v={u:+.2f} m/s  tilt_rich={math.degrees(self.outer_corr):+.2f} deg",
        )

    def abort(self, why):
        self.get_logger().error(f"STOP balance: {why}")
        self.inner.reset()
        self.outer.reset()
        self.go("STOPPING", f"velocita' 0 per {self.stop_time} s")
        self.send_vel(0.0)

    def write_log(self, dt):
        if self.log is None or self.raw is None or self.pos is None:
            return
        phi = (self.angle_sign or 1) * self.tilt()
        self.log.write(
            f"{time.monotonic():.4f},{self.state},{math.degrees(phi):.3f},{self.raw_dot:.4f},"
            f"{self.pos:.4f},{self.vel:.4f},{self.pos_sp:.4f},{math.degrees(self.outer_corr):.3f},"
            f"{self.vel_cmd if self.state == 'BALANCING' else 0.0:.3f}\n"
        )

    def shutdown(self):
        if self.state in ("BALANCING", "STOPPING"):
            self.send_vel(0.0)  # poi il mux scade e manda coppia 0
        if self.log is not None:
            self.log.close()


def main(args=None):
    rclpy.init(args=args)
    node = PidBalancer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

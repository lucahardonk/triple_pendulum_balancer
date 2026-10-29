#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
single_balancer.py — Bilanciatore pendolo inverso singolo su carrello lineare.

Hardware:
  * Carrello a cinghia mosso da MKS XDrive Mini (firmware ODrive, pacchetto `odrive` legacy)
  * Encoder del pendolo letto da ESP32-C6 -> seriale USB (default /dev/ttyACM0 @ 921600)
    formato riga:  "θ: 179.87° θ̇: 0.012 rad/s"
    convenzione:   0° = pendolo appeso (giù), 180° = pendolo in alto

Flusso:
  STARTUP -> HOMING_RIGHT -> HOMING_LEFT -> CENTERING -> READY -> BALANCING
                                                         ^  |       |
                                                         |  v       v
                                                        MANUAL <----+
  In qualsiasi stato: SPAZIO -> EMERGENCY -> IDLE (motore libero), 'r' per riarmare.

Tasti (terminale):
  SPAZIO  = fungo d'emergenza (input_vel=0 -> IDLE)
  r       = reset emergenza (riabilita closed loop -> READY)
  b       = arma il balance (READY: entra in BALANCING appena l'asta è nel cono di cattura)
  m       = passa in MANUAL (balance disattivato)
  q / ESC = uscita pulita (anche Ctrl+C)

Joystick (pygame):
  AXIS 0  = READY/MANUAL: velocità carrello diretta | BALANCING: sposta setpoint posizione
  BTN 0   = stop velocità (READY/MANUAL) / setpoint = posizione attuale (BALANCING)
  BTN 1   = toggle BALANCING <-> MANUAL

Tutti i parametri da tarare sono flag CLI: `python3 single_balancer.py --help`.
"""

import argparse
import math
import os
import re
import select
import sys
import threading
import time

# --------------------------------------------------------------------------------------
# Costanti fisse (non da tarare in campo)
# --------------------------------------------------------------------------------------
JOY_AXIS = 0
JOY_INVERT = 1
JOY_DEADZONE = 0.10
JOY_BTN_STOP = 0
JOY_BTN_TOGGLE = 1

ODRIVE_STATE_IDLE = 1
ODRIVE_STATE_CLOSED_LOOP = 8
CONTROL_MODE_VELOCITY = 2
# ODrive fw 0.5.x/0.6.x: 0 = INACTIVE (input ignorato!), 1 = PASSTHROUGH, 2 = VEL_RAMP
INPUT_MODE_INACTIVE = 0
INPUT_MODE_PASSTHROUGH = 1

PRINT_PERIOD_S = 0.10          # print di stato ogni 100 ms
SIGN_WINDOW_S = 0.8            # homing: finestra di misura del verso dell'encoder pendolo
SIGN_MIN_DEG = 2.0             # picco minimo perché la misura del segno sia valida
SERIAL_STALE_S = 0.10          # dato ESP32 più vecchio di così = sensore perso
CENTER_TOLERANCE_TURNS = 0.5   # arrivo al centro durante CENTERING
EMERGENCY_DECEL_TIMEOUT_S = 0.6
EMERGENCY_STOP_VEL = 0.2       # turns/s sotto cui consideriamo il carrello fermo


# ======================================================================================
# CLI
# ======================================================================================
def parse_args():
    p = argparse.ArgumentParser(
        description="Bilanciatore pendolo inverso singolo (ODrive + ESP32)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    # Comunicazione
    p.add_argument("--port", default="/dev/ttyACM0", help="Porta seriale ESP32")
    p.add_argument("--baud", type=int, default=921600, help="Baud rate ESP32")
    p.add_argument("--axis", type=int, default=0, choices=[0, 1], help="Asse ODrive")
    p.add_argument("--no-auto-zero", action="store_true",
                   help="Disattiva l'azzeramento automatico dell'angolo all'avvio "
                        "(di default il pendolo APPESO e fermo diventa 0°)")
    p.add_argument("--theta-down-raw", type=float, default=0.0,
                   help="Con --no-auto-zero: angolo grezzo ESP32 che corrisponde al pendolo appeso")
    # Calibrazione / homing
    p.add_argument("--rail-turns", type=float, default=None,
                   help="Lunghezza percorribile in turns ODrive. Se data, è usata come range "
                        "ufficiale (il centro resta il punto medio dei finecorsa misurati). "
                        "Ignorato se --rail-meters è fornito.")
    p.add_argument("--rail-meters", type=float, default=None,
                   help="Lunghezza percorribile in metri. Se fornito, durante l'homing viene "
                        "calcolato automaticamente il rapporto turns/m (gear ratio) dalla "
                        "corsa misurata dall'encoder. Sovrascrive --rail-turns.")
    p.add_argument("--turns-per-meter", type=float, default=None,
                   help="Rapporto turns/m noto (stampato a fine homing). Serve solo con "
                        "--skip-homing per convertire --rail-meters in turns.")
    p.add_argument("--soft-fraction", type=float, default=0.80,
                   help="Frazione del semi-range usata come soft limit")
    p.add_argument("--homing-vel", type=float, default=4.0, help="Velocità homing [turns/s]")
    p.add_argument("--stall-window", type=float, default=0.5,
                   help="Finestra di rilevamento stallo [s]")
    p.add_argument("--stall-dist", type=float, default=0.1,
                   help="Spostamento minimo nella finestra per NON essere in stallo [turns]")
    p.add_argument("--breakaway-dist", type=float, default=0.3,
                   help="Homing: turns da percorrere prima di valutare lo stallo "
                        "(serve a staccarsi dal finecorsa)")
    p.add_argument("--breakaway-timeout", type=float, default=3.0,
                   help="Homing: tempo massimo [s] per percorrere --breakaway-dist, "
                        "altrimenti il motore è considerato bloccato")
    p.add_argument("--homing-timeout", type=float, default=30.0,
                   help="Timeout per ogni fase di homing [s]")
    p.add_argument("--skip-homing", action="store_true",
                   help="Salta homing: centro = posizione attuale, richiede --rail-turns")
    # Limiti
    p.add_argument("--catch-angle", type=float, default=15.0,
                   help="Max angolo da verticale per attivare il balance [deg]")
    p.add_argument("--fall-angle", type=float, default=35.0,
                   help="Oltre questo angolo da verticale in BALANCING -> asta caduta [deg]")
    p.add_argument("--max-vel", type=float, default=35.0, help="Velocità max comando [turns/s]")
    p.add_argument("--joy-max-vel", type=float, default=12.0,
                   help="Velocità max joystick in MANUAL/READY [turns/s]")
    p.add_argument("--joy-sp-rate", type=float, default=2.0,
                   help="Velocità di spostamento setpoint con joystick in BALANCING [turns/s]")
    p.add_argument("--recenter-rate", type=float, default=0.3,
                   help="Velocità di rientro automatico del setpoint verso il centro "
                        "in BALANCING a joystick fermo [turns/s] (0 = disabilitato)")
    # PID inner (inclinazione asta -> velocità carrello). Unità SI: convertite in turns
    # con il gear ratio misurato. Con comando in VELOCITÀ il termine Ki è quello che
    # contrasta la gravità: DEVE essere > 9.81 m/s²/rad, altrimenti l'asta cade sempre.
    p.add_argument("--kp-in", type=float, default=6.0, help="Kp inner [m/s / rad]")
    p.add_argument("--ki-in", type=float, default=30.0,
                   help="Ki inner [m/s² / rad] (deve essere > 9.81)")
    p.add_argument("--kd-in", type=float, default=0.0, help="Kd inner [m / rad]")
    # PID outer (posizione carrello -> inclinazione desiderata, 'pendi verso il target')
    p.add_argument("--kp-out", type=float, default=0.07, help="Kp outer [rad / m]")
    p.add_argument("--ki-out", type=float, default=0.0, help="Ki outer [rad / (m*s)]")
    p.add_argument("--kd-out", type=float, default=0.10, help="Kd outer [rad / (m/s)]")
    p.add_argument("--max-tilt", type=float, default=8.0,
                   help="Max inclinazione richiesta dall'outer [deg]")
    p.add_argument("--outer-div", type=int, default=5,
                   help="L'outer gira ogni N iterazioni del loop principale")
    # Segni (misurati automaticamente durante l'homing)
    p.add_argument("--angle-sign", type=int, default=0, choices=[0, 1, -1],
                   help="Verso dell'encoder del pendolo rispetto al carrello. 0 = misurato "
                        "in automatico durante l'homing (obbligatorio ±1 con --skip-homing)")
    p.add_argument("--thetadot-sign", type=int, default=0, choices=[0, 1, -1],
                   help="Segno di θ̇ dell'ESP32 rispetto a dθ/dt. 0 = verificato in automatico "
                        "durante l'homing (con --skip-homing vale +1)")
    # Loop
    p.add_argument("--dt", type=float, default=0.002, help="Periodo loop di controllo [s]")
    p.add_argument("--log", default=None, help="Se dato, salva CSV di tuning in questo file")
    a = p.parse_args()

    if not 0.0 < a.soft_fraction <= 1.0:
        p.error("--soft-fraction deve essere in (0, 1]")
    if a.rail_meters is not None and a.rail_meters <= 0:
        p.error("--rail-meters deve essere > 0")
    if a.turns_per_meter is not None and a.turns_per_meter <= 0:
        p.error("--turns-per-meter deve essere > 0")
    if a.skip_homing:
        # Senza homing serve conoscere la corsa in turns: o diretta, o metri * rapporto
        if a.rail_turns is None and not (a.rail_meters is not None
                                         and a.turns_per_meter is not None):
            p.error("--skip-homing richiede --rail-turns, oppure "
                    "--rail-meters insieme a --turns-per-meter")
    if a.rail_meters is None:
        p.error("serve --rail-meters: i guadagni del balance sono in metri e vanno "
                "convertiti in turns con il gear ratio")
    if a.skip_homing and a.turns_per_meter is None:
        p.error("--skip-homing richiede --turns-per-meter (stampato a fine homing)")
    if a.skip_homing and a.angle_sign == 0:
        p.error("--skip-homing richiede --angle-sign ±1 (stampato a fine homing)")
    if a.ki_in <= 9.81:
        p.error("--ki-in deve essere > 9.81 m/s²/rad: con comando in velocità sotto "
                "questo valore il controllo non può reggere l'asta")
    if a.outer_div < 1:
        p.error("--outer-div deve essere >= 1")
    return a


# ======================================================================================
# Utilità
# ======================================================================================
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def wrap_deg_180(a):
    """Riporta un angolo in gradi nell'intervallo [-180, 180)."""
    return (a + 180.0) % 360.0 - 180.0


def dev_from_top_deg(theta_deg):
    """Deviazione con segno dalla verticale alta (180°), in [-180, 180)."""
    return wrap_deg_180(theta_deg - 180.0)


def clear_errors(odrv, ax):
    """Pulisce gli errori ODrive (compatibile firmware 0.5.x legacy e 0.6.x)."""
    try:
        odrv.clear_errors()          # fw >= 0.6
        return
    except Exception:
        pass
    for obj in (ax, getattr(ax, "motor", None), getattr(ax, "encoder", None),
                getattr(ax, "controller", None), getattr(ax, "sensorless_estimator", None)):
        if obj is None:
            continue
        try:
            obj.error = 0
        except Exception:
            pass


def odrive_error_summary(ax):
    parts = []
    for name in ("", "motor", "encoder", "controller"):
        obj = getattr(ax, name) if name else ax
        try:
            e = int(obj.error)
            if e:
                parts.append("%s=0x%X" % (name or "axis", e))
        except Exception:
            pass
    return ", ".join(parts) if parts else "nessuno"


def odrive_diag(ax):
    """Fotografia dei parametri ODrive utili quando il motore non si muove."""
    items = [
        ("current_state", lambda: ax.current_state),
        ("control_mode", lambda: ax.controller.config.control_mode),
        ("input_mode", lambda: ax.controller.config.input_mode),
        ("input_vel", lambda: ax.controller.input_vel),
        ("vel_setpoint", lambda: ax.controller.vel_setpoint),
        ("vel_limit", lambda: ax.controller.config.vel_limit),
        ("vel_gain", lambda: ax.controller.config.vel_gain),
        ("vel_integrator_gain", lambda: ax.controller.config.vel_integrator_gain),
        ("current_lim", lambda: ax.motor.config.current_lim),
        ("Iq_setpoint", lambda: ax.motor.current_control.Iq_setpoint),
        ("Iq_measured", lambda: ax.motor.current_control.Iq_measured),
        ("encoder.is_ready", lambda: ax.encoder.is_ready),
        ("pos_estimate", lambda: ax.encoder.pos_estimate),
        ("vel_estimate", lambda: ax.encoder.vel_estimate),
    ]
    out = []
    for name, fn in items:
        try:
            v = fn()
            out.append("%s=%s" % (name, ("%.4g" % v) if isinstance(v, float) else v))
        except Exception:
            out.append("%s=n/d" % name)
    return "\n             ".join(out + ["errori: %s" % odrive_error_summary(ax)])


# ======================================================================================
# Thread seriale ESP32
# ======================================================================================
class PendulumSensor(threading.Thread):
    """Legge le righe dell'ESP32 e aggiorna (theta_deg, theta_dot, timestamp) sotto lock."""

    # "θ: 179.87° θ̇: 0.012 rad/s" — prendiamo i due numeri in ordine.
    _NUM = re.compile(r"[-+]?\d+(?:\.\d+)?")

    def __init__(self, port, baud):
        super().__init__(daemon=True)
        import serial  # pyserial
        self.ser = serial.Serial(port, baud, timeout=0.01)
        self.lock = threading.Lock()
        self.theta_deg = None          # angolo GREZZO dall'ESP32
        self.zero_offset = 0.0         # sottratto in get(): appeso -> 0°, su -> 180°
        self.theta_dot = 0.0
        self.stamp = 0.0
        self.n_lines = 0
        self.n_bad = 0
        self.running = True

    def _parse(self, line):
        if "rad/s" not in line:
            return None
        # Separa sulla seconda etichetta per non confondere i numeri
        if "θ̇" in line:
            left, right = line.split("θ̇", 1)
        elif ":" in line[line.find(":") + 1:]:
            i = line.find(":", line.find(":") + 1)
            left, right = line[:i], line[i:]
        else:
            return None
        n1 = self._NUM.findall(left)
        n2 = self._NUM.findall(right)
        if not n1 or not n2:
            return None
        return float(n1[-1]), float(n2[0])

    def run(self):
        buf = b""
        while self.running:
            try:
                chunk = self.ser.read(256)
            except Exception as e:
                if not self.running:       # porta chiusa in uscita: normale
                    break
                print("\r\n[SERIAL   ] errore lettura: %s" % e)
                time.sleep(0.05)
                continue
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("utf-8", errors="ignore").strip()
                res = self._parse(line)
                if res is None:
                    self.n_bad += 1
                    continue
                with self.lock:
                    self.theta_deg, self.theta_dot = res
                    self.stamp = time.monotonic()
                    self.n_lines += 1
            if len(buf) > 4096:     # protezione da stream senza newline
                buf = b""

    def get_raw(self):
        """Come get() ma con l'angolo grezzo dell'ESP32 (senza offset di zero)."""
        with self.lock:
            age = time.monotonic() - self.stamp if self.stamp else float("inf")
            return self.theta_deg, self.theta_dot, age

    def get(self):
        """Ritorna (theta_deg, theta_dot_rad_s, età_dato_s) con 0°=appeso, 180°=su.
        theta_deg=None se mai ricevuto."""
        theta, theta_dot, age = self.get_raw()
        if theta is not None:
            theta = (theta - self.zero_offset) % 360.0
            if theta >= 360.0:             # arrotondamento float di valori ~ -0
                theta = 0.0
        return theta, theta_dot, age

    def stop(self):
        self.running = False
        try:
            self.ser.close()
        except Exception:
            pass


# ======================================================================================
# Thread tastiera (fungo d'emergenza su SPAZIO)
# ======================================================================================
class KeyboardWatcher(threading.Thread):
    """Terminale in modalità cbreak: legge un carattere alla volta senza Invio."""

    def __init__(self):
        super().__init__(daemon=True)
        self.emergency = threading.Event()
        self.quit = threading.Event()
        self.reset_req = threading.Event()
        self.arm_req = threading.Event()
        self.manual_req = threading.Event()
        self.running = True
        self.enabled = sys.stdin.isatty()
        self._old = None

    def run(self):
        if not self.enabled:
            print("[KEYBOARD ] stdin non è un terminale: SPAZIO-emergenza NON disponibile!")
            return
        import termios
        import tty
        fd = sys.stdin.fileno()
        self._old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)   # no echo, no Invio; Ctrl+C genera ancora SIGINT
            while self.running:
                r, _, _ = select.select([sys.stdin], [], [], 0.05)
                if not r:
                    continue
                ch = os.read(fd, 1)
                if ch == b" ":
                    self.emergency.set()
                elif ch in (b"q", b"Q", b"\x1b", b"\x03"):
                    self.quit.set()
                elif ch in (b"r", b"R"):
                    self.reset_req.set()
                elif ch in (b"b", b"B"):
                    self.arm_req.set()
                elif ch in (b"m", b"M"):
                    self.manual_req.set()
        finally:
            self.restore()

    def restore(self):
        if self._old is not None:
            import termios
            try:
                termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self._old)
            except Exception:
                pass
            self._old = None

    def stop(self):
        self.running = False


# ======================================================================================
# Joystick
# ======================================================================================
class Joystick:
    def __init__(self):
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
        os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
        self.js = None
        self.prev_buttons = {}
        try:
            import pygame
            self.pg = pygame
            pygame.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                self.js = pygame.joystick.Joystick(0)
                self.js.init()
                print("[JOYSTICK ] %s connesso" % self.js.get_name())
            else:
                print("[JOYSTICK ] nessun joystick trovato (solo tastiera)")
        except Exception as e:
            self.pg = None
            print("[JOYSTICK ] pygame non disponibile: %s" % e)

    def poll(self):
        """Ritorna (axis in [-1,1] con deadzone, set dei pulsanti appena premuti)."""
        if self.js is None:
            return 0.0, set()
        self.pg.event.pump()
        x = self.js.get_axis(JOY_AXIS) * (-1 if JOY_INVERT else 1)
        if abs(x) < JOY_DEADZONE:
            x = 0.0
        else:  # riscala per partire da 0 al bordo della deadzone
            x = math.copysign((abs(x) - JOY_DEADZONE) / (1.0 - JOY_DEADZONE), x)
        pressed = set()
        for b in (JOY_BTN_STOP, JOY_BTN_TOGGLE):
            if b < self.js.get_numbuttons():
                now = bool(self.js.get_button(b))
                if now and not self.prev_buttons.get(b, False):
                    pressed.add(b)
                self.prev_buttons[b] = now
        return clamp(x, -1.0, 1.0), pressed

    def close(self):
        if self.pg is not None:
            try:
                self.pg.quit()
            except Exception:
                pass


# ======================================================================================
# PID
# ======================================================================================
class PID:
    """PID con derivata sulla misura (passata dall'esterno) e anti-windup a clamping."""

    def __init__(self, kp, ki, kd, out_limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit = out_limit
        self.integral = 0.0
        self.last_out = 0.0

    def reset(self):
        self.integral = 0.0
        self.last_out = 0.0

    def update(self, error, meas_rate, dt):
        """error = misura - setpoint ; meas_rate = derivata della misura."""
        u_unsat = self.kp * error + self.ki * self.integral + self.kd * meas_rate
        u = clamp(u_unsat, -self.out_limit, self.out_limit)
        # Anti-windup: integra solo se non saturo, oppure se l'integrazione riduce la saturazione
        if self.ki != 0.0 and (u == u_unsat or (u_unsat > 0) != (error > 0)):
            self.integral += error * dt
        self.last_out = u
        return u


# ======================================================================================
# Controller principale
# ======================================================================================
class Balancer:
    def __init__(self, args, odrv, sensor, keys, joy):
        self.a = args
        self.odrv = odrv
        self.ax = getattr(odrv, "axis%d" % args.axis)
        self.sensor = sensor
        self.keys = keys
        self.joy = joy

        self.state = "STARTUP"
        self.center = 0.0
        self.half_range = 0.0
        self.soft_limit = 0.0
        self.hard_limit = 0.0
        # Rapporto meccanico turns/m (gear ratio). None = sconosciuto -> si mostra in turns.
        self.turns_per_meter = args.turns_per_meter
        self.pos_sp = 0.0           # setpoint posizione (offset dal centro) in BALANCING
        self.vel_cmd = 0.0
        self.outer_corr = 0.0       # rad
        self.emergency_reason = ""

        # Inner: uscita in m/s (convertita in turns/s in balance_step)
        self.inner = PID(args.kp_in, args.ki_in, args.kd_in, float("inf"))
        self.outer = PID(args.kp_out, args.ki_out, args.kd_out, math.radians(args.max_tilt))
        # Segni: angle_sign porta θ nel riferimento del carrello (φ>0 = punta verso +turns)
        self.angle_sign = args.angle_sign or None
        self.thetadot_sign = args.thetadot_sign or (1 if args.skip_homing else None)
        self._sign_samples = {}

        self._last_print = 0.0
        self._loop_count = 0
        self._rate_t0 = time.monotonic()
        self._rate_n = 0
        self.loop_hz = 0.0

        self.log = None
        if args.log:
            self.log = open(args.log, "w")
            self.log.write("t,state,theta_deg,theta_dot,pos,vel,pos_sp,outer_corr_deg,vel_cmd\n")
        self._t0 = time.monotonic()

    # ---------------------------------------------------------------- ODrive helpers
    def odrive_enable(self):
        ax = self.ax
        # Partenza pulita a ogni avvio: IDLE -> clear errors -> config -> CLOSED_LOOP
        ax.requested_state = ODRIVE_STATE_IDLE
        time.sleep(0.1)
        print("[ODRIVE   ] Errori prima del clear: %s" % odrive_error_summary(ax))
        clear_errors(self.odrv, ax)
        time.sleep(0.05)
        print("[ODRIVE   ] Errori dopo il clear  : %s" % odrive_error_summary(ax))
        ax.controller.config.control_mode = CONTROL_MODE_VELOCITY
        ax.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        ax.controller.config.vel_limit = self.a.max_vel * 1.2
        ax.controller.input_vel = 0.0
        ax.requested_state = ODRIVE_STATE_CLOSED_LOOP
        t0 = time.monotonic()
        while time.monotonic() - t0 < 2.0:
            if int(ax.current_state) == ODRIVE_STATE_CLOSED_LOOP:
                cm = int(ax.controller.config.control_mode)
                im = int(ax.controller.config.input_mode)
                print("[ODRIVE   ] CLOSED_LOOP attivo (asse %d) control_mode=%d input_mode=%d"
                      % (self.a.axis, cm, im))
                if cm != CONTROL_MODE_VELOCITY or im != INPUT_MODE_PASSTHROUGH:
                    print("[ODRIVE   ] Modalità non applicate (attese control_mode=%d "
                          "input_mode=%d)" % (CONTROL_MODE_VELOCITY, INPUT_MODE_PASSTHROUGH))
                    ax.requested_state = ODRIVE_STATE_IDLE
                    return False
                return True
            time.sleep(0.02)
        print("[ODRIVE   ] impossibile entrare in CLOSED_LOOP, errori: %s"
              % odrive_error_summary(ax))
        return False

    def set_vel(self, v):
        self.vel_cmd = clamp(v, -self.a.max_vel, self.a.max_vel)
        self.ax.controller.input_vel = self.vel_cmd

    def read_pos_vel(self):
        return self.ax.encoder.pos_estimate, self.ax.encoder.vel_estimate

    def odrive_dropped_out(self):
        try:
            return int(self.ax.current_state) != ODRIVE_STATE_CLOSED_LOOP
        except Exception:
            return True

    # ---------------------------------------------------------------- print / log
    def status(self, msg, force=False):
        now = time.monotonic()
        if force or now - self._last_print >= PRINT_PERIOD_S:
            self._last_print = now
            print("[%-9s] %s" % (self.state, msg), flush=True)

    def write_log(self, theta, theta_dot, pos, vel):
        if self.log is None:
            return
        self.log.write("%.4f,%s,%.3f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f\n" % (
            time.monotonic() - self._t0, self.state,
            theta if theta is not None else float("nan"), theta_dot, pos, vel,
            self.pos_sp, math.degrees(self.outer_corr), self.vel_cmd))

    # ---------------------------------------------------------------- emergency
    def check_keys_emergency(self):
        if self.keys.quit.is_set():
            raise KeyboardInterrupt
        if self.keys.emergency.is_set():
            self.keys.emergency.clear()
            self.trigger_emergency("SPAZIO premuto")
            return True
        return False

    def trigger_emergency(self, reason):
        """input_vel=0 -> attesa decelerazione -> IDLE."""
        self.state = "EMERGENCY"
        self.emergency_reason = reason
        try:
            self.set_vel(0.0)
            t0 = time.monotonic()
            while time.monotonic() - t0 < EMERGENCY_DECEL_TIMEOUT_S:
                if abs(self.ax.encoder.vel_estimate) < EMERGENCY_STOP_VEL:
                    break
                time.sleep(0.01)
        except Exception:
            pass
        try:
            self.ax.requested_state = ODRIVE_STATE_IDLE
        except Exception:
            pass
        self.inner.reset()
        self.outer.reset()
        self.status("%s — motore IDLE. Premi 'r' per riarmare, 'q' per uscire." % reason,
                    force=True)

    # ================================================================= CALIBRAZIONE
    def startup_check(self):
        """Lettura iniziale encoder ODrive + verifica flusso dati ESP32."""
        self.state = "STARTUP"
        pos0 = self.ax.encoder.pos_estimate
        self.status("Encoder ODrive all'avvio: pos=%+.3f turns" % pos0, force=True)
        t0 = time.monotonic()
        while True:
            theta, theta_dot, age = self.sensor.get()
            if theta is not None and age < SERIAL_STALE_S:
                break
            if time.monotonic() - t0 > 3.0:
                raise RuntimeError("Nessun dato valido dall'ESP32 su %s (righe scartate: %d)"
                                   % (self.a.port, self.sensor.n_bad))
            time.sleep(0.02)
        raw, _, _ = self.sensor.get_raw()
        self.status("ESP32 ok: θ grezzo=%.2f° θ̇=%+.3f rad/s" % (raw, theta_dot), force=True)

        if self.a.no_auto_zero:
            self.sensor.zero_offset = self.a.theta_down_raw
            self.status("Zero fisso: θ grezzo %.2f° = pendolo appeso" % self.a.theta_down_raw,
                        force=True)
        else:
            # Il pendolo DEVE essere appeso e fermo: media su 0.5 s
            samples, max_rate = [], 0.0
            t0 = time.monotonic()
            while time.monotonic() - t0 < 0.5:
                r, rd, _ = self.sensor.get_raw()
                samples.append(r)
                max_rate = max(max_rate, abs(rd))
                time.sleep(0.01)
            # media circolare (robusta al passaggio 359°->0°)
            sx = sum(math.cos(math.radians(x)) for x in samples)
            sy = sum(math.sin(math.radians(x)) for x in samples)
            down = math.degrees(math.atan2(sy, sx)) % 360.0
            if max_rate > 0.3:
                raise RuntimeError("Pendolo non fermo all'avvio (|θ̇| max %.2f rad/s): "
                                   "lascialo APPESO e immobile e riavvia" % max_rate)
            self.sensor.zero_offset = down
            self.status("Zero automatico: pendolo APPESO a θ grezzo=%.2f° -> 0°, "
                        "verticale alta = grezzo %.2f° -> 180°"
                        % (down, (down + 180.0) % 360.0), force=True)

    def _sweep_until_stall(self, direction, label):
        """Muove a homing_vel nella direzione data finché il carrello non si blocca.

        Fase 1 (STACCO): il carrello deve prima percorrere almeno --breakaway-dist turns.
          Partendo appoggiato a un finecorsa accelera lentamente: in questa fase lo
          stallo NON viene valutato. Se non si stacca entro --breakaway-timeout -> errore.
        Fase 2 (CORSA): stallo = spostamento < --stall-dist nell'ultima --stall-window.
        """
        a = self.a
        self.set_vel(direction * a.homing_vel)
        t_start = time.monotonic()
        pos_start = self.ax.encoder.pos_estimate
        moving = False                    # True dopo lo stacco
        t_moving = None
        history = []                      # (t, pos), solo in fase 2
        samples = self._sign_samples.setdefault(label, [])   # (t, dev_giù_deg, θ̇)
        while True:
            if self.check_keys_emergency():
                return None
            if self.odrive_dropped_out():
                self.trigger_emergency("ODrive uscito da CLOSED_LOOP durante homing (%s)"
                                       % odrive_error_summary(self.ax))
                return None
            now = time.monotonic()
            pos = self.ax.encoder.pos_estimate
            elapsed = now - t_start
            if elapsed < SIGN_WINDOW_S and self.sensor is not None:
                th, thd, age = self.sensor.get()
                if th is not None and age < SERIAL_STALE_S:
                    samples.append((now, wrap_deg_180(th), thd))

            if not moving:
                # ---- Fase 1: attesa stacco
                if abs(pos - pos_start) >= a.breakaway_dist:
                    moving = True
                    t_moving = now
                    self.status("Carrello in movimento verso %s (Δ=%+.3f t)"
                                % (label, pos - pos_start), force=True)
                elif elapsed > a.breakaway_timeout:
                    # Diagnostica PRIMA di azzerare input_vel, per vedere il comando attivo.
                    print("[DIAG     ] Il carrello non si è staccato verso %s in %.1f s "
                          "(Δ=%+.4f t). Stato ODrive:\n             %s"
                          % (label, a.breakaway_timeout, pos - pos_start,
                             odrive_diag(self.ax)))
                    self.trigger_emergency("Motore fermo durante homing %s: vedi [DIAG] sopra"
                                           % label)
                    return None
            else:
                # ---- Fase 2: corsa, rilevamento finecorsa
                history.append((now, pos))
                while history and now - history[0][0] > a.stall_window:
                    history.pop(0)
                if (now - t_moving > a.stall_window and
                        now - history[0][0] >= a.stall_window * 0.9 and
                        abs(pos - history[0][1]) < a.stall_dist):
                    self.set_vel(0.0)
                    time.sleep(0.2)
                    pos = self.ax.encoder.pos_estimate
                    self.status("Finecorsa %s trovato: pos=%+.3f t (percorsi %.3f t)"
                                % (label, pos, abs(pos - pos_start)), force=True)
                    return pos

            if elapsed > a.homing_timeout:
                self.set_vel(0.0)
                self.trigger_emergency("Timeout homing %s" % label)
                return None
            self.status("Scansione %s... pos=%+.3f t" % (label, pos))
            time.sleep(0.01)

    def _wait_pendulum_still(self, timeout):
        """Aspetta che il pendolo appeso smetta di oscillare (dopo l'urto al finecorsa)."""
        t0 = time.monotonic()
        quiet_since = None
        while time.monotonic() - t0 < timeout:
            if self.check_keys_emergency():
                return False
            th, thd, age = self.sensor.get()
            now = time.monotonic()
            if th is not None and abs(wrap_deg_180(th)) < 3.0 and abs(thd) < 0.2:
                quiet_since = quiet_since or now
                if now - quiet_since > 0.5:
                    return True
            else:
                quiet_since = None
            self.status("Attendo che il pendolo si fermi... θ=%s"
                        % ("%.1f°" % th if th is not None else "n/d"))
            time.sleep(0.01)
        return False

    def _compute_signs(self):
        """Ricava i segni dall'oscillazione del pendolo appeso all'avvio delle due corse.

        Fisica: quando il carrello accelera verso +turns, il pendolo appeso resta indietro
        (va verso -turns). Ruotando con continuità, il lato "giù a sinistra" porta al lato
        "su a destra": se all'accelerazione verso +turns l'angolo dal basso aumenta,
        allora sopra la verticale un angolo che aumenta = punta verso +turns.
        Quindi angle_sign = segno(picco angolo) * direzione della corsa.
        """
        votes = []
        for label, direction in (("DESTRA", +1), ("SINISTRA", -1)):
            if label == "SINISTRA" and self._sign_samples.get("SINISTRA_SKIP"):
                continue
            smp = self._sign_samples.get(label, [])
            if not smp:
                continue
            peak = max(smp, key=lambda r: abs(r[1]))[1]
            if abs(peak) < SIGN_MIN_DEG:
                self.status("Segno da corsa %s: inconcludente (picco %.1f°)" % (label, peak),
                            force=True)
                continue
            sgn = (1 if peak > 0 else -1) * direction
            votes.append(sgn)
            self.status("Segno da corsa %s: picco angolo %+.1f° -> angle_sign=%+d"
                        % (label, peak, sgn), force=True)

        # θ̇ dell'ESP32 concorda con la derivata numerica di θ?
        corr = 0.0
        for label in ("DESTRA", "SINISTRA"):
            smp = self._sign_samples.get(label, [])
            for (t0, d0, w0), (t1, d1, w1) in zip(smp, smp[1:]):
                if t1 - t0 > 1e-4 and d1 != d0:
                    corr += 0.5 * (w0 + w1) * math.radians(d1 - d0) / (t1 - t0)
        if self.a.thetadot_sign:
            self.thetadot_sign = self.a.thetadot_sign
        elif abs(corr) > 1e-3:
            self.thetadot_sign = 1 if corr > 0 else -1
            self.status("θ̇ dell'ESP32 %s con dθ/dt -> thetadot_sign=%+d"
                        % ("CONCORDE" if corr > 0 else "INVERTITO", self.thetadot_sign),
                        force=True)
        else:
            self.thetadot_sign = 1
            self.status("θ̇: verifica inconcludente, assumo +1", force=True)

        if self.a.angle_sign:
            if votes and any(v != self.a.angle_sign for v in votes):
                self.status("ATTENZIONE: --angle-sign %+d ma l'homing misura %+d"
                            % (self.a.angle_sign, votes[0]), force=True)
            self.angle_sign = self.a.angle_sign
            return True
        if not votes:
            self.trigger_emergency("Impossibile misurare il verso dell'encoder del pendolo: "
                                   "rilancia con --angle-sign 1 o -1")
            return False
        if len(set(votes)) > 1:
            self.trigger_emergency("Corse DESTRA/SINISTRA danno segni opposti: rilancia "
                                   "col pendolo fermo, oppure passa --angle-sign")
            return False
        self.angle_sign = votes[0]
        self.status("angle_sign=%+d (θ che aumenta sopra la verticale = punta verso %s)"
                    % (self.angle_sign, "DESTRA" if self.angle_sign > 0 else "SINISTRA"),
                    force=True)
        return True

    def homing(self):
        a = self.a
        if a.skip_homing:
            self.center = self.ax.encoder.pos_estimate
            if a.rail_meters is not None and a.turns_per_meter is not None:
                rail_t = a.rail_meters * a.turns_per_meter
            else:
                rail_t = a.rail_turns
            self.half_range = rail_t / 2.0
            self.status("Homing saltato: centro = posizione attuale %+.3f t" % self.center,
                        force=True)
        else:
            self.state = "HOMING_R"
            pos_right = self._sweep_until_stall(+1.0, "DESTRA")
            if pos_right is None:
                return False
            if not self._wait_pendulum_still(8.0):
                self.status("Pendolo ancora in oscillazione: la corsa SINISTRA non verrà "
                            "usata per verificare i segni", force=True)
                self._sign_samples["SINISTRA_SKIP"] = True
            self.state = "HOMING_L"
            pos_left = self._sweep_until_stall(-1.0, "SINISTRA")
            if pos_left is None:
                return False
            if not self._compute_signs():
                return False
            if pos_left > pos_right:     # senso encoder invertito rispetto al comando
                pos_left, pos_right = pos_right, pos_left
            measured = pos_right - pos_left
            self.center = (pos_right + pos_left) / 2.0
            if measured < 2 * CENTER_TOLERANCE_TURNS:
                self.trigger_emergency("Corsa misurata troppo corta (%.2f t)" % measured)
                return False
            self.status("Corsa misurata=%.3f t  centro=%+.3f t" % (measured, self.center),
                        force=True)
            if a.rail_meters is not None:
                # ---- GEAR RATIO: turns misurati tra i finecorsa / metri reali del binario
                self.turns_per_meter = measured / a.rail_meters
                mm_per_turn = 1000.0 / self.turns_per_meter
                self.status("GEAR RATIO: %.3f t / %.3f m = %.3f turns/m  (%.2f mm/turn)"
                            % (measured, a.rail_meters, self.turns_per_meter, mm_per_turn),
                            force=True)
                self.status("  -> al prossimo avvio puoi usare: --skip-homing "
                            "--rail-meters %.3f --turns-per-meter %.4f --angle-sign %d "
                            "--thetadot-sign %d"
                            % (a.rail_meters, self.turns_per_meter, self.angle_sign,
                               self.thetadot_sign), force=True)
                if a.turns_per_meter is not None:
                    diff = abs(self.turns_per_meter - a.turns_per_meter) / a.turns_per_meter
                    if diff > 0.05:
                        self.status("ATTENZIONE: rapporto misurato %.4f vs --turns-per-meter "
                                    "%.4f (%.0f%%) -> uso quello misurato"
                                    % (self.turns_per_meter, a.turns_per_meter, diff * 100),
                                    force=True)
                self.half_range = measured / 2.0
            elif a.rail_turns is not None:
                diff = abs(measured - a.rail_turns) / a.rail_turns
                if diff > 0.20:
                    self.status("ATTENZIONE: corsa misurata %.2f t vs --rail-turns %.2f t "
                                "(%.0f%% di differenza)" % (measured, a.rail_turns, diff * 100),
                                force=True)
                self.half_range = a.rail_turns / 2.0
            else:
                self.half_range = measured / 2.0

        self.soft_limit = self.half_range * a.soft_fraction
        # Barriera dura in MANUAL/READY: a metà tra soft limit e finecorsa
        self.hard_limit = (self.soft_limit + self.half_range) / 2.0
        self.status("Semi-corsa=%.2f t  soft limit=±%.2f t  barriera=±%.2f t"
                    % (self.half_range, self.soft_limit, self.hard_limit), force=True)
        if self.turns_per_meter:
            self.status("In metri: semi-corsa=%s  soft limit=±%s  barriera=±%s"
                        % (self.fmt_pos(self.half_range, False),
                           self.fmt_pos(self.soft_limit, False),
                           self.fmt_pos(self.hard_limit, False)), force=True)

        # ---- CENTERING
        self.state = "CENTERING"
        t_start = time.monotonic()
        while True:
            if self.check_keys_emergency():
                return False
            if self.odrive_dropped_out():
                self.trigger_emergency("ODrive uscito da CLOSED_LOOP durante centering")
                return False
            err = self.ax.encoder.pos_estimate - self.center
            if abs(err) < CENTER_TOLERANCE_TURNS:
                self.set_vel(0.0)
                break
            # rallenta vicino al centro (min 30% della velocità di homing)
            v = a.homing_vel * clamp(abs(err) / 2.0, 0.3, 1.0)
            self.set_vel(-math.copysign(v, err))
            if time.monotonic() - t_start > a.homing_timeout:
                self.trigger_emergency("Timeout centering")
                return False
            self.status("Vado al centro... errore=%s" % self.fmt_pos(err))
            time.sleep(0.01)
        self.status("Carrello al centro. Pronto.", force=True)
        self.state = "READY"
        return True

    # ================================================================= UNITÀ
    def fmt_pos(self, turns, signed=True):
        """Formatta una posizione/distanza: in mm se il gear
        ratio è noto, altrimenti in turns. Il controllo interno resta in turns."""
        if self.turns_per_meter:
            mm = turns / self.turns_per_meter * 1000.0
            return ("%+.0fmm" if signed else "%.0fmm") % mm
        return ("%+.2ft" if signed else "%.2ft") % turns

    # ================================================================= STATI
    def enter_balancing(self, pos):
        self.inner.reset()
        self.outer.reset()
        self.outer_corr = 0.0
        self.pos_sp = clamp(pos, -self.soft_limit * 0.8, self.soft_limit * 0.8)
        self._loop_count = 0
        self.state = "BALANCING"
        self.status("CATTURA! Balance attivo (setpoint=%s)" % self.fmt_pos(self.pos_sp), force=True)

    def handle_requests(self, theta_dev, pos, buttons):
        """Gestisce richieste da tastiera/joystick (reset, arm, manual, toggle)."""
        k = self.keys
        if k.reset_req.is_set():
            k.reset_req.clear()
            if self.state == "EMERGENCY":
                self.status("Riarmo in corso...", force=True)
                if self.odrive_enable():
                    self.state = "READY" if self.half_range > 0 else "EMERGENCY"
                    self.status("Riarmato. In READY (balance automatico alla cattura).",
                                force=True)
        if self.state == "EMERGENCY":
            return
        in_catch = theta_dev is not None and abs(theta_dev) < self.a.catch_angle
        if k.manual_req.is_set():
            k.manual_req.clear()
            self.state = "MANUAL"
            self.set_vel(0.0)
        if k.arm_req.is_set():
            k.arm_req.clear()
            if self.state == "MANUAL":
                self.state = "READY"
        if JOY_BTN_TOGGLE in buttons:
            if self.state == "BALANCING":
                self.state = "MANUAL"
                self.set_vel(0.0)
                self.status("Toggle -> MANUAL", force=True)
            elif in_catch:
                self.enter_balancing(pos)
            else:
                self.state = "READY"
                self.status("Toggle -> READY (asta fuori dal cono di cattura)", force=True)

    def manual_velocity(self, axis_val, pos, buttons):
        """READY/MANUAL: joystick -> velocità diretta, con barriere virtuali."""
        v = axis_val * self.a.joy_max_vel
        if JOY_BTN_STOP in buttons:
            v = 0.0
        # Non permettere di andare oltre il soft limit verso l'esterno
        if pos >= self.soft_limit and v > 0:
            v = 0.0
        if pos <= -self.soft_limit and v < 0:
            v = 0.0
        self.set_vel(v)

    def balance_step(self, theta_deg, theta_dot, pos, vel, axis_val, buttons, dt):
        a = self.a
        # ---- setpoint posizione da joystick (con rientro lento verso il centro)
        if JOY_BTN_STOP in buttons:
            self.pos_sp = pos
        if axis_val != 0.0:
            self.pos_sp += axis_val * a.joy_sp_rate * dt
        elif a.recenter_rate > 0.0:
            step = a.recenter_rate * dt
            self.pos_sp -= clamp(self.pos_sp, -step, step)
        sp_lim = self.soft_limit * 0.8
        self.pos_sp = clamp(self.pos_sp, -sp_lim, sp_lim)

        # Riferimento unico: x verso +turns, φ = inclinazione asta dalla verticale alta,
        # φ>0 = punta verso +turns. Tutto in metri/radianti, convertito alla fine.
        tpm = self.turns_per_meter
        phi = self.angle_sign * math.radians(dev_from_top_deg(theta_deg))
        phi_dot = self.angle_sign * self.thetadot_sign * theta_dot

        # ---- OUTER: carrello lontano dal setpoint -> l'asta deve pendere VERSO il setpoint
        if self._loop_count % a.outer_div == 0:
            e_pos = (pos - self.pos_sp) / tpm          # m
            self.outer_corr = -self.outer.update(e_pos, vel / tpm, dt * a.outer_div)
        self._loop_count += 1

        # ---- INNER: il carrello va sotto la punta. v = Kp·e + Ki·∫e + Kd·φ̇  [m/s]
        # (Ki·∫e equivale a un'accelerazione proporzionale all'inclinazione: regge la gravità)
        e_angle = phi - self.outer_corr
        u = self.inner.update(e_angle, phi_dot, dt)
        # anti-windup: se il comando satura, non far crescere ulteriormente l'integrale
        v_lim = a.max_vel / tpm
        if abs(u) > v_lim:
            self.inner.integral -= e_angle * dt
            u = math.copysign(v_lim, u)
        self.set_vel(u * tpm)
        return e_angle, u

    # ================================================================= MAIN LOOP
    def run(self):
        a = self.a
        next_t = time.monotonic()
        while True:
            now = time.monotonic()
            # Statistica frequenza loop
            self._rate_n += 1
            if now - self._rate_t0 >= 1.0:
                self.loop_hz = self._rate_n / (now - self._rate_t0)
                self._rate_t0, self._rate_n = now, 0

            self.check_keys_emergency()
            axis_val, buttons = self.joy.poll()
            theta, theta_dot, age = self.sensor.get()
            theta_dev = dev_from_top_deg(theta) if theta is not None else None

            if self.state == "EMERGENCY":
                self.handle_requests(theta_dev, 0.0, buttons)
                self.status("%s — IDLE. 'r'=riarma, 'q'=esci" % self.emergency_reason)
                time.sleep(0.02)
                next_t = time.monotonic()
                continue

            # ---- letture ODrive + controlli di sicurezza comuni
            if self.odrive_dropped_out():
                self.trigger_emergency("ODrive non più in CLOSED_LOOP (errori: %s)"
                                       % odrive_error_summary(self.ax))
                continue
            pos_abs, vel = self.read_pos_vel()
            pos = pos_abs - self.center

            self.handle_requests(theta_dev, pos, buttons)

            if abs(pos) > self.hard_limit:
                self.trigger_emergency("Barriera virtuale superata: pos=%s (lim ±%s)"
                                       % (self.fmt_pos(pos), self.fmt_pos(self.hard_limit, False)))
                continue

            # ---- macchina a stati
            if self.state == "BALANCING":
                if abs(pos) > self.soft_limit:
                    self.trigger_emergency("Soft limit superato in balance: pos=%s "
                                           "(lim ±%s)" % (self.fmt_pos(pos),
                                                          self.fmt_pos(self.soft_limit, False)))
                    continue
                if theta is None or age > SERIAL_STALE_S:
                    self.trigger_emergency("Dato ESP32 perso (età %.0f ms)" % (age * 1000))
                    continue
                if abs(theta_dev) > a.fall_angle:
                    self.set_vel(0.0)
                    self.state = "MANUAL"
                    self.status("Asta caduta (|θ-180|=%.1f° > %.1f°) -> MANUAL. "
                                "'b' o BTN1 per riarmare." % (abs(theta_dev), a.fall_angle),
                                force=True)
                else:
                    _, u = self.balance_step(theta, theta_dot, pos, vel, axis_val, buttons,
                                             a.dt)
                    self.status("θ=%6.1f° ω=%+6.2f pos=%7s sp=%7s | v=%+5.2fm/s "
                                "tilt_rich=%+5.2f° | soft_lim=±%s | %3.0f Hz"
                                % (theta, theta_dot, self.fmt_pos(pos),
                                   self.fmt_pos(self.pos_sp), u,
                                   math.degrees(self.outer_corr),
                                   self.fmt_pos(self.soft_limit, False), self.loop_hz))

            elif self.state == "READY":
                if theta is not None and age < SERIAL_STALE_S and abs(theta_dev) < a.catch_angle:
                    self.enter_balancing(pos)
                else:
                    self.manual_velocity(axis_val, pos, buttons)
                    dev_txt = "%.1f°" % abs(theta_dev) if theta_dev is not None else "n/d"
                    self.status("In attesa di cattura (|θ-180|=%s > %.1f°) pos=%s "
                                "vel_cmd=%+.1f" % (dev_txt, a.catch_angle, self.fmt_pos(pos),
                                                   self.vel_cmd))

            elif self.state == "MANUAL":
                self.manual_velocity(axis_val, pos, buttons)
                th_txt = "%.1f°" % theta if theta is not None else "n/d"
                self.status("Joystick: vel_cmd=%+.1f t/s pos=%s θ=%s | 'b'/BTN1=arma"
                            % (self.vel_cmd, self.fmt_pos(pos), th_txt))

            self.write_log(theta, theta_dot, pos, vel)

            # ---- temporizzazione a periodo fisso
            next_t += a.dt
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            elif sleep < -0.05:      # siamo molto in ritardo: riallinea
                next_t = time.monotonic()

    def close(self):
        if self.log is not None:
            self.log.close()


# ======================================================================================
# main
# ======================================================================================
def main():
    args = parse_args()
    print("=" * 78)
    print(" SINGLE BALANCER — SPAZIO = EMERGENZA | r = riarma | b = arma | m = manual | q = esci")
    print(" Inner [m/s/rad]: Kp=%.2f Ki=%.2f Kd=%.2f | Outer [rad/m]: Kp=%.3f Ki=%.3f "
          "Kd=%.3f | tilt ±%.1f°"
          % (args.kp_in, args.ki_in, args.kd_in, args.kp_out, args.ki_out, args.kd_out,
             args.max_tilt))
    print("=" * 78)

    odrv = ax = sensor = keys = joy = bal = None
    try:
        keys = KeyboardWatcher()
        keys.start()

        print("[STARTUP  ] Apro ESP32 su %s @ %d..." % (args.port, args.baud))
        try:
            sensor = PendulumSensor(args.port, args.baud)
        except Exception as e:
            if "busy" in str(e).lower() or "Errno 16" in str(e):
                raise RuntimeError(
                    "%s occupata da un altro processo (istanza precedente dello script, "
                    "monitor seriale Arduino/screen/minicom?). Trovalo con: "
                    "fuser -v %s" % (args.port, args.port))
            raise
        sensor.start()

        print("[STARTUP  ] Cerco ODrive...")
        import odrive
        odrv = odrive.find_any(timeout=10)
        ax = getattr(odrv, "axis%d" % args.axis)
        print("[STARTUP  ] ODrive trovato (serial %s)" % getattr(odrv, "serial_number", "?"))

        joy = Joystick()
        bal = Balancer(args, odrv, sensor, keys, joy)
        bal.startup_check()
        if not bal.odrive_enable():
            raise RuntimeError("ODrive non abilitato")
        if not bal.homing():
            print("[STARTUP  ] Homing non completato. 'r' NON riprende l'homing: riavvia lo script.")
            # Rimani in EMERGENCY finché l'utente esce
            while not keys.quit.is_set():
                time.sleep(0.05)
            return
        bal.run()

    except KeyboardInterrupt:
        print("\r\n[EXIT     ] Uscita richiesta dall'utente")
    except Exception as e:
        print("\r\n[ERROR    ] %s: %s" % (type(e).__name__, e))
    finally:
        # Il motore va SEMPRE in IDLE, qualunque cosa succeda
        if ax is not None:
            try:
                ax.controller.input_vel = 0.0
                time.sleep(0.1)
            except Exception:
                pass
            try:
                ax.requested_state = ODRIVE_STATE_IDLE
                print("[EXIT     ] ODrive in IDLE")
            except Exception:
                print("[EXIT     ] ATTENZIONE: impossibile mettere ODrive in IDLE!")
        if bal is not None:
            bal.close()
        if sensor is not None:
            sensor.stop()
        if joy is not None:
            joy.close()
        if keys is not None:
            keys.stop()
            keys.restore()


if __name__ == "__main__":
    main()

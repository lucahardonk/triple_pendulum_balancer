// overrun_node: calibra la corsa del carrello, lo porta al centro, ripubblica la
// telemetria in unita' SI su /pendulum_status e sorveglia due barriere virtuali.
//
// START_UP  -> attende la telemetria CAN
// ACTIVE    -> coppia 0 e richiesta CLOSED_LOOP
// PUSH_POS  -> coppia +push_torque fino a carrello fermo           => min_position
// PAUSE_1   -> coppia 0 per 1 s
// PUSH_NEG  -> coppia -push_torque fino a carrello fermo           => max_position
// PAUSE_2   -> coppia 0 per 1 s
// GO_CENTER -> PID in coppia verso il centro; nel range kCenterTol e' arrivato
// READY     -> coppia 0, calibrazione valida: parte /pendulum_status
// VIGILANT  -> sorveglia (a 200 Hz) due barriere a kBarrierMargin dai fine corsa
// OVERRUN   -> barriera superata: motore subito in IDLE, l'operatore riporta il carrello
//              al centro a mano (contatore a terminale)
// REARM     -> carrello al centro da kRecoverHold s: richiede CLOSED_LOOP -> READY -> VIGILANT
// ERROR     -> timeout, telemetria persa, errore ODrive: coppia 0 + IDLE, fine
//
// /pendulum_status (da READY in poi): posizione [m] dal centro, velocita' [m/s], corsa [m]
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <odrive_can/msg/controller_status.hpp>
#include <odrive_can/msg/control_message.hpp>
#include <odrive_can/srv/axis_state.hpp>
#include <pendulum_safety_package/msg/pendulum_status.hpp>

using namespace std::chrono_literals;
using odrive_can::msg::ControllerStatus;
using odrive_can::msg::ControlMessage;
using odrive_can::srv::AxisState;
using pendulum_safety_package::msg::PendulumStatus;
using Clock = std::chrono::steady_clock;

class OverrunNode : public rclcpp::Node
{
public:
  OverrunNode() : Node("overrun_node")
  {
    push_torque_ = declare_parameter("push_torque", 0.2);

    // Telemetria a 200 Hz: ripubblicazione SI + controllo barriere nello stesso callback.
    sub_ = create_subscription<ControllerStatus>("/odrive_axis0/controller_status",
      rclcpp::QoS(10).best_effort(), [this](ControllerStatus::SharedPtr m) { on_status(*m); });
    pub_ = create_publisher<ControlMessage>("/odrive_axis0/control_message", 10);
    status_pub_ = create_publisher<PendulumStatus>("/pendulum_status", rclcpp::QoS(10).best_effort());
    axis_ = create_client<AxisState>("/odrive_axis0/request_axis_state");
    timer_ = create_wall_timer(100ms, [this]() { step(); });

    go(State::START_UP, "avvio nodo");
  }

private:
  enum class State { START_UP, ACTIVE, PUSH_POS, PAUSE_1, PUSH_NEG, PAUSE_2, GO_CENTER,
                     READY, VIGILANT, OVERRUN, REARM, ERROR };
  static constexpr const char * kNames[] = {"START_UP", "ACTIVE", "PUSH_POS", "PAUSE_1", "PUSH_NEG",
                                            "PAUSE_2", "GO_CENTER", "READY", "VIGILANT", "OVERRUN",
                                            "REARM", "ERROR"};

  // ------------------------------------------------------------ telemetria (200 Hz)
  void on_status(const ControllerStatus & m)
  {
    msg_ = m;
    msg_time_ = Clock::now();
    have_msg_ = true;
    if (!calibrated_) return;                      // niente /pendulum_status prima di READY

    const double p = pos_m();
    PendulumStatus s;
    s.header.stamp = now();
    s.position_m = p;
    s.velocity_m_s = msg_.vel_estimate * kTau * kMPerRad;
    s.stroke_m = stroke_m();
    status_pub_->publish(s);

    // Barriere virtuali: controllate a ogni messaggio, non a 10 Hz, per reagire subito.
    if (state_ == State::VIGILANT && std::abs(p) > barrier_) trip(p);
  }

  // ------------------------------------------------------------ macchina a stati (10 Hz)
  void step()
  {
    if (state_ == State::ERROR) return;

    const double pos = msg_.pos_estimate * kTau;   // [rad]
    const double vel = msg_.vel_estimate * kTau;   // [rad/s]

    // Controlli di sicurezza. READY/VIGILANT/OVERRUN possono durare quanto vogliono.
    const bool timed = state_ != State::READY && state_ != State::VIGILANT && state_ != State::OVERRUN;
    const double limit = (state_ == State::GO_CENTER) ? kTimeoutCenter : kTimeout;
    if (timed && since(state_time_) > limit) return fail("timeout stato");
    if (state_ != State::START_UP) {
      if (since(msg_time_) > kTelemTimeout) return fail("telemetria CAN persa");
      if (msg_.active_errors) return fail("errore ODrive");
    }

    tq_ = 0.0;
    switch (state_) {
      // ---------------- calibrazione
      case State::START_UP:
        if (have_msg_) go(State::ACTIVE, "telemetria ok, pos=" + fmt(pos) + " rad");
        break;

      case State::ACTIVE:
        if (closed_loop_ok_) go(State::PUSH_POS, "CLOSED_LOOP attivo, parte spinta +" + fmt(push_torque_) + " N*m");
        break;

      case State::PUSH_POS:
        tq_ = +push_torque_;
        if (stopped(pos)) {
          min_position_ = pos;
          go(State::PAUSE_1, "min_position=" + fmt(min_position_) + " rad");
        }
        break;

      case State::PAUSE_1:
        if (since(state_time_) > 1.0) go(State::PUSH_NEG, "parte spinta -" + fmt(push_torque_) + " N*m");
        break;

      case State::PUSH_NEG:
        tq_ = -push_torque_;
        if (stopped(pos)) {
          max_position_ = pos;
          center_ = (min_position_ + max_position_) / 2.0;
          barrier_ = std::max(0.0, stroke_m() / 2.0 - kBarrierMargin);
          go(State::PAUSE_2, "max_position=" + fmt(max_position_) + " rad, corsa=" + fmt(stroke_m()) +
             " m, centro=" + fmt(center_) + " rad");
        }
        break;

      case State::PAUSE_2:
        if (since(state_time_) > 1.0) go(State::GO_CENTER, "vado verso il centro");
        break;

      case State::GO_CENTER: {  // PID in coppia: P sull'errore, I contro l'attrito, D sulla velocita'
        const double err = center_ - pos;
        if (std::abs(err) < kIntRange)
          integ_ = std::clamp(integ_ + err * kDt, -kMaxI / kI, kMaxI / kI);
        tq_ = std::clamp(kP * err + kI * integ_ - kD * vel, -kMaxTorque, kMaxTorque);
        if (std::abs(err) < kCenterTol)
          go(State::READY, "al centro, pos=" + fmt(pos_m()) + " m, err=" + fmt(err) + " rad");
        break;
      }

      // ---------------- sorveglianza
      case State::READY:        // coppia gia' a 0: si passa subito alla sorveglianza
        go(State::VIGILANT, "barriere a +/-" + fmt(barrier_) + " m dal centro (" +
           fmt(kBarrierMargin * 100.0) + " cm dai fine corsa)");
        return;

      case State::VIGILANT:     // il controllo e' in on_status(); qui non si comanda il motore
        return;

      case State::OVERRUN:      // motore in IDLE: guida l'operatore
        guide_operator();
        return;

      case State::REARM:        // closed_loop_ok_ arriva dalla risposta del servizio
        if (closed_loop_ok_) go(State::READY, "CLOSED_LOOP di nuovo attivo");
        return;

      default: return;
    }
    send_torque(tq_);           // solo gli stati di calibrazione arrivano qui
  }

  // ------------------------------------------------------------ barriera superata
  void trip(double p)
  {
    send_torque(0.0);
    request_state(kIdle);
    RCLCPP_ERROR(get_logger(), "!!! BARRIERA %s SUPERATA: pos=%+.3f m, limite +/-%.3f m -> motore in IDLE",
      p > 0 ? "POSITIVA" : "NEGATIVA", p, barrier_);
    go(State::OVERRUN, "riporta il carrello A MANO entro +/-" + fmt(kRecoverTol * 100.0) + " cm dal centro");
  }

  // Contatore a terminale (ogni 0.5 s): quanto manca e in che direzione spingere.
  void guide_operator()
  {
    const double p = pos_m();
    const bool inside = std::abs(p) < kRecoverTol;
    if (!inside) in_range_since_ = Clock::now();

    const double held = since(in_range_since_);
    if (held > kRecoverHold) {
      go(State::REARM, "OK, carrello al centro (pos=" + fmt(p) + " m), richiedo CLOSED_LOOP");
      return;
    }

    if (since(last_hint_) < 0.5) return;
    last_hint_ = Clock::now();
    if (inside) {
      RCLCPP_INFO(get_logger(), "[OVERRUN] OK al centro (pos=%+.3f m): tieni fermo ancora %.1f s",
        p, kRecoverHold - held);
    } else {
      RCLCPP_WARN(get_logger(), "[OVERRUN] mancano %5.1f cm  -> spingi verso il lato %s   (pos=%+.3f m)",
        (std::abs(p) - kRecoverTol) * 100.0, p > 0 ? "NEGATIVO (-)" : "POSITIVO (+)", p);
    }
  }

  // ------------------------------------------------------------ transizioni
  void go(State next, const std::string & info)
  {
    RCLCPP_INFO(get_logger(), "%s -> %s | %s", kNames[static_cast<int>(state_)],
      kNames[static_cast<int>(next)], info.c_str());
    state_ = next;
    state_time_ = Clock::now();
    tq_ = 0.0;
    last_pos_ = 1e9;                       // reset rilevamento fermo
    start_pos_ = msg_.pos_estimate * kTau;
    moved_ = false;
    integ_ = 0.0;
    in_range_since_ = Clock::now();
    last_hint_ = Clock::time_point{};      // primo suggerimento OVERRUN subito

    switch (next) {                        // azioni d'ingresso
      case State::ACTIVE:
      case State::REARM:                   // coppia 0 + CLOSED_LOOP
        closed_loop_ok_ = false;
        send_torque(0.0);
        request_state(kClosedLoop);
        break;
      case State::READY:                   // calibrazione valida: parte /pendulum_status
        send_torque(0.0);
        calibrated_ = true;
        break;
      default: break;
    }
  }

  void fail(const std::string & why)
  {
    RCLCPP_ERROR(get_logger(), "ERRORE in %s: %s (err=0x%X)", kNames[static_cast<int>(state_)],
      why.c_str(), msg_.active_errors);
    send_torque(0.0);
    request_state(kIdle);
    state_ = State::ERROR;
  }

  // ------------------------------------------------------------ comandi ODrive
  void send_torque(double torque)
  {
    ControlMessage cmd;
    cmd.control_mode = kTorqueControl;
    cmd.input_mode = kPassthrough;
    cmd.input_torque = static_cast<float>(torque);
    pub_->publish(cmd);
  }

  void request_state(uint32_t requested)
  {
    auto req = std::make_shared<AxisState::Request>();
    req->axis_requested_state = requested;
    axis_->async_send_request(req, [this, requested](rclcpp::Client<AxisState>::SharedFuture f) {
      auto r = f.get();
      if (requested != kClosedLoop) return;
      if (r->axis_state == kClosedLoop && r->active_errors == 0) closed_loop_ok_ = true;
      else fail("CLOSED_LOOP rifiutato");
    });
  }

  // ------------------------------------------------------------ utilita'
  static double since(Clock::time_point t) { return std::chrono::duration<double>(Clock::now() - t).count(); }
  static std::string fmt(double v) { char b[32]; std::snprintf(b, sizeof b, "%.3f", v); return b; }
  double pos_m() const { return (msg_.pos_estimate * kTau - center_) * kMPerRad; }   // [m] dal centro
  double stroke_m() const { return std::abs(max_position_ - min_position_) * kMPerRad; }

  // Fermo = posizione (2 decimali) invariata da 1 s.
  bool still(double pos)
  {
    const double r = std::round(pos * 100.0) / 100.0;
    if (r != last_pos_) { last_pos_ = r; still_since_ = Clock::now(); }
    return since(still_since_) > 1.0;
  }

  // Si e' mosso di almeno 0.05 rad dalla partenza e ora e' fermo (evita il falso "fermo" iniziale).
  bool stopped(double pos)
  {
    if (std::abs(pos - start_pos_) > 0.05) moved_ = true;
    return still(pos) && moved_;
  }

  // ------------------------------------------------------------ dati
  static constexpr double kTau         = 2.0 * M_PI;
  static constexpr double kPulleyPitch = 0.005;          // [m] HTD 5M
  static constexpr double kPulleyTeeth = 12.0;
  static constexpr double kMPerRad     = kPulleyPitch * kPulleyTeeth / kTau;  // ~0.00955 m/rad

  // calibrazione
  static constexpr double kDt            = 0.1;    // [s] periodo del timer
  static constexpr double kTimeout       = 15.0;   // [s] max per stato
  static constexpr double kTimeoutCenter = 5.0;    // [s] max per GO_CENTER
  static constexpr double kTelemTimeout  = 2.0;    // [s] telemetria ferma
  static constexpr double kCenterTol     = 0.3;    // [rad]
  static constexpr double kP             = 0.1;    // [N*m/rad]
  static constexpr double kI             = 0.05;   // [N*m/(rad*s)]
  static constexpr double kD             = 0.01;   // [N*m/(rad/s)]
  static constexpr double kMaxI          = 0.08;   // [N*m]
  static constexpr double kIntRange      = 5.0;    // [rad]
  static constexpr double kMaxTorque     = 0.2;    // [N*m]

  // sorveglianza
  static constexpr double kBarrierMargin = 0.10;   // [m] barriera a 10 cm da ogni fine corsa
  static constexpr double kRecoverTol    = 0.05;   // [m] "al centro" per il riarmo
  static constexpr double kRecoverHold   = 1.0;    // [s] tempo da tenere fermo al centro

  static constexpr uint32_t kIdle = 1, kClosedLoop = 8, kTorqueControl = 1, kPassthrough = 1;

  double push_torque_{0.2}, tq_{0.0}, integ_{0.0};
  double min_position_{0.0}, max_position_{0.0}, center_{0.0}, barrier_{0.0};
  double last_pos_{1e9}, start_pos_{0.0};
  bool have_msg_{false}, closed_loop_ok_{false}, moved_{false}, calibrated_{false};
  State state_{State::START_UP};
  Clock::time_point state_time_, msg_time_, still_since_, in_range_since_, last_hint_;
  ControllerStatus msg_;

  rclcpp::Subscription<ControllerStatus>::SharedPtr sub_;
  rclcpp::Publisher<ControlMessage>::SharedPtr pub_;
  rclcpp::Publisher<PendulumStatus>::SharedPtr status_pub_;
  rclcpp::Client<AxisState>::SharedPtr axis_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OverrunNode>());
  rclcpp::shutdown();
  return 0;
}

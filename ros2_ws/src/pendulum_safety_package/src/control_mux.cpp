// control_mux: multiplexer a priorita' per /odrive_axis0/control_message.
// Stessa logica di twist_mux, ma per odrive_can/ControlMessage (twist_mux accetta solo Twist).
// E' l'UNICO nodo che pubblica sul topic di comando dell'ODrive.
//
// - Ogni sorgente pubblica sul proprio topic (default /control/<nome>).
// - Priorita' = ordine nella lista "inputs": la prima e' la piu' alta.
// - Una sorgente e' attiva se ha pubblicato negli ultimi <nome>.timeout secondi.
// - All'ODrive passa solo la sorgente attiva con priorita' piu' alta.
// - Se nessuna e' attiva manda UNA volta coppia 0 (l'ODrive altrimenti tiene l'ultimo setpoint).
// - La sorgente vincente e' pubblicata su ~/active (std_msgs/String, latched).
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <odrive_can/msg/control_message.hpp>

using namespace std::chrono_literals;
using odrive_can::msg::ControlMessage;
using Clock = std::chrono::steady_clock;

class ControlMux : public rclcpp::Node
{
public:
  ControlMux() : Node("control_mux")
  {
    const auto names = declare_parameter<std::vector<std::string>>(
      "inputs", std::vector<std::string>{"safety", "joystick", "controller"});
    const auto out = declare_parameter<std::string>("output_topic", "/odrive_axis0/control_message");

    pub_ = create_publisher<ControlMessage>(out, 10);   // reliable, come si aspetta odrive_can
    active_pub_ = create_publisher<std_msgs::msg::String>("~/active", rclcpp::QoS(1).transient_local());

    // Ingressi best effort: accettano sia publisher reliable sia best effort.
    const auto in_qos = rclcpp::QoS(10).best_effort();
    for (size_t i = 0; i < names.size(); ++i) {
      auto in = std::make_unique<Input>();
      in->name = names[i];
      in->topic = declare_parameter<std::string>(names[i] + ".topic", "/control/" + names[i]);
      in->timeout = declare_parameter<double>(names[i] + ".timeout", 0.5);
      Input * p = in.get();
      in->sub = create_subscription<ControlMessage>(in->topic, in_qos,
        [this, p](ControlMessage::SharedPtr m) {
          p->last = *m;
          p->stamp = Clock::now();
          p->seen = true;
          on_input(p);
        });
      RCLCPP_INFO(get_logger(), "priorita' %zu: %-10s <- %-22s (timeout %.2f s)",
        i + 1, in->name.c_str(), in->topic.c_str(), in->timeout);
      inputs_.push_back(std::move(in));
    }
    RCLCPP_INFO(get_logger(), "uscita -> %s", out.c_str());

    publish_active();
    timer_ = create_wall_timer(20ms, [this]() { switch_to(winner()); });   // timeout a 50 Hz
  }

private:
  struct Input
  {
    std::string name, topic;
    double timeout{0.5};
    ControlMessage last;
    Clock::time_point stamp;
    bool seen{false};
    rclcpp::Subscription<ControlMessage>::SharedPtr sub;
  };

  // Sorgente attiva con priorita' piu' alta (nullptr se nessuna).
  Input * winner() const
  {
    const auto t = Clock::now();
    for (const auto & in : inputs_)
      if (in->seen && std::chrono::duration<double>(t - in->stamp).count() < in->timeout)
        return in.get();
    return nullptr;
  }

  // Messaggio in ingresso: inoltrato subito se viene dalla sorgente vincente.
  void on_input(Input * src)
  {
    switch_to(winner());
    if (current_ == src) pub_->publish(src->last);
  }

  void switch_to(Input * next)
  {
    if (next == current_) return;
    RCLCPP_WARN(get_logger(), "sorgente: %s -> %s", label(current_), next ? next->name.c_str() : "nessuna (coppia 0)");
    current_ = next;
    publish_active();
    if (next) {
      pub_->publish(next->last);
    } else {
      ControlMessage zero;
      zero.control_mode = kTorqueControl;
      zero.input_mode = kPassthrough;
      zero.input_torque = 0.0f;
      pub_->publish(zero);
    }
  }

  void publish_active()
  {
    std_msgs::msg::String s;
    s.data = current_ ? current_->name : "none";
    active_pub_->publish(s);
  }

  static const char * label(const Input * in) { return in ? in->name.c_str() : "nessuna"; }

  static constexpr uint32_t kTorqueControl = 1, kPassthrough = 1;

  std::vector<std::unique_ptr<Input>> inputs_;
  Input * current_{nullptr};
  rclcpp::Publisher<ControlMessage>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlMux>());
  rclcpp::shutdown();
  return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

using namespace std::chrono_literals;

class OverrunNode : public rclcpp::Node
{
public:
  using Trigger = std_srvs::srv::Trigger;
  using Client = rclcpp::Client<Trigger>;
  using Clock = std::chrono::steady_clock;

  OverrunNode() : Node("overrun_node")
  {
    auto qos = rclcpp::QoS(1).best_effort();

    command_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/odrive_driver/command", qos);

    position_pub_ = create_publisher<std_msgs::msg::Float64>(
      "~/position_rad", 10);

    idle_ = create_client<Trigger>("/odrive_driver/set_idle");
    velocity_mode_ = create_client<Trigger>(
      "/odrive_driver/set_vel_mode");
    enable_ = create_client<Trigger>(
      "/odrive_driver/set_closed_loop");

    feedback_sub_ = create_subscription<std_msgs::msg::String>(
      "/odrive_driver/feedback", qos,
      [this](std_msgs::msg::String::SharedPtr msg) {
        feedback(msg->data);
      });

    stop_service_ = create_service<Trigger>(
      "~/stop",
      [this](
        const std::shared_ptr<Trigger::Request>,
        std::shared_ptr<Trigger::Response> response)
      {
        stop("Manual stop");
        response->success = true;
        response->message = "IDLE requested";
      });

    timer_ = create_wall_timer(10ms, [this]() { tick(); });
  }

private:
  enum class State {
    WAITING,
    SERVICE,
    IDLE_SETTLE,
    ENABLE,
    ENABLE_SETTLE,
    FIRST_SWEEP,
    SECOND_SWEEP,
    CENTERING,
    READY,
    STOPPED
  };

  static double steady()
  {
    return std::chrono::duration<double>(
      Clock::now().time_since_epoch()).count();
  }

  void command(double velocity)
  {
    std_msgs::msg::Float64 msg;
    msg.data = velocity;
    command_pub_->publish(msg);
  }

  void call(const Client::SharedPtr & client, State next)
  {
    state_ = State::SERVICE;

    client->async_send_request(
      std::make_shared<Trigger::Request>(),
      [this, next](Client::SharedFuture future) {
        if (state_ == State::STOPPED) {
          return;
        }

        try {
          const auto response = future.get();
          if (!response->success) {
            stop("Driver service failed: " + response->message);
            return;
          }

          state_ = next;
          state_since_ = steady();

          if (next == State::READY) {
            RCLCPP_INFO(
              get_logger(), "READY: IDLE; barriers active");
          }
        } catch (const std::exception & e) {
          stop(e.what());
        }
      });
  }

  void stop(const std::string & reason)
  {
    if (state_ == State::STOPPED) {
      return;
    }

    state_ = State::STOPPED;
    RCLCPP_WARN(get_logger(), "%s; requesting IDLE", reason.c_str());

    // No braking command: request IDLE directly.
    idle_->async_send_request(std::make_shared<Trigger::Request>());
  }

  void start_sweep(State state, double direction)
  {
    state_ = state;
    direction_ = direction;
    history_.clear();

    // Allow startup/reversal before observing a full stationary window.
    observe_after_ = steady() + 0.5;
    command(direction_ * speed_);

    RCLCPP_INFO(
      get_logger(), "Sweep: %.3f rad/s", direction_ * speed_);
  }

  void feedback(const std::string & text)
  {
    try {
      const auto json = nlohmann::json::parse(text);
      const auto & p = json.at("position_rad");

      if (p.at("unit").get<std::string>() != "rad") {
        throw std::runtime_error("Position must be in rad");
      }

      position_ = p.at("value").get<double>();
      if (!std::isfinite(position_)) {
        throw std::runtime_error("Invalid position");
      }

      have_position_ = true;
      const double now = steady();

      if (calibrated_) {
        std_msgs::msg::Float64 msg;
        msg.data = position_ - center_;
        position_pub_->publish(msg);
      }

      if (barriers_active_ &&
          std::abs(position_ - center_) >= barrier_)
      {
        stop("Virtual barrier crossed");
        return;
      }

      if (state_ == State::CENTERING) {
        const double error = center_ - position_;

        if (std::abs(error) <= center_tolerance_) {
          command(0.0);
          barriers_active_ = true;
          call(idle_, State::READY);
        } else {
          command(std::clamp(error, -speed_, speed_));
        }
        return;
      }

      if (state_ != State::FIRST_SWEEP &&
          state_ != State::SECOND_SWEEP)
      {
        return;
      }

      if (now < observe_after_) {
        return;
      }

      history_.emplace_back(now, position_);

      while (history_.size() >= 2 &&
             history_[1].first <= now - stall_window_)
      {
        history_.pop_front();
      }

      if (history_.size() < 2 ||
          now - history_.front().first < stall_window_)
      {
        return;
      }

      // A gap in messages is not evidence of a stationary encoder.
      for (std::size_t i = 1; i < history_.size(); ++i) {
        if (history_[i].first - history_[i - 1].first >
            stall_window_ / 2.0)
        {
          history_.clear();
          history_.emplace_back(now, position_);
          return;
        }
      }

      double minimum = history_.front().second;
      double maximum = minimum;

      for (const auto & sample : history_) {
        minimum = std::min(minimum, sample.second);
        maximum = std::max(maximum, sample.second);
      }

      if (maximum - minimum > stall_distance_) {
        return;
      }

      if (state_ == State::FIRST_SWEEP) {
        first_endpoint_ = position_;
        RCLCPP_INFO(
          get_logger(), "First endpoint: %.6f rad", first_endpoint_);
        start_sweep(State::SECOND_SWEEP, 1.0);
        return;
      }

      const double span = position_ - first_endpoint_;

      if (span <= 2.0 * center_tolerance_ / 0.9) {
        stop("Measured range insufficient to define center and barriers");
        return;
      }

      center_ = (first_endpoint_ + position_) / 2.0;
      barrier_ = 0.9 * span / 2.0;
      calibrated_ = true;

      RCLCPP_INFO(
        get_logger(),
        "min=%.6f rad, max=%.6f rad, center=%.6f rad; "
        "barriers relative to center: [%.6f, %.6f] rad",
        first_endpoint_, position_, center_, -barrier_, barrier_);

      state_ = State::CENTERING;
      command(std::clamp(center_ - position_, -speed_, speed_));

    } catch (const std::exception & e) {
      stop(e.what());
    }
  }

  void tick()
  {
    switch (state_) {
      case State::WAITING:
        if (have_position_ &&
            idle_->service_is_ready() &&
            velocity_mode_->service_is_ready() &&
            enable_->service_is_ready())
        {
          call(idle_, State::IDLE_SETTLE);
        }
        break;

      case State::IDLE_SETTLE:
        if (steady() - state_since_ >= 0.2) {
          // Configure velocity/passthrough while IDLE.
          call(velocity_mode_, State::ENABLE);
        }
        break;

      case State::ENABLE:
        command(0.0);
        call(enable_, State::ENABLE_SETTLE);
        break;

      case State::ENABLE_SETTLE:
        if (steady() - state_since_ >= 0.2) {
          start_sweep(State::FIRST_SWEEP, -1.0);
        }
        break;

      case State::FIRST_SWEEP:
      case State::SECOND_SWEEP:
        command(direction_ * speed_);
        break;

      default:
        break;
    }
  }

  State state_{State::WAITING};

  const double speed_{1.0};             // rad/s
  const double stall_window_{0.5};      // s
  const double stall_distance_{0.02};   // rad
  const double center_tolerance_{0.05}; // rad

  double position_{0.0};
  double first_endpoint_{0.0};
  double center_{0.0}, barrier_{0.0};
  double direction_{-1.0};
  double state_since_{0.0}, observe_after_{0.0};

  bool have_position_{false};
  bool calibrated_{false}, barriers_active_{false};

  std::deque<std::pair<double, double>> history_;

  Client::SharedPtr idle_, velocity_mode_, enable_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr feedback_sub_;
  rclcpp::Service<Trigger>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OverrunNode>());
  rclcpp::shutdown();
  return 0;
}
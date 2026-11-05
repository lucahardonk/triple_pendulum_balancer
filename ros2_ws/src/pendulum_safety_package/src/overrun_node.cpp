#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <odrive_can/msg/control_message.hpp>
#include <odrive_can/msg/controller_status.hpp>
#include <odrive_can/srv/axis_state.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
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
  using AxisState = odrive_can::srv::AxisState;
  using ControlMessage = odrive_can::msg::ControlMessage;
  using ControllerStatus = odrive_can::msg::ControllerStatus;
  using Client = rclcpp::Client<AxisState>;
  using Clock = std::chrono::steady_clock;

  OverrunNode() : Node("overrun_node")
  {
    // odrive_can subscribes with a reliable QoS: the publisher must be reliable.
    command_pub_ = create_publisher<ControlMessage>(
      "/odrive_axis0/control_message", rclcpp::QoS(10));

    position_pub_ = create_publisher<std_msgs::msg::Float64>(
      "~/position_rad", 10);

    axis_state_ = create_client<AxisState>(
      "/odrive_axis0/request_axis_state");

    feedback_sub_ = create_subscription<ControllerStatus>(
      "/odrive_axis0/controller_status", rclcpp::QoS(10).best_effort(),
      [this](ControllerStatus::SharedPtr msg) {
        feedback(*msg);
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

  // Velocity command in rad/s, sent to the ODrive in turns/s.
  void command(double velocity)
  {
    ControlMessage msg;
    msg.control_mode = kVelocityControl;
    msg.input_mode = kPassthrough;
    msg.input_vel = static_cast<float>(velocity / kTau);
    msg.input_torque = 0.0F;
    command_pub_->publish(msg);
  }

  static std::shared_ptr<AxisState::Request> request(uint32_t state)
  {
    auto req = std::make_shared<AxisState::Request>();
    req->axis_requested_state = state;
    return req;
  }

  void call(uint32_t requested, State next)
  {
    state_ = State::SERVICE;

    axis_state_->async_send_request(
      request(requested),
      [this, requested, next](Client::SharedFuture future) {
        if (state_ == State::STOPPED) {
          return;
        }

        try {
          const auto response = future.get();
          if (response->axis_state != requested) {
            stop(
              "Axis state " + std::to_string(response->axis_state) +
              " instead of " + std::to_string(requested) +
              " (errors 0x" + to_hex(response->active_errors) + ")");
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
    axis_state_->async_send_request(request(kIdle));
  }

  static std::string to_hex(uint32_t value)
  {
    char buffer[9];
    std::snprintf(buffer, sizeof(buffer), "%X", value);
    return buffer;
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

  void feedback(const ControllerStatus & status)
  {
    try {
      // odrive_can reports turns: convert to rad.
      position_ = static_cast<double>(status.pos_estimate) * kTau;
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
          call(kIdle, State::READY);
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
        if (have_position_ && axis_state_->service_is_ready()) {
          call(kIdle, State::IDLE_SETTLE);
        }
        break;

      case State::IDLE_SETTLE:
        if (steady() - state_since_ >= 0.2) {
          // Velocity/passthrough with zero setpoint is set while IDLE.
          command(0.0);
          state_ = State::ENABLE;
        }
        break;

      case State::ENABLE:
        command(0.0);
        call(kClosedLoop, State::ENABLE_SETTLE);
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

  static constexpr uint32_t kIdle = 1;
  static constexpr uint32_t kClosedLoop = 8;
  static constexpr uint32_t kVelocityControl = 2;
  static constexpr uint32_t kPassthrough = 1;
  static constexpr double kTau = 2.0 * M_PI;

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

  Client::SharedPtr axis_state_;
  rclcpp::Publisher<ControlMessage>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_pub_;
  rclcpp::Subscription<ControllerStatus>::SharedPtr feedback_sub_;
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
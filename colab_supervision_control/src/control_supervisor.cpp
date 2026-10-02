#include "colab_supervision_control/control_supervisor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace colab_supervision_control
{

ControlSupervisor::ControlSupervisor()
: Node("control_supervisor")
{
  // ============================================================
  // TOPICS
  // ============================================================
  interaction_topic_ = declare_parameter<std::string>(
    "interaction_topic", "/robot_operator_interaction");

  action_topic_ = declare_parameter<std::string>(
    "action_topic", "/control/action");

  speed_topic_ = declare_parameter<std::string>(
    "speed_topic", "/control/speed_percent");

  actions_allowed_topic_ = declare_parameter<std::string>(
    "actions_allowed_topic", "/control/actions_allowed");

  status_topic_ = declare_parameter<std::string>(
    "status_topic", "/control/status");

  // ============================================================
  // DISTANCE / SPEED POLICY
  // ============================================================
  full_speed_distance_ = declare_parameter<double>(
    "full_speed_distance", 1.900);

  half_speed_distance_ = declare_parameter<double>(
    "half_speed_distance", 1.600);

  minimum_speed_distance_ = declare_parameter<double>(
    "minimum_speed_distance", 0.400);

  full_speed_percent_ = declare_parameter<double>(
    "full_speed_percent", 100.0);

  half_speed_percent_ = declare_parameter<double>(
    "half_speed_percent", 50.0);

  minimum_speed_percent_ = declare_parameter<double>(
    "minimum_speed_percent", 5.0);

  robot_not_detected_min_distance_ = declare_parameter<double>(
    "robot_not_detected_min_distance", 1.650);

  stop_distance_ = declare_parameter<double>(
    "stop_distance", 1.600);

  // ============================================================
  // TEMPORAL POLICY
  // ============================================================
  warning_history_size_ = declare_parameter<int>(
    "warning_history_size", 5);

  danger_pause_frames_ = declare_parameter<int>(
    "danger_pause_frames", 5);

  distance_memory_timeout_ = declare_parameter<double>(
    "distance_memory_timeout", 1.0);

  // ============================================================
  // VALIDATION
  // ============================================================
  if (full_speed_distance_ <= half_speed_distance_) {
    throw std::invalid_argument(
      "full_speed_distance must be greater than half_speed_distance");
  }

  if (half_speed_distance_ <= minimum_speed_distance_) {
    throw std::invalid_argument(
      "half_speed_distance must be greater than minimum_speed_distance");
  }

  if (minimum_speed_percent_ < 0.0 ||
      half_speed_percent_ < 0.0 ||
      full_speed_percent_ < 0.0) {
    throw std::invalid_argument("Speed percentages must be non-negative");
  }

  if (warning_history_size_ < 2) {
    throw std::invalid_argument("warning_history_size must be at least 2");
  }

  if (danger_pause_frames_ < 1) {
    throw std::invalid_argument("danger_pause_frames must be at least 1");
  }

  if (distance_memory_timeout_ <= 0.0) {
    throw std::invalid_argument(
      "distance_memory_timeout must be greater than zero");
  }

  // ============================================================
  // ROS INTERFACES
  // ============================================================
  interaction_sub_ =
    create_subscription<
      colab_supervision_interfaces::msg::RobotOperatorInteraction>(
      interaction_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
        &ControlSupervisor::interaction_callback,
        this,
        std::placeholders::_1));

  action_pub_ = create_publisher<std_msgs::msg::String>(action_topic_, 10);
  speed_pub_ = create_publisher<std_msgs::msg::Float32>(speed_topic_, 10);
  actions_allowed_pub_ =
    create_publisher<std_msgs::msg::Bool>(actions_allowed_topic_, 10);
  status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);

  RCLCPP_INFO(get_logger(), "====================================================");
  RCLCPP_INFO(get_logger(), "COLAB SUPERVISION CONTROL - Control Supervisor");
  RCLCPP_INFO(get_logger(), "Interaction topic: %s", interaction_topic_.c_str());
  RCLCPP_INFO(
    get_logger(),
    "Speed profile: %.3f m -> %.1f%% | %.3f m -> %.1f%% | %.3f m -> %.1f%%",
    full_speed_distance_, full_speed_percent_,
    half_speed_distance_, half_speed_percent_,
    minimum_speed_distance_, minimum_speed_percent_);
  RCLCPP_INFO(
    get_logger(),
    "Robot not detected requires distance >= %.3f m for actions",
    robot_not_detected_min_distance_);
  RCLCPP_INFO(get_logger(), "STOP distance: %.3f m", stop_distance_);
  RCLCPP_INFO(
    get_logger(), "PAUSE after %d consecutive danger frames", danger_pause_frames_);
  RCLCPP_INFO(get_logger(), "====================================================");
}

void ControlSupervisor::interaction_callback(
  const colab_supervision_interfaces::msg::RobotOperatorInteraction::SharedPtr msg)
{
  evaluate(*msg);
}

void ControlSupervisor::evaluate(
  const colab_supervision_interfaces::msg::RobotOperatorInteraction & msg)
{
  const bool operator_detected = msg.operator_detected;
  const bool robot_detected = msg.robot_detected;

  // ------------------------------------------------------------
  // Current distance. When the interaction estimate is invalid,
  // preserve the last valid value only for a short configurable
  // interval.
  // ------------------------------------------------------------
  bool distance_valid =
    msg.interaction_valid && std::isfinite(msg.distance) && msg.distance >= 0.0;

  double distance = 0.0;
  const auto now = get_clock()->now();

  if (distance_valid) {
    distance = msg.distance;
    last_valid_distance_ = distance;
    last_valid_distance_time_ = now;
  } else if (last_valid_distance_.has_value()) {
    const double age = (now - last_valid_distance_time_).seconds();
    if (age <= distance_memory_timeout_) {
      distance = *last_valid_distance_;
      distance_valid = true;
    }
  }

  // ------------------------------------------------------------
  // Risk classification
  // ------------------------------------------------------------
  const bool warning = (msg.collision_risk == "WARNING");
  const bool danger =
    warning ||
    msg.collision_risk == "DANGEROUS" ||
    msg.collision_risk == "CRITICAL" ||
    msg.collision_imminent;

  // Evaluate previous frames before adding current frame to history.
  const bool previous_two_warning = previous_two_warnings();

  SampleState current_sample;
  current_sample.operator_detected = operator_detected;
  current_sample.robot_detected = robot_detected;
  current_sample.warning = warning;
  current_sample.danger = danger;
  current_sample.distance_valid = distance_valid;
  current_sample.distance = distance;
  push_history(current_sample);

  const bool any_warning_last_five = any_warning_last_five();
  const bool five_danger = five_consecutive_danger();

  std::string reason;
  const Action action = determine_action(
    operator_detected,
    robot_detected,
    distance_valid,
    distance,
    previous_two_warning,
    any_warning_last_five,
    five_danger,
    reason);

  double speed_percent = 0.0;
  bool actions_allowed = false;

  if (action == Action::ALLOW_ACTIONS || action == Action::LIMIT_SPEED) {
    speed_percent = compute_speed_percent(distance);
    actions_allowed = true;
  }

  publish_decision(
    action,
    speed_percent,
    actions_allowed,
    reason,
    msg,
    distance_valid,
    distance);

  initialized_ = true;
}

bool ControlSupervisor::previous_two_warnings() const
{
  if (history_.size() < 2) {
    return false;
  }

  return history_[history_.size() - 2].warning &&
         history_[history_.size() - 1].warning;
}

bool ControlSupervisor::any_warning_last_five() const
{
  const std::size_t start =
    history_.size() > static_cast<std::size_t>(warning_history_size_)
      ? history_.size() - static_cast<std::size_t>(warning_history_size_)
      : 0U;

  for (std::size_t index = start; index < history_.size(); ++index) {
    if (history_[index].warning) {
      return true;
    }
  }
  return false;
}

bool ControlSupervisor::five_consecutive_danger() const
{
  const std::size_t required = static_cast<std::size_t>(danger_pause_frames_);

  if (history_.size() < required) {
    return false;
  }

  const std::size_t start = history_.size() - required;
  for (std::size_t index = start; index < history_.size(); ++index) {
    if (!history_[index].danger) {
      return false;
    }
  }
  return true;
}

void ControlSupervisor::push_history(const SampleState & sample)
{
  history_.push_back(sample);

  const std::size_t max_history = static_cast<std::size_t>(
    std::max(warning_history_size_, danger_pause_frames_) + 2);

  while (history_.size() > max_history) {
    history_.pop_front();
  }
}

double ControlSupervisor::compute_speed_percent(double distance) const
{
  if (distance >= full_speed_distance_) {
    return full_speed_percent_;
  }

  // 1.900 m -> 100% and 1.600 m -> 50%
  if (distance >= half_speed_distance_) {
    const double normalized =
      (full_speed_distance_ - distance) /
      (full_speed_distance_ - half_speed_distance_);

    const double ratio = half_speed_percent_ / full_speed_percent_;
    return full_speed_percent_ * std::pow(ratio, normalized);
  }

  // 1.600 m -> 50% and 0.400 m -> 5%
  if (distance >= minimum_speed_distance_) {
    const double normalized =
      (half_speed_distance_ - distance) /
      (half_speed_distance_ - minimum_speed_distance_);

    const double ratio = minimum_speed_percent_ / half_speed_percent_;
    return half_speed_percent_ * std::pow(ratio, normalized);
  }

  return minimum_speed_percent_;
}

ControlSupervisor::Action ControlSupervisor::determine_action(
  bool operator_detected,
  bool robot_detected,
  bool distance_valid,
  double distance,
  bool previous_two_warning,
  bool any_warning_last_five,
  bool five_danger,
  std::string & reason) const
{
  // ============================================================
  // PRIORITY 1 - STOP
  // ============================================================
  if (!operator_detected && previous_two_warning) {
    reason =
      "STOP: previous two camera observations were WARNING "
      "and operator is no longer detected";
    return Action::STOP;
  }

  if (!operator_detected && distance_valid && distance < stop_distance_) {
    reason =
      "STOP: operator lost after valid distance below 1.600 m";
    return Action::STOP;
  }

  if (!robot_detected && distance_valid && distance < stop_distance_) {
    reason =
      "STOP: robot lost while last valid interaction distance was below 1.600 m";
    return Action::STOP;
  }

  // ============================================================
  // PRIORITY 2 - PAUSE
  // ============================================================
  if (five_danger) {
    reason = "PAUSE: five consecutive dangerous camera observations";
    return Action::PAUSE;
  }

  // ============================================================
  // PRIORITY 3 - ROBOT NOT DETECTED
  // ============================================================
  if (!robot_detected) {
    if (!any_warning_last_five) {
      reason =
        "WAIT: robot not detected and no WARNING in the last five frames";
      return Action::WAIT;
    }

    if (!distance_valid) {
      reason =
        "WAIT: robot not detected and no valid operator distance";
      return Action::WAIT;
    }

    if (distance < robot_not_detected_min_distance_) {
      reason =
        "WAIT: robot not detected and last valid distance is below 1.650 m";
      return Action::WAIT;
    }
  }

  // ============================================================
  // PRIORITY 4 - OPERATOR / DISTANCE VALIDATION
  // ============================================================
  if (!operator_detected) {
    reason = "WAIT: operator not detected without a STOP condition";
    return Action::WAIT;
  }

  if (!distance_valid) {
    reason = "WAIT: valid operator-robot distance unavailable";
    return Action::WAIT;
  }

  // ============================================================
  // PRIORITY 5 - SPEED CONTROL
  // ============================================================
  const double speed = compute_speed_percent(distance);

  if (speed >= full_speed_percent_ - 1e-3) {
    reason = "ALLOW_ACTIONS: operator distance supports full speed";
    return Action::ALLOW_ACTIONS;
  }

  reason =
    "LIMIT_SPEED: operator distance requires exponential speed reduction";
  return Action::LIMIT_SPEED;
}

const char * ControlSupervisor::action_to_string(Action action)
{
  switch (action) {
    case Action::WAIT: return "WAIT";
    case Action::ALLOW_ACTIONS: return "ALLOW_ACTIONS";
    case Action::LIMIT_SPEED: return "LIMIT_SPEED";
    case Action::PAUSE: return "PAUSE";
    case Action::STOP: return "STOP";
    default: return "WAIT";
  }
}

void ControlSupervisor::publish_decision(
  Action action,
  double speed_percent,
  bool actions_allowed,
  const std::string & reason,
  const colab_supervision_interfaces::msg::RobotOperatorInteraction & msg,
  bool distance_valid,
  double distance)
{
  std_msgs::msg::String action_message;
  action_message.data = action_to_string(action);
  action_pub_->publish(action_message);

  std_msgs::msg::Float32 speed_message;
  speed_message.data = static_cast<float>(speed_percent);
  speed_pub_->publish(speed_message);

  std_msgs::msg::Bool allowed_message;
  allowed_message.data = actions_allowed;
  actions_allowed_pub_->publish(allowed_message);

  std_msgs::msg::String status_message;
  status_message.data = reason;
  status_pub_->publish(status_message);

  if (!initialized_ ||
      action == Action::STOP ||
      action == Action::PAUSE ||
      action == Action::WAIT) {
    const std::string distance_text =
      distance_valid ? std::to_string(distance) : std::string("invalid");

    RCLCPP_INFO(
      get_logger(),
      "Action=%s | speed=%.1f%% | operator=%s | robot=%s | "
      "distance=%s m | risk=%s | reason=%s",
      action_to_string(action),
      speed_percent,
      msg.operator_detected ? "YES" : "NO",
      msg.robot_detected ? "YES" : "NO",
      distance_text.c_str(),
      msg.collision_risk.c_str(),
      reason.c_str());
  }
}

}  // namespace colab_supervision_control

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<colab_supervision_control::ControlSupervisor>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("control_supervisor"),
      "Node initialization failed: %s",
      error.what());
  }

  rclcpp::shutdown();
  return 0;
}

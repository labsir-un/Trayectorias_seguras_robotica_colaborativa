#include "colab_supervision_control/control_supervisor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace colab_supervision_control
{

// ============================================================================
// CONSTRUCTOR
// ============================================================================

ControlSupervisor::ControlSupervisor()
: Node("control_supervisor")
{
  // ==========================================================================
  // TOPICS
  //
  // Input:
  //   /robot_operator_interaction
  //
  // Output:
  //   /control/robot_operator_state
  //
  // The output is an intermediate, preprocessed state. It is NOT a final
  // control command. Final safety decisions are delegated to control_decision.
  // ==========================================================================

  interaction_topic_ = declare_parameter<std::string>(
    "interaction_topic",
    "/robot_operator_interaction"
  );

  supervision_topic_ = declare_parameter<std::string>(
    "supervision_topic",
    "/control/robot_operator_state"
  );

  // ==========================================================================
  // TEMPORAL PREPROCESSING
  // ============================================================================

  warning_history_size_ = declare_parameter<int>(
    "warning_history_size",
    5
  );

  danger_persistent_frames_ = declare_parameter<int>(
    "danger_persistent_frames",
    5
  );

  distance_memory_timeout_ = declare_parameter<double>(
    "distance_memory_timeout",
    1.0
  );

  // ==========================================================================
  // PARAMETER VALIDATION
  // ============================================================================

  if (warning_history_size_ < 1) {
    throw std::invalid_argument(
      "warning_history_size must be at least 1"
    );
  }

  if (danger_persistent_frames_ < 1) {
    throw std::invalid_argument(
      "danger_persistent_frames must be at least 1"
    );
  }

  if (distance_memory_timeout_ <= 0.0) {
    throw std::invalid_argument(
      "distance_memory_timeout must be greater than zero"
    );
  }

  // ==========================================================================
  // ROS SUBSCRIPTION
  // ============================================================================

  interaction_sub_ =
    create_subscription<
      colab_supervision_interfaces::msg::RobotOperatorInteraction
    >(
      interaction_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
        &ControlSupervisor::interaction_callback,
        this,
        std::placeholders::_1
      )
    );

  // ==========================================================================
  // ROS PUBLISHER
  // ============================================================================

  supervision_pub_ =
    create_publisher<
      colab_supervision_interfaces::msg::RobotOperatorSupervision
    >(
      supervision_topic_,
      10
    );

  // ==========================================================================
  // INITIALIZATION LOG
  // ============================================================================

  RCLCPP_INFO(
    get_logger(),
    "===================================================="
  );

  RCLCPP_INFO(
    get_logger(),
    "COLAB SUPERVISION CONTROL - Control Supervisor"
  );

  RCLCPP_INFO(
    get_logger(),
    "Input interaction topic: %s",
    interaction_topic_.c_str()
  );

  RCLCPP_INFO(
    get_logger(),
    "Processed output topic: %s",
    supervision_topic_.c_str()
  );

  RCLCPP_INFO(
    get_logger(),
    "Warning temporal window: %d samples",
    warning_history_size_
  );

  RCLCPP_INFO(
    get_logger(),
    "Persistent danger threshold: %d consecutive samples",
    danger_persistent_frames_
  );

  RCLCPP_INFO(
    get_logger(),
    "Distance memory timeout: %.2f s",
    distance_memory_timeout_
  );

  RCLCPP_INFO(
    get_logger(),
    "This node does NOT generate robot control actions"
  );

  RCLCPP_INFO(
    get_logger(),
    "===================================================="
  );
}


// ============================================================================
// INTERACTION CALLBACK
// ============================================================================

void ControlSupervisor::interaction_callback(
  const colab_supervision_interfaces::msg::RobotOperatorInteraction::SharedPtr msg)
{
  process_interaction(*msg);
}


// ============================================================================
// MAIN PREPROCESSING
// ============================================================================

void ControlSupervisor::process_interaction(
  const colab_supervision_interfaces::msg::RobotOperatorInteraction & msg)
{
  // ==========================================================================
  // CURRENT RISK CLASSIFICATION
  //
  // The collision risk itself has already been calculated by
  // robot_operator_interaction. This node does not recalculate it.
  // It only derives temporal indicators from the already classified result.
  // ==========================================================================

  const bool warning =
    msg.collision_risk == "WARNING";

  const bool danger =
    warning ||
    msg.collision_risk == "DANGEROUS" ||
    msg.collision_risk == "CRITICAL" ||
    msg.collision_imminent;

  // ==========================================================================
  // DISTANCE PREPROCESSING
  //
  // The current valid distance is used directly. If it is temporarily
  // unavailable, the last valid processed distance can be retained for the
  // configured timeout. No new perception or geometric calculation is made.
  // ==========================================================================

  double distance = 0.0;
  bool distance_from_memory = false;

  const bool distance_valid =
    get_distance(
      msg,
      distance,
      distance_from_memory
    );

  // ==========================================================================
  // TEMPORAL HISTORY
  // ============================================================================

  SampleState current_sample;

  current_sample.operator_detected =
    msg.operator_detected;

  current_sample.robot_detected =
    msg.robot_detected;

  current_sample.warning =
    warning;

  current_sample.danger =
    danger;

  current_sample.distance_valid =
    distance_valid;

  current_sample.distance =
    distance;

  push_history(current_sample);

  const bool warning_in_window =
    any_warning_last_window();

  const bool danger_is_persistent =
    danger_persistent();

  // ==========================================================================
  // DESCRIPTIVE STATE
  // ============================================================================

  const std::string supervision_state =
    determine_supervision_state(
      msg,
      distance_valid,
      distance_from_memory
    );

  // ==========================================================================
  // PUBLISH PROCESSED STATE
  // ============================================================================

  publish_supervision_state(
    msg,
    distance_valid,
    distance,
    distance_from_memory,
    warning_in_window,
    danger_is_persistent,
    supervision_state
  );

  initialized_ = true;
}


// ============================================================================
// DISTANCE MEMORY
// ============================================================================

bool ControlSupervisor::get_distance(
  const colab_supervision_interfaces::msg::RobotOperatorInteraction & msg,
  double & distance,
  bool & distance_from_memory)
{
  distance_from_memory = false;

  const auto now =
    get_clock()->now();

  // --------------------------------------------------------------------------
  // Current valid distance
  // --------------------------------------------------------------------------

  if (
    msg.interaction_valid &&
    std::isfinite(msg.distance) &&
    msg.distance >= 0.0
  ) {
    distance =
      static_cast<double>(msg.distance);

    last_valid_distance_ =
      distance;

    last_valid_distance_time_ =
      now;

    return true;
  }

  // --------------------------------------------------------------------------
  // Last valid distance
  // --------------------------------------------------------------------------

  if (last_valid_distance_.has_value()) {

    const double age =
      (now - last_valid_distance_time_).seconds();

    if (age <= distance_memory_timeout_) {

      distance =
        *last_valid_distance_;

      distance_from_memory =
        true;

      return true;
    }
  }

  return false;
}


// ============================================================================
// TEMPORAL WARNING WINDOW
// ============================================================================

bool ControlSupervisor::any_warning_last_window() const
{
  const std::size_t required_history =
    static_cast<std::size_t>(
      warning_history_size_
    );

  const std::size_t start =
    history_.size() > required_history
      ? history_.size() - required_history
      : 0U;

  for (
    std::size_t index = start;
    index < history_.size();
    ++index
  ) {

    if (history_[index].warning) {
      return true;
    }
  }

  return false;
}


// ============================================================================
// PERSISTENT DANGER
// ============================================================================

bool ControlSupervisor::danger_persistent() const
{
  const std::size_t required =
    static_cast<std::size_t>(
      danger_persistent_frames_
    );

  if (history_.size() < required) {
    return false;
  }

  const std::size_t start =
    history_.size() - required;

  for (
    std::size_t index = start;
    index < history_.size();
    ++index
  ) {

    if (!history_[index].danger) {
      return false;
    }
  }

  return true;
}


// ============================================================================
// HISTORY MANAGEMENT
// ============================================================================

void ControlSupervisor::push_history(
  const SampleState & sample)
{
  history_.push_back(sample);

  const std::size_t max_history =
    static_cast<std::size_t>(
      std::max(
        warning_history_size_,
        danger_persistent_frames_
      )
    );

  while (history_.size() > max_history) {
    history_.pop_front();
  }
}


// ============================================================================
// SUPERVISION STATE DESCRIPTION
// ============================================================================

std::string ControlSupervisor::determine_supervision_state(
  const colab_supervision_interfaces::msg::RobotOperatorInteraction & msg,
  bool distance_valid,
  bool distance_from_memory) const
{
  if (!msg.operator_detected) {
    return "OPERATOR_NOT_DETECTED";
  }

  if (!msg.robot_detected) {
    return "ROBOT_NOT_DETECTED";
  }

  if (!distance_valid) {
    return "INVALID_INTERACTION";
  }

  if (distance_from_memory) {
    return "DISTANCE_MEMORY";
  }

  return "VALID";
}


// ============================================================================
// PUBLISH PROCESSED STATE
// ============================================================================

void ControlSupervisor::publish_supervision_state(
  const colab_supervision_interfaces::msg::RobotOperatorInteraction & msg,
  bool distance_valid,
  double distance,
  bool distance_from_memory,
  bool warning_in_window,
  bool danger_is_persistent,
  const std::string & supervision_state)
{
  colab_supervision_interfaces::msg::RobotOperatorSupervision state;

  // --------------------------------------------------------------------------
  // Detection and availability information
  // --------------------------------------------------------------------------

  state.operator_detected =
    msg.operator_detected;

  state.robot_detected =
    msg.robot_detected;

  state.operator_position_valid =
    msg.operator_position_valid;

  state.robot_position_valid =
    msg.robot_position_valid;

  state.interaction_valid =
    msg.interaction_valid;

  // --------------------------------------------------------------------------
  // Detection confidence
  // --------------------------------------------------------------------------

  state.operator_confidence =
    msg.operator_confidence;

  state.robot_confidence =
    msg.robot_confidence;

  // --------------------------------------------------------------------------
  // Distance
  // --------------------------------------------------------------------------

  state.distance =
    static_cast<float>(distance);

  state.distance_valid =
    distance_valid;

  state.distance_from_memory =
    distance_from_memory;

  // --------------------------------------------------------------------------
  // Motion variables already estimated upstream
  // --------------------------------------------------------------------------

  state.relative_velocity =
    msg.relative_velocity;

  state.approach_velocity =
    msg.approach_velocity;

  state.time_to_collision =
    msg.time_to_collision;

  state.time_to_collision_valid =
    msg.time_to_collision_valid;

  // --------------------------------------------------------------------------
  // Collision risk already estimated upstream
  // --------------------------------------------------------------------------

  state.collision_risk_score =
    msg.collision_risk_score;

  state.collision_risk =
    msg.collision_risk;

  state.collision_imminent =
    msg.collision_imminent;

  // --------------------------------------------------------------------------
  // Temporal conditioning performed by this node
  // --------------------------------------------------------------------------

  state.warning_in_window =
    warning_in_window;

  state.danger_persistent =
    danger_is_persistent;

  state.supervision_state =
    supervision_state;

  supervision_pub_->publish(state);

  // --------------------------------------------------------------------------
  // Logging only on significant state changes or safety conditions
  // --------------------------------------------------------------------------

  if (
    !initialized_ ||
    msg.collision_risk == "CRITICAL" ||
    msg.collision_risk == "DANGEROUS" ||
    msg.collision_imminent ||
    distance_from_memory ||
    !msg.operator_detected ||
    !msg.robot_detected ||
    !distance_valid
  ) {
    const std::string distance_text =
      distance_valid
        ? std::to_string(distance)
        : std::string("invalid");

    RCLCPP_INFO(
      get_logger(),
      "State=%s | operator=%s | robot=%s | distance=%s m | "
      "memory=%s | risk=%s | TTC_valid=%s | warning_window=%s | "
      "danger_persistent=%s",
      supervision_state.c_str(),
      msg.operator_detected ? "YES" : "NO",
      msg.robot_detected ? "YES" : "NO",
      distance_text.c_str(),
      distance_from_memory ? "YES" : "NO",
      msg.collision_risk.c_str(),
      msg.time_to_collision_valid ? "YES" : "NO",
      warning_in_window ? "YES" : "NO",
      danger_is_persistent ? "YES" : "NO"
    );
  }
}

}  // namespace colab_supervision_control


// ============================================================================
// MAIN
// ============================================================================

int main(
  int argc,
  char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    auto node =
      std::make_shared<
        colab_supervision_control::ControlSupervisor
      >();

    rclcpp::spin(node);

  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("control_supervisor"),
      "Node initialization failed: %s",
      error.what()
    );
  }

  rclcpp::shutdown();

  return 0;
}

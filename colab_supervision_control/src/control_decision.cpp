#include "colab_supervision_control/control_decision.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace colab_supervision_control
{

// ============================================================================
// CONSTRUCTOR
// ============================================================================

ControlDecision::ControlDecision()
: Node("control_decision")
{
  // ==========================================================================
  // INPUT TOPICS
  // ==========================================================================

  operator_state_topic_ = declare_parameter<std::string>(
    "operator_state_topic",
    "/operator/state_analisis"
  );

  interaction_topic_ = declare_parameter<std::string>(
    "interaction_topic",
    "/robot_operator_interaction"
  );

  trajectory_topic_ = declare_parameter<std::string>(
    "trajectory_topic",
    "/control/trajectory"
  );

  // ==========================================================================
  // OUTPUT TOPIC
  // ==========================================================================

  command_topic_ = declare_parameter<std::string>(
    "command_topic",
    "/control/command"
  );

  // ==========================================================================
  // DECISION TIMING
  // ==========================================================================

  decision_rate_hz_ = declare_parameter<double>(
    "decision_rate_hz",
    10.0
  );

  max_input_age_sec_ = declare_parameter<double>(
    "max_input_age_sec",
    0.50
  );

  max_trajectory_age_sec_ = declare_parameter<double>(
    "max_trajectory_age_sec",
    2.0
  );

  // ==========================================================================
  // ZONES
  //
  // The interaction interface contains distance but not an explicit zone.
  // Therefore, the zone is determined from configurable distance boundaries.
  //
  // Defaults:
  //   Z3: d < 0.250 m
  //   Z4: 0.250 m <= d < 0.400 m
  //   Z5: 0.400 m <= d <= 1.600 m
  //   Z6: d > 1.600 m
  // ==========================================================================

  zone3_max_distance_ = declare_parameter<double>(
    "zone3_max_distance",
    0.250
  );

  zone4_max_distance_ = declare_parameter<double>(
    "zone4_max_distance",
    0.400
  );

  zone5_max_distance_ = declare_parameter<double>(
    "zone5_max_distance",
    1.600
  );

  // ==========================================================================
  // SPEED PROFILE
  // ==========================================================================

  full_speed_distance_ = declare_parameter<double>(
    "full_speed_distance",
    1.900
  );

  half_speed_distance_ = declare_parameter<double>(
    "half_speed_distance",
    1.600
  );

  minimum_speed_distance_ = declare_parameter<double>(
    "minimum_speed_distance",
    0.400
  );

  full_speed_percent_ = declare_parameter<double>(
    "full_speed_percent",
    100.0
  );

  half_speed_percent_ = declare_parameter<double>(
    "half_speed_percent",
    50.0
  );

  minimum_speed_percent_ = declare_parameter<double>(
    "minimum_speed_percent",
    5.0
  );

  // ==========================================================================
  // OPERATOR CONDITIONS
  // ==========================================================================

  pause_on_fatigue_high_ = declare_parameter<bool>(
    "pause_on_fatigue_high",
    true
  );

  pause_on_attention_other_ = declare_parameter<bool>(
    "pause_on_attention_other",
    true
  );

  pause_on_attention_unknown_ = declare_parameter<bool>(
    "pause_on_attention_unknown",
    true
  );

  // ==========================================================================
  // TRAJECTORY SAFETY
  //
  // There is no explicit trajectory-safety result in the three input topics.
  // Therefore this node uses a conservative current-state check as a temporary
  // decision criterion. This is intentionally isolated in
  // trajectory_keeps_separation(), so a future FK/trajectory evaluator can
  // replace it without changing the decision hierarchy.
  // ==========================================================================

  conservative_trajectory_check_ = declare_parameter<bool>(
    "conservative_trajectory_check",
    true
  );

  trajectory_min_separation_distance_ = declare_parameter<double>(
    "trajectory_min_separation_distance",
    0.400
  );

  // ==========================================================================
  // VALIDATION
  // ==========================================================================

  if (decision_rate_hz_ <= 0.0) {
    throw std::invalid_argument(
      "decision_rate_hz must be greater than zero"
    );
  }

  if (max_input_age_sec_ <= 0.0) {
    throw std::invalid_argument(
      "max_input_age_sec must be greater than zero"
    );
  }

  if (max_trajectory_age_sec_ <= 0.0) {
    throw std::invalid_argument(
      "max_trajectory_age_sec must be greater than zero"
    );
  }

  if (zone3_max_distance_ < 0.0) {
    throw std::invalid_argument(
      "zone3_max_distance must be non-negative"
    );
  }

  if (zone4_max_distance_ <= zone3_max_distance_) {
    throw std::invalid_argument(
      "zone4_max_distance must be greater than zone3_max_distance"
    );
  }

  if (zone5_max_distance_ < zone4_max_distance_) {
    throw std::invalid_argument(
      "zone5_max_distance must be greater than or equal to zone4_max_distance"
    );
  }

  if (full_speed_distance_ <= half_speed_distance_) {
    throw std::invalid_argument(
      "full_speed_distance must be greater than half_speed_distance"
    );
  }

  if (half_speed_distance_ <= minimum_speed_distance_) {
    throw std::invalid_argument(
      "half_speed_distance must be greater than minimum_speed_distance"
    );
  }

  if (trajectory_min_separation_distance_ < 0.0) {
    throw std::invalid_argument(
      "trajectory_min_separation_distance must be non-negative"
    );
  }

  // ==========================================================================
  // SUBSCRIBERS
  // ==========================================================================

  operator_state_sub_ =
    create_subscription<
      colab_supervision_interfaces::msg::OperatorStateAnalysis
    >(
      operator_state_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
        &ControlDecision::operator_state_callback,
        this,
        std::placeholders::_1
      )
    );

  interaction_sub_ =
    create_subscription<
      colab_supervision_interfaces::msg::RobotOperatorInteraction
    >(
      interaction_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
        &ControlDecision::interaction_callback,
        this,
        std::placeholders::_1
      )
    );

  trajectory_sub_ =
    create_subscription<
      moveit_msgs::msg::RobotTrajectory
    >(
      trajectory_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
        &ControlDecision::trajectory_callback,
        this,
        std::placeholders::_1
      )
    );

  // ==========================================================================
  // COMMAND PUBLISHER
  // ==========================================================================

  command_pub_ =
    create_publisher<
      colab_supervision_interfaces::msg::ControlCommand
    >(
      command_topic_,
      10
    );

  // ==========================================================================
  // DECISION TIMER
  // ==========================================================================

  const auto period =
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(
        1.0 / decision_rate_hz_
      )
    );

  decision_timer_ =
    create_wall_timer(
      period,
      std::bind(
        &ControlDecision::decision_timer_callback,
        this
      )
    );

  // ==========================================================================
  // INITIAL LOG
  // ==========================================================================

  RCLCPP_INFO(
    get_logger(),
    "===================================================="
  );

  RCLCPP_INFO(
    get_logger(),
    "CONTROL DECISION NODE"
  );

  RCLCPP_INFO(
    get_logger(),
    "Operator state : %s",
    operator_state_topic_.c_str()
  );

  RCLCPP_INFO(
    get_logger(),
    "Interaction    : %s",
    interaction_topic_.c_str()
  );

  RCLCPP_INFO(
    get_logger(),
    "Trajectory     : %s",
    trajectory_topic_.c_str()
  );

  RCLCPP_INFO(
    get_logger(),
    "Command output  : %s",
    command_topic_.c_str()
  );

  RCLCPP_INFO(
    get_logger(),
    "Decision rate   : %.2f Hz",
    decision_rate_hz_
  );

  RCLCPP_INFO(
    get_logger(),
    "Zones: Z3 < %.3f m | Z4 < %.3f m | Z5 <= %.3f m | Z6 > %.3f m",
    zone3_max_distance_,
    zone4_max_distance_,
    zone5_max_distance_,
    zone5_max_distance_
  );

  RCLCPP_INFO(
    get_logger(),
    "===================================================="
  );
}


// ============================================================================
// CALLBACKS
// ============================================================================

void ControlDecision::operator_state_callback(
  const colab_supervision_interfaces::msg::OperatorStateAnalysis::SharedPtr msg)
{
  latest_operator_state_ = *msg;
  operator_state_received_time_ = get_clock()->now();
}


void ControlDecision::interaction_callback(
  const colab_supervision_interfaces::msg::RobotOperatorInteraction::SharedPtr msg)
{
  latest_interaction_ = *msg;
  interaction_received_time_ = get_clock()->now();
}


void ControlDecision::trajectory_callback(
  const moveit_msgs::msg::RobotTrajectory::SharedPtr msg)
{
  latest_trajectory_ = *msg;
  trajectory_received_time_ = get_clock()->now();
}


// ============================================================================
// TIMER
// ============================================================================

void ControlDecision::decision_timer_callback()
{
  const DecisionResult result =
    evaluate_decision();

  publish_command(result);

  initialized_ = true;
}


// ============================================================================
// INPUT AVAILABILITY
// ============================================================================

bool ControlDecision::is_detection_system_available() const
{
  const auto now =
    get_clock()->now();

  const bool operator_state_ok =
    latest_operator_state_.has_value() &&
    (now - operator_state_received_time_).seconds() <= max_input_age_sec_;

  const bool interaction_ok =
    latest_interaction_.has_value() &&
    (now - interaction_received_time_).seconds() <= max_input_age_sec_;

  return operator_state_ok && interaction_ok;
}


bool ControlDecision::input_data_available() const
{
  return (
    latest_operator_state_.has_value() &&
    latest_interaction_.has_value()
  );
}


// ============================================================================
// OPERATOR CONDITION
// ============================================================================

bool ControlDecision::operator_condition_anomalous() const
{
  if (!latest_operator_state_.has_value()) {
    return true;
  }

  const auto & state =
    *latest_operator_state_;

  if (
    pause_on_fatigue_high_ &&
    state.fatigue_condition == "HIGH"
  ) {
    return true;
  }

  if (
    pause_on_attention_other_ &&
    state.attention_condition == "OTHER"
  ) {
    return true;
  }

  if (
    pause_on_attention_unknown_ &&
    state.attention_condition == "UNKNOWN"
  ) {
    return true;
  }

  return false;
}


// ============================================================================
// COLLISION
// ============================================================================

bool ControlDecision::collision_is_critical() const
{
  if (!latest_interaction_.has_value()) {
    return true;
  }

  const auto & interaction =
    *latest_interaction_;

  return (
    interaction.collision_imminent ||
    interaction.collision_risk == "CRITICAL"
  );
}


// ============================================================================
// ZONE
// ============================================================================

ControlDecision::Zone
ControlDecision::determine_zone(double distance) const
{
  if (!std::isfinite(distance) || distance < 0.0) {
    return Zone::UNKNOWN;
  }

  // Z3: d < 0.250 m
  if (distance < zone3_max_distance_) {
    return Zone::Z3;
  }

  // Z4: 0.250 m <= d < 0.400 m
  if (distance < zone4_max_distance_) {
    return Zone::Z4;
  }

  // Z5: 0.400 m <= d <= 1.600 m
  if (distance <= zone5_max_distance_) {
    return Zone::Z5;
  }

  // Z6: d > 1.600 m
  return Zone::Z6;
}


std::string ControlDecision::zone_to_string(Zone zone) const
{
  switch (zone) {

    case Zone::Z3:
      return "Z3";

    case Zone::Z4:
      return "Z4";

    case Zone::Z5:
      return "Z5";

    case Zone::Z6:
      return "Z6";

    default:
      return "UNKNOWN";
  }
}


// ============================================================================
// TRAJECTORY
// ============================================================================

bool ControlDecision::trajectory_available() const
{
  if (!latest_trajectory_.has_value()) {
    return false;
  }

  return !latest_trajectory_->joint_trajectory.points.empty();
}


bool ControlDecision::trajectory_target_reachable() const
{
  // MoveIt provides a trajectory only when a planning result has produced
  // trajectory points. This is used as the current reachability indication.
  //
  // A future planner-specific result can replace this method if a more
  // explicit reachability status is available.
  return trajectory_available();
}


bool ControlDecision::trajectory_keeps_separation(
  Zone zone,
  double distance) const
{
  // --------------------------------------------------------------------------
  // Z6
  // --------------------------------------------------------------------------
  //
  // At Z6, the decision matrix identifies the condition as safe when there is
  // no anomaly. Therefore the current separation is considered sufficient.
  // --------------------------------------------------------------------------

  if (zone == Zone::Z6) {
    return true;
  }

  // --------------------------------------------------------------------------
  // Conservative current-state check for Z4/Z5
  // --------------------------------------------------------------------------

  if (!conservative_trajectory_check_) {
    return false;
  }

  if (
    distance <
    trajectory_min_separation_distance_
  ) {
    return false;
  }

  if (!latest_interaction_.has_value()) {
    return false;
  }

  const auto & interaction =
    *latest_interaction_;

  if (
    interaction.collision_imminent ||
    interaction.collision_risk == "CRITICAL" ||
    interaction.collision_risk == "DANGEROUS"
  ) {
    return false;
  }

  return true;
}


// ============================================================================
// SPEED PROFILE
// ============================================================================

int ControlDecision::compute_speed_percent(double distance) const
{
  double speed;

  // --------------------------------------------------------------------------
  // >= 1.900 m -> 100 %
  // --------------------------------------------------------------------------

  if (distance >= full_speed_distance_) {

    speed =
      full_speed_percent_;

  // --------------------------------------------------------------------------
  // 1.600 m <= d < 1.900 m -> 100 ... 50 %
  // --------------------------------------------------------------------------

  } else if (distance >= half_speed_distance_) {

    const double normalized =
      (
        full_speed_distance_ -
        distance
      ) /
      (
        full_speed_distance_ -
        half_speed_distance_
      );

    const double ratio =
      half_speed_percent_ /
      full_speed_percent_;

    speed =
      full_speed_percent_ *
      std::pow(
        ratio,
        normalized
      );

  // --------------------------------------------------------------------------
  // 0.400 m <= d < 1.600 m -> 50 ... 5 %
  // --------------------------------------------------------------------------

  } else if (distance >= minimum_speed_distance_) {

    const double normalized =
      (
        half_speed_distance_ -
        distance
      ) /
      (
        half_speed_distance_ -
        minimum_speed_distance_
      );

    const double ratio =
      minimum_speed_percent_ /
      half_speed_percent_;

    speed =
      half_speed_percent_ *
      std::pow(
        ratio,
        normalized
      );

  // --------------------------------------------------------------------------
  // < 0.400 m -> 5 %
  // --------------------------------------------------------------------------

  } else {

    speed =
      minimum_speed_percent_;
  }

  speed =
    std::clamp(
      speed,
      minimum_speed_percent_,
      full_speed_percent_
    );

  return static_cast<int>(
    std::lround(speed)
  );
}


// ============================================================================
// COMMAND BUILDERS
// ============================================================================

ControlDecision::DecisionResult
ControlDecision::make_emergency_stop(
  const std::string & reason) const
{
  return {
    "EMERGENCY_STOP",
    "ROBOT",
    "STATE",
    "EMERGENCY_STOP",
    reason
  };
}


ControlDecision::DecisionResult
ControlDecision::make_safe_stop(
  const std::string & reason) const
{
  return {
    "STOP",
    "ROBOT",
    "STATE",
    "SAFE_STOP",
    reason
  };
}


ControlDecision::DecisionResult
ControlDecision::make_pause(
  const std::string & reason) const
{
  return {
    "PAUSE",
    "ROBOT",
    "STATE",
    "PAUSED",
    reason
  };
}


ControlDecision::DecisionResult
ControlDecision::make_increase_distance(
  int speed_percent,
  const std::string & reason) const
{
  return {
    "INCREASE_DISTANCE",
    "ROBOT",
    "SPEED",
    std::to_string(speed_percent),
    reason
  };
}


ControlDecision::DecisionResult
ControlDecision::make_limit_speed(
  int speed_percent,
  const std::string & reason) const
{
  return {
    "SET_SPEED",
    "ROBOT",
    "SPEED",
    std::to_string(speed_percent),
    reason
  };
}


ControlDecision::DecisionResult
ControlDecision::make_normal_operation(
  int speed_percent,
  const std::string & reason) const
{
  return {
    "EXECUTE",
    "TRAJECTORY",
    "SPEED",
    std::to_string(speed_percent),
    reason
  };
}


// ============================================================================
// MAIN DECISION HIERARCHY
// ============================================================================

ControlDecision::DecisionResult
ControlDecision::evaluate_decision()
{
  // ==========================================================================
  // PRIORITY 1
  //
  // Any failure or unavailability of the detection system.
  // ==========================================================================

  if (!is_detection_system_available()) {

    return make_emergency_stop(
      "PRIORITY 1: detection system failure or unavailable input"
    );
  }

  // ==========================================================================
  // CURRENT DATA
  // ==========================================================================

  const auto & operator_state =
    *latest_operator_state_;

  const auto & interaction =
    *latest_interaction_;

  const bool operator_present =
    operator_state.operator_present;

  const bool interaction_valid =
    interaction.interaction_valid;

  const bool distance_valid =
    interaction_valid &&
    std::isfinite(interaction.distance) &&
    interaction.distance >= 0.0;

  // ==========================================================================
  // PRIORITY 1 - INVALID INTERACTION DATA
  // ==========================================================================

  if (!interaction_valid || !distance_valid) {

    return make_emergency_stop(
      "PRIORITY 1: interaction/distance data unavailable"
    );
  }

  const double distance =
    interaction.distance;

  const Zone zone =
    determine_zone(distance);

  if (zone == Zone::UNKNOWN) {

    return make_emergency_stop(
      "PRIORITY 1: operator-robot zone cannot be determined"
    );
  }

  // ==========================================================================
  // PRIORITY 2 - Z3
  //
  // Operator detected in Z3 -> safe stop.
  // ==========================================================================

  if (
    zone == Zone::Z3 &&
    operator_present
  ) {

    return make_safe_stop(
      "PRIORITY 2: operator detected in Z3"
    );
  }

  // ==========================================================================
  // PRIORITY 2 - CRITICAL COLLISION RISK IN Z4/Z5
  // ==========================================================================

  if (
    (zone == Zone::Z4 || zone == Zone::Z5) &&
    collision_is_critical()
  ) {

    return make_safe_stop(
      "PRIORITY 2: critical collision risk in Z4 or Z5"
    );
  }

  // ==========================================================================
  // PRIORITY 3 - OPERATOR ANOMALY
  //
  // High fatigue or an attention condition configured as anomalous.
  // ==========================================================================

  if (
    operator_present &&
    operator_condition_anomalous()
  ) {

    return make_pause(
      "PRIORITY 3: elevated fatigue or anomalous operator condition"
    );
  }

  // ==========================================================================
  // PRIORITY 3 - TARGET NOT REACHABLE
  //
  // A trajectory with no points is considered unavailable/unreachable for the
  // current target.
  // ==========================================================================

  const bool target_reachable =
    trajectory_target_reachable();

  if (
    (zone == Zone::Z4 || zone == Zone::Z5 || zone == Zone::Z6) &&
    !target_reachable
  ) {

    if (zone == Zone::Z4 || zone == Zone::Z5) {

      return make_pause(
        "PRIORITY 3: target point not reachable under current conditions"
      );
    }

    // In Z6, operator loss has a specific priority-3 condition below.
    if (!operator_present) {

      return make_pause(
        "PRIORITY 3: operator unavailable in Z6"
      );
    }

    return make_pause(
      "PRIORITY 3: trajectory/target unavailable"
    );
  }

  // ==========================================================================
  // PRIORITY 3 - Z6 + OPERATOR NOT DETECTED
  // ==========================================================================

  if (
    zone == Zone::Z6 &&
    !operator_present
  ) {

    return make_pause(
      "PRIORITY 3: operator not available in Z6"
    );
  }

  // ==========================================================================
  // PRIORITY 4 - TRAJECTORY CANNOT MAINTAIN SEPARATION
  //
  // The current implementation uses a conservative current-state test. The
  // trajectory-wide geometric evaluation can replace this later.
  // ==========================================================================

  if (
    operator_present &&
    (zone == Zone::Z4 || zone == Zone::Z5 || zone == Zone::Z6) &&
    target_reachable
  ) {

    const bool maintains_separation =
      trajectory_keeps_separation(
        zone,
        distance
      );

    if (
      (zone == Zone::Z4 || zone == Zone::Z5) &&
      !maintains_separation
    ) {

      const int speed =
        compute_speed_percent(distance);

      return make_increase_distance(
        speed,
        "PRIORITY 4: trajectory cannot be executed while maintaining "
        "the required separation"
      );
    }
  }

  // ==========================================================================
  // PRIORITY 5 - Z4/Z5 SAFE TRAJECTORY
  //
  // Reduce speed according to the distance profile.
  // ==========================================================================

  if (
    (zone == Zone::Z4 || zone == Zone::Z5) &&
    operator_present &&
    target_reachable
  ) {

    const int speed =
      compute_speed_percent(distance);

    return make_limit_speed(
      speed,
      "PRIORITY 5: Z4/Z5 trajectory can execute with separation; "
      "speed reduction required"
    );
  }

  // ==========================================================================
  // PRIORITY 5 - Z5 / NOT REACHABLE / COLLABORATIVE CONDITION
  //
  // The matrix contains a specific Z5 row for a non-reachable condition.
  // Without a separate reachability reason in the current trajectory message,
  // this case is represented by the conservative speed-limiting command.
  // ==========================================================================

  if (
    (zone == Zone::Z4 || zone == Zone::Z5) &&
    !target_reachable
  ) {

    const int speed =
      compute_speed_percent(distance);

    return make_limit_speed(
      speed,
      "PRIORITY 5: Z4/Z5 collaborative condition with unavailable target"
    );
  }

  // ==========================================================================
  // PRIORITY 6 - Z6, OPERATOR PRESENT, NO ANOMALY
  // ==========================================================================

  if (
    zone == Zone::Z6 &&
    operator_present &&
    target_reachable
  ) {

    const int speed =
      compute_speed_percent(distance);

    return make_normal_operation(
      speed,
      "PRIORITY 6: Z6 operation without anomalous condition"
    );
  }

  // ==========================================================================
  // FAIL-SAFE FALLBACK
  // ==========================================================================

  return make_pause(
    "Fail-safe fallback: no valid decision rule matched"
  );
}


// ============================================================================
// PUBLICATION
// ============================================================================

void ControlDecision::publish_command(
  const DecisionResult & result)
{
  colab_supervision_interfaces::msg::ControlCommand msg;

  msg.command =
    result.command;

  msg.object =
    result.object;

  msg.variable =
    result.variable;

  msg.value =
    result.value;

  // --------------------------------------------------------------------------
  // Avoid unnecessary duplicate logs, but continue publishing the command.
  // --------------------------------------------------------------------------

  std::ostringstream signature_stream;

  signature_stream
    << result.command << "|"
    << result.object << "|"
    << result.variable << "|"
    << result.value;

  const std::string signature =
    signature_stream.str();

  if (
    signature !=
    last_published_signature_
  ) {

    RCLCPP_INFO(
      get_logger(),

      "COMMAND: %s | "
      "object=%s | "
      "variable=%s | "
      "value=%s | "
      "reason=%s",

      result.command.c_str(),
      result.object.c_str(),
      result.variable.c_str(),
      result.value.c_str(),
      result.reason.c_str()
    );

    last_published_signature_ =
      signature;
  }

  command_pub_->publish(
    msg
  );
}

}  // namespace colab_supervision_control


// ============================================================================
// MAIN
// ============================================================================

int main(
  int argc,
  char * argv[])
{
  rclcpp::init(
    argc,
    argv
  );

  try {

    auto node =
      std::make_shared<
        colab_supervision_control::ControlDecision
      >();

    rclcpp::spin(
      node
    );

  } catch (
    const std::exception & error) {

    RCLCPP_FATAL(
      rclcpp::get_logger(
        "control_decision"
      ),
      "Node initialization failed: %s",
      error.what()
    );
  }

  rclcpp::shutdown();

  return 0;
}
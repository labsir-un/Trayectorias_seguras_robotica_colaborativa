#include "colab_supervision_control/control_decision.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace colab_supervision_control
{

// ============================================================================
// CONSTRUCTOR
// ============================================================================

ControlDecision::ControlDecision()
: Node("control_decision")
{
  // --------------------------------------------------------------------------
  // INPUT TOPICS
  // --------------------------------------------------------------------------

  operator_state_topic_ = declare_parameter<std::string>(
    "operator_state_topic",
    "/operator/state_analisis");

  supervision_topic_ = declare_parameter<std::string>(
    "supervision_topic",
    "/control/robot_operator_state");

  trajectory_topic_ = declare_parameter<std::string>(
    "trajectory_topic",
    "/control/trajectory");

  command_topic_ = declare_parameter<std::string>(
    "command_topic",
    "/control/command");

  // --------------------------------------------------------------------------
  // DECISION TIMING
  // --------------------------------------------------------------------------

  decision_rate_hz_ = declare_parameter<double>(
    "decision_rate_hz",
    10.0);

  max_input_age_sec_ = declare_parameter<double>(
    "max_input_age_sec",
    0.50);

  max_trajectory_age_sec_ = declare_parameter<double>(
    "max_trajectory_age_sec",
    2.0);

  // --------------------------------------------------------------------------
  // SAFETY ZONES
  // --------------------------------------------------------------------------

  zone3_max_distance_ = declare_parameter<double>(
    "zone3_max_distance",
    0.250);

  zone4_max_distance_ = declare_parameter<double>(
    "zone4_max_distance",
    0.400);

  zone5_max_distance_ = declare_parameter<double>(
    "zone5_max_distance",
    1.600);

  // --------------------------------------------------------------------------
  // SPEED PROFILE
  // --------------------------------------------------------------------------

  full_speed_distance_ = declare_parameter<double>(
    "full_speed_distance",
    1.900);

  half_speed_distance_ = declare_parameter<double>(
    "half_speed_distance",
    1.600);

  minimum_speed_distance_ = declare_parameter<double>(
    "minimum_speed_distance",
    0.400);

  full_speed_percent_ = declare_parameter<double>(
    "full_speed_percent",
    100.0);

  half_speed_percent_ = declare_parameter<double>(
    "half_speed_percent",
    50.0);

  minimum_speed_percent_ = declare_parameter<double>(
    "minimum_speed_percent",
    5.0);

  speed_change_threshold_percent_ = declare_parameter<double>(
    "speed_change_threshold_percent",
    3.0);

  // Medium fatigue applies an additional multiplier to the distance-based
  // speed profile. A value of 0.70 means the profile speed is reduced by 30%.
  // LOW fatigue keeps the profile at 100%, while HIGH fatigue is handled by
  // the pause decision logic.
  fatigue_medium_speed_factor_ = declare_parameter<double>(
    "fatigue_medium_speed_factor",
    0.70);

  // --------------------------------------------------------------------------
  // OPERATOR CONDITIONS
  // --------------------------------------------------------------------------

  pause_on_fatigue_high_ = declare_parameter<bool>(
    "pause_on_fatigue_high",
    true);

  pause_on_attention_other_ = declare_parameter<bool>(
    "pause_on_attention_other",
    true);

  pause_on_attention_unknown_ = declare_parameter<bool>(
    "pause_on_attention_unknown",
    true);

  // --------------------------------------------------------------------------
  // TRAJECTORY SAFETY
  // --------------------------------------------------------------------------

  conservative_trajectory_check_ = declare_parameter<bool>(
    "conservative_trajectory_check",
    true);

  trajectory_min_separation_distance_ = declare_parameter<double>(
    "trajectory_min_separation_distance",
    0.400);

  // --------------------------------------------------------------------------
  // COMMAND GRAMMAR
  //
  // These parameters define the commands already understood by the parser.
  // No generic commands such as STOP/PAUSE/EXECUTE are generated here.
  // --------------------------------------------------------------------------

  send_trajectory_interest_command_ = declare_parameter<bool>(
    "send_trajectory_interest_command",
    true);

  send_restart_before_motion_ = declare_parameter<bool>(
    "send_restart_before_motion",
    true);

  send_speed_command_before_motion_ = declare_parameter<bool>(
    "send_speed_command_before_motion",
    true);

  block_during_distance_correction_ = declare_parameter<bool>(
    "block_during_distance_correction",
    false);

  distance_move_type_ = declare_parameter<std::string>(
    "distance_move_type",
    "MoveLinear");

  // Empty until the user provides the definitive offset.
  // The node will NOT send a SetOffsets command while this is empty.
  set_offsets_value_ = declare_parameter<std::string>(
    "set_offsets_value",
    "");

  safe_posture_joint_point_ = declare_parameter<std::string>(
    "safe_posture_joint_point",
    "");

  safe_posture_linear_point_ = declare_parameter<std::string>(
    "safe_posture_linear_point",
    "");

  // --------------------------------------------------------------------------
  // VALIDATION
  // --------------------------------------------------------------------------

  if (decision_rate_hz_ <= 0.0) {
    throw std::invalid_argument("decision_rate_hz must be greater than zero");
  }

  if (max_input_age_sec_ <= 0.0) {
    throw std::invalid_argument("max_input_age_sec must be greater than zero");
  }

  if (max_trajectory_age_sec_ <= 0.0) {
    throw std::invalid_argument("max_trajectory_age_sec must be greater than zero");
  }

  if (zone3_max_distance_ < 0.0) {
    throw std::invalid_argument("zone3_max_distance must be non-negative");
  }

  if (zone4_max_distance_ <= zone3_max_distance_) {
    throw std::invalid_argument(
      "zone4_max_distance must be greater than zone3_max_distance");
  }

  if (zone5_max_distance_ < zone4_max_distance_) {
    throw std::invalid_argument(
      "zone5_max_distance must be greater than or equal to zone4_max_distance");
  }

  if (full_speed_distance_ <= half_speed_distance_) {
    throw std::invalid_argument(
      "full_speed_distance must be greater than half_speed_distance");
  }

  if (half_speed_distance_ <= minimum_speed_distance_) {
    throw std::invalid_argument(
      "half_speed_distance must be greater than minimum_speed_distance");
  }

  if (
    full_speed_percent_ < 0.0 ||
    half_speed_percent_ < 0.0 ||
    minimum_speed_percent_ < 0.0)
  {
    throw std::invalid_argument("Speed percentages must be non-negative");
  }

  if (speed_change_threshold_percent_ < 0.0) {
    throw std::invalid_argument(
      "speed_change_threshold_percent must be non-negative");
  }

  if (fatigue_medium_speed_factor_ < 0.0 ||
      fatigue_medium_speed_factor_ > 1.0) {
    throw std::invalid_argument(
      "fatigue_medium_speed_factor must be between 0.0 and 1.0");
  }

  if (trajectory_min_separation_distance_ < 0.0) {
    throw std::invalid_argument(
      "trajectory_min_separation_distance must be non-negative");
  }

  if (
    distance_move_type_ != "MoveJoint" &&
    distance_move_type_ != "MoveLinear")
  {
    throw std::invalid_argument(
      "distance_move_type must be MoveJoint or MoveLinear");
  }

  // --------------------------------------------------------------------------
  // SUBSCRIBERS
  // --------------------------------------------------------------------------

  operator_state_sub_ =
    create_subscription<colab_supervision_interfaces::msg::OperatorStateAnalysis>(
      operator_state_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
        &ControlDecision::operator_state_callback,
        this,
        std::placeholders::_1));

  supervision_sub_ =
    create_subscription<colab_supervision_interfaces::msg::RobotOperatorSupervision>(
      supervision_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
        &ControlDecision::supervision_callback,
        this,
        std::placeholders::_1));

  trajectory_sub_ =
    create_subscription<moveit_msgs::msg::RobotTrajectory>(
      trajectory_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
        &ControlDecision::trajectory_callback,
        this,
        std::placeholders::_1));

  // --------------------------------------------------------------------------
  // PUBLISHER
  // --------------------------------------------------------------------------

  command_pub_ =
    create_publisher<colab_supervision_interfaces::msg::ControlCommand>(
      command_topic_,
      10);

  // --------------------------------------------------------------------------
  // DECISION TIMER
  // --------------------------------------------------------------------------

  const auto period =
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / decision_rate_hz_));

  decision_timer_ =
    create_wall_timer(
      period,
      std::bind(
        &ControlDecision::decision_timer_callback,
        this));

  RCLCPP_INFO(get_logger(), "====================================================");
  RCLCPP_INFO(get_logger(), "CONTROL DECISION NODE");
  RCLCPP_INFO(get_logger(), "Operator state: %s", operator_state_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Supervision state: %s", supervision_topic_.c_str());
  RCLCPP_INFO(get_logger(), "MoveIt trajectory: %s", trajectory_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Control command: %s", command_topic_.c_str());
  RCLCPP_INFO(
    get_logger(),
    "Zones: Z3 < %.3f | Z4 < %.3f | Z5 <= %.3f | Z6 > %.3f m",
    zone3_max_distance_, zone4_max_distance_, zone5_max_distance_, zone5_max_distance_);
  RCLCPP_INFO(
    get_logger(),
    "Command grammar: Variable / Velocidad / Distancia");
  RCLCPP_INFO(
    get_logger(),
    "Fatigue speed profile: LOW=100%% | MEDIUM=%.0f%% | HIGH=PAUSE",
    fatigue_medium_speed_factor_ * 100.0);
  RCLCPP_INFO(
    get_logger(),
    "Commander is planning only; this node decides and publishes the command");

  if (set_offsets_value_.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "set_offsets_value is empty: distance correction will fail-safe to PAUSE "
      "until a valid [x,y,z] value is configured");
  }

  RCLCPP_INFO(get_logger(), "====================================================");
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

void ControlDecision::supervision_callback(
  const colab_supervision_interfaces::msg::RobotOperatorSupervision::SharedPtr msg)
{
  latest_supervision_ = *msg;
  supervision_received_time_ = get_clock()->now();
}

void ControlDecision::trajectory_callback(
  const moveit_msgs::msg::RobotTrajectory::SharedPtr msg)
{
  latest_trajectory_ = *msg;
  trajectory_received_time_ = get_clock()->now();

  // Build a lightweight signature from trajectory content. This is used to
  // distinguish a new MoveIt plan from the same plan being retained.
  std::ostringstream ss;
  ss << latest_trajectory_->joint_trajectory.joint_names.size() << "|"
     << latest_trajectory_->joint_trajectory.points.size();

  if (!latest_trajectory_->joint_trajectory.points.empty()) {
    const auto & p = latest_trajectory_->joint_trajectory.points.back();
    ss << "|" << p.positions.size();
    for (const auto value : p.positions) {
      ss << "|" << std::setprecision(9) << value;
    }
  }

  const std::string new_signature = ss.str();

  // A repeated copy of the same trajectory is not considered a new plan.
  // This prevents the control decision node from repeatedly generating the
  // Interes command when the exact same MoveIt trajectory is retained.
  if (new_signature != last_trajectory_signature_) {
    last_trajectory_signature_ = new_signature;
    ++trajectory_generation_;
  }
}

// ============================================================================
// TIMER / PLAN EXECUTION
// ============================================================================

void ControlDecision::decision_timer_callback()
{
  const DecisionPlan plan = evaluate_decision();

  // A different decision state starts a new one-shot command sequence.
  if (!plan_is_active() || !same_plan(plan)) {
    apply_plan(plan);
    return;
  }

  // Every command step is a pulse. Once published it is never repeated while
  // the same decision remains active. Multi-step actions advance one command
  // at a time on subsequent decision cycles.
  if (active_plan_.steps.empty()) {
    return;
  }

  if (active_step_index_ + 1 < active_plan_.steps.size()) {
    ++active_step_index_;
    publish_current_step();
  }
}

// ============================================================================
// INPUT FRESHNESS
// ============================================================================

bool ControlDecision::is_operator_state_fresh() const
{
  if (!latest_operator_state_.has_value()) {
    return false;
  }

  return (get_clock()->now() - operator_state_received_time_).seconds() <=
         max_input_age_sec_;
}

bool ControlDecision::is_supervision_fresh() const
{
  if (!latest_supervision_.has_value()) {
    return false;
  }

  return (get_clock()->now() - supervision_received_time_).seconds() <=
         max_input_age_sec_;
}

bool ControlDecision::is_trajectory_fresh() const
{
  if (!latest_trajectory_.has_value()) {
    return false;
  }

  return (get_clock()->now() - trajectory_received_time_).seconds() <=
         max_trajectory_age_sec_;
}

bool ControlDecision::processed_information_available() const
{
  return is_operator_state_fresh() && is_supervision_fresh();
}

// ============================================================================
// OPERATOR / COLLISION CONDITIONS
// ============================================================================

bool ControlDecision::operator_condition_anomalous() const
{
  if (!latest_operator_state_.has_value()) {
    return true;
  }

  const auto & state = *latest_operator_state_;

  // MEDIUM fatigue is a valid operating condition and does NOT generate a
  // pause. Only HIGH fatigue is considered anomalous here.
  if (pause_on_fatigue_high_ && state.fatigue_condition == "HIGH") {
    return true;
  }

  if (pause_on_attention_other_ && state.attention_condition == "OTHER") {
    return true;
  }

  if (pause_on_attention_unknown_ && state.attention_condition == "UNKNOWN") {
    return true;
  }

  return false;
}

bool ControlDecision::operator_is_available() const
{
  if (!latest_operator_state_.has_value() || !latest_supervision_.has_value()) {
    return false;
  }

  // Operator availability is accepted when at least one perception source
  // has a valid operator observation. In particular, a valid operator state
  // from /operator/state_analisis is sufficient even when the interaction
  // camera temporarily does not report the operator.
  return (
    latest_operator_state_->operator_present ||
    latest_supervision_->operator_detected);
}

bool ControlDecision::collision_is_critical() const
{
  if (!latest_supervision_.has_value()) {
    return true;
  }

  const auto & supervision = *latest_supervision_;

  return (
    supervision.collision_imminent ||
    supervision.collision_risk == "CRITICAL" ||
    supervision.collision_risk == "DANGEROUS");
}

// ============================================================================
// ZONES
// ============================================================================

ControlDecision::Zone ControlDecision::determine_zone(double distance) const
{
  if (!std::isfinite(distance) || distance < 0.0) {
    return Zone::UNKNOWN;
  }

  if (distance < zone3_max_distance_) {
    return Zone::Z3;
  }

  if (distance < zone4_max_distance_) {
    return Zone::Z4;
  }

  if (distance <= zone5_max_distance_) {
    return Zone::Z5;
  }

  return Zone::Z6;
}

const char * ControlDecision::zone_to_string(Zone zone) const
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

const char * ControlDecision::action_to_string(Action action) const
{
  switch (action) {
    case Action::EMERGENCY_STOP:
      return "EMERGENCY_STOP";
    case Action::SAFE_STOP:
      return "SAFE_STOP";
    case Action::PAUSE:
      return "PAUSE";
    case Action::INCREASE_DISTANCE:
      return "INCREASE_DISTANCE";
    case Action::LIMIT_SPEED:
      return "LIMIT_SPEED";
    case Action::NORMAL_OPERATION:
      return "NORMAL_OPERATION";
    default:
      return "UNKNOWN";
  }
}

// ============================================================================
// TRAJECTORY VALIDATION
// ============================================================================

bool ControlDecision::trajectory_available() const
{
  return (
    latest_trajectory_.has_value() &&
    is_trajectory_fresh() &&
    !latest_trajectory_->joint_trajectory.points.empty());
}

bool ControlDecision::trajectory_points_valid() const
{
  if (!trajectory_available()) {
    return false;
  }

  const auto & trajectory = latest_trajectory_->joint_trajectory;

  if (trajectory.joint_names.empty()) {
    return false;
  }

  for (const auto & point : trajectory.points) {
    if (point.positions.size() != trajectory.joint_names.size()) {
      return false;
    }

    for (const auto position : point.positions) {
      if (!std::isfinite(position)) {
        return false;
      }
    }
  }

  return true;
}

bool ControlDecision::trajectory_target_reachable() const
{
  // MoveIt only publishes a trajectory after a successful plan. Therefore,
  // at this interface level, a fresh trajectory with valid joint points is
  // the indication that the requested target is reachable by the planner.
  return trajectory_points_valid();
}

bool ControlDecision::trajectory_keeps_separation(
  Zone zone,
  double distance) const
{
  if (zone == Zone::Z6) {
    return true;
  }

  if (!conservative_trajectory_check_) {
    return true;
  }

  if (distance < trajectory_min_separation_distance_) {
    return false;
  }

  if (!latest_supervision_.has_value()) {
    return false;
  }

  const auto & supervision = *latest_supervision_;

  if (!supervision.interaction_valid || !supervision.distance_valid) {
    return false;
  }

  if (
    supervision.collision_imminent ||
    supervision.collision_risk == "CRITICAL" ||
    supervision.collision_risk == "DANGEROUS")
  {
    return false;
  }

  // IMPORTANT:
  // The current RobotOperatorSupervision message contains the current scalar
  // distance, not the 3-D operator/robot trajectory geometry. Therefore this
  // is a conservative proxy based on the supervised current state, not a FK
  // check of every MoveIt trajectory point.
  return true;
}

// ============================================================================
// SPEED PROFILE
// ============================================================================

int ControlDecision::compute_speed_percent(double distance) const
{
  double speed = minimum_speed_percent_;

  if (distance >= full_speed_distance_) {
    speed = full_speed_percent_;
  }
  else if (distance >= half_speed_distance_) {
    const double normalized =
      (full_speed_distance_ - distance) /
      (full_speed_distance_ - half_speed_distance_);

    const double ratio = half_speed_percent_ / full_speed_percent_;

    speed = full_speed_percent_ * std::pow(ratio, normalized);
  }
  else if (distance >= minimum_speed_distance_) {
    const double normalized =
      (half_speed_distance_ - distance) /
      (half_speed_distance_ - minimum_speed_distance_);

    const double ratio = minimum_speed_percent_ / half_speed_percent_;

    speed = half_speed_percent_ * std::pow(ratio, normalized);
  }

  speed = std::clamp(
    speed,
    minimum_speed_percent_,
    full_speed_percent_);

  return static_cast<int>(std::lround(speed));
}

double ControlDecision::fatigue_speed_factor() const
{
  if (!latest_operator_state_.has_value()) {
    return 1.0;
  }

  const auto & state = *latest_operator_state_;

  // LOW: use the distance profile without any additional reduction.
  if (state.fatigue_condition == "LOW") {
    return 1.0;
  }

  // MEDIUM: reduce the already calculated distance-based profile by 30%
  // through the configurable factor (default = 0.70).
  if (state.fatigue_condition == "MEDIUM") {
    return fatigue_medium_speed_factor_;
  }

  // HIGH is not converted to a speed here because it is handled as a pause
  // condition by operator_condition_anomalous().
  return 1.0;
}

int ControlDecision::compute_effective_speed_percent(double distance) const
{
  const int profile_speed = compute_speed_percent(distance);
  const double factor = fatigue_speed_factor();

  const double effective_speed =
    static_cast<double>(profile_speed) * factor;

  // The fatigue reduction is applied after the distance profile. Therefore
  // MEDIUM fatigue can reduce even the minimum 5% profile value. Do not clamp
  // back to minimum_speed_percent_ here, otherwise the requested additional
  // 30% reduction would be lost near the lower end of the profile.
  return static_cast<int>(std::lround(
    std::clamp(effective_speed, 0.0, full_speed_percent_)));
}

bool ControlDecision::speed_change_requires_command(
  int desired_speed_percent) const
{
  if (!last_commanded_speed_percent_.has_value()) {
    return true;
  }

  return std::abs(
           desired_speed_percent - *last_commanded_speed_percent_) >=
         speed_change_threshold_percent_;
}

std::string ControlDecision::speed_signature(int desired_speed_percent) const
{
  if (speed_change_requires_command(desired_speed_percent)) {
    return std::to_string(desired_speed_percent);
  }

  // Keep the plan signature stable while the requested speed moves inside the
  // deadband. This avoids rebuilding the plan and sending other commands for
  // every small fluctuation in distance.
  return std::to_string(*last_commanded_speed_percent_);
}

// ============================================================================
// EXACT COMMAND BUILDERS
// ============================================================================

ControlDecision::DecisionResult ControlDecision::make_variable_set(
  const std::string & variable,
  const std::string & value,
  const std::string & reason) const
{
  return {"Variable", "SET", variable, value, reason};
}

ControlDecision::DecisionResult ControlDecision::make_speed_percent(
  int speed_percent,
  const std::string & reason) const
{
  return {
    "Velocidad",
    "Porcentaje",
    "VelMax",
    std::to_string(speed_percent),
    reason};
}

ControlDecision::DecisionResult ControlDecision::make_speed_max(
  int speed_percent,
  const std::string & reason) const
{
  return {
    "Velocidad",
    "Max",
    "VelMax",
    std::to_string(speed_percent),
    reason};
}

ControlDecision::DecisionResult ControlDecision::make_get_tcp_euler(
  const std::string & reason) const
{
  return {
    "Distancia",
    "GetPostura",
    "Prototipo",
    "TCPEuler",
    reason};
}

ControlDecision::DecisionResult ControlDecision::make_get_tcp_quaternion(
  const std::string & reason) const
{
  return {
    "Distancia",
    "GetPostura",
    "Prototipo",
    "TCPTCPCua",
    reason};
}

ControlDecision::DecisionResult ControlDecision::make_update_interest(
  const std::string & reason) const
{
  return {
    "Distancia",
    "CambiarTrayectoria",
    "ActualizarPuntos",
    "Interes",
    reason};
}

ControlDecision::DecisionResult ControlDecision::make_set_offsets(
  const std::string & reason) const
{
  return {
    "Distancia",
    "CambiarTrayectoria",
    "SetOffsets",
    set_offsets_value_,
    reason};
}

ControlDecision::DecisionResult ControlDecision::make_save_exit_point(
  const std::string & reason) const
{
  return {
    "Distancia",
    "CambiarTrayectoria",
    "ActualizarPuntos",
    "GuardarPSalida",
    reason};
}

ControlDecision::DecisionResult ControlDecision::make_move_temp(
  const std::string & move_type,
  const std::string & reason) const
{
  return {
    "Distancia",
    "CambiarTrayectoria",
    "MoveTemp",
    move_type,
    reason};
}

ControlDecision::DecisionResult ControlDecision::make_safe_posture_joint(
  const std::string & point_name,
  const std::string & reason) const
{
  return {
    "Distancia",
    "Postura_segura",
    "robtargetJ",
    point_name,
    reason};
}

ControlDecision::DecisionResult ControlDecision::make_safe_posture_linear(
  const std::string & point_name,
  const std::string & reason) const
{
  return {
    "Distancia",
    "Postura_segura",
    "robtargetL",
    point_name,
    reason};
}

ControlDecision::DecisionResult ControlDecision::make_joint_configuration(
  const std::vector<double> & joints,
  const std::string & reason) const
{
  return {
    "Distancia",
    "Postura_segura",
    "JoinConf",
    format_joint_configuration(joints),
    reason};
}

// ============================================================================
// PLAN BUILDERS
// ============================================================================

ControlDecision::DecisionPlan ControlDecision::make_emergency_stop_plan(
  const std::string & reason) const
{
  DecisionPlan plan;
  plan.action = Action::EMERGENCY_STOP;
  plan.signature = "EMERGENCY_STOP";
  plan.reason = reason;
  plan.steps.push_back({
      make_variable_set("Sock_Parada", "TRUE", reason)});
  return plan;
}

ControlDecision::DecisionPlan ControlDecision::make_safe_stop_plan(
  const std::string & reason) const
{
  DecisionPlan plan;
  plan.action = Action::SAFE_STOP;
  plan.signature = "SAFE_STOP";
  plan.reason = reason;

  // Detención segura is represented by pausing the current motion and then
  // blocking new actions. Both commands are part of the user's command grammar.
  plan.steps.push_back({
      make_variable_set("Sock_Pausa", "TRUE", reason)});

  plan.steps.push_back({
      make_variable_set("Sock_Bloquear", "TRUE", reason)});

  return plan;
}

ControlDecision::DecisionPlan ControlDecision::make_pause_plan(
  const std::string & reason) const
{
  DecisionPlan plan;
  plan.action = Action::PAUSE;
  plan.signature = "PAUSE";
  plan.reason = reason;
  plan.steps.push_back({
      make_variable_set("Sock_Pausa", "TRUE", reason)});
  return plan;
}

ControlDecision::DecisionPlan ControlDecision::make_increase_distance_plan(
  int speed_percent,
  const std::string & reason) const
{
  DecisionPlan plan;
  plan.action = Action::INCREASE_DISTANCE;
  plan.signature =
    "INCREASE_DISTANCE|speed=" + speed_signature(speed_percent) +
    "|move=" + distance_move_type_ +
    "|offset=" + set_offsets_value_ +
    "|traj=" + std::to_string(trajectory_generation_);
  plan.reason = reason;

  // No valid offset -> do not send a zero/undefined offset to the robot.
  if (set_offsets_value_.empty()) {
    return make_pause_plan(
      "PRIORITY 3: distance correction requested but SetOffsets is not configured");
  }

  if (block_during_distance_correction_) {
    plan.steps.push_back({
        make_variable_set(
          "Sock_Bloquear",
          "TRUE",
          "Block new actions before distance correction")});
  }

  if (
    send_speed_command_before_motion_ &&
    speed_change_requires_command(speed_percent))
  {
    plan.steps.push_back({
        make_speed_percent(
          speed_percent,
          "PRIORITY 4: reduce VelMax before increasing separation")});
  }

  plan.steps.push_back({
      make_save_exit_point(
        "PRIORITY 4: save trajectory interruption point before correction")});

  plan.steps.push_back({
      make_set_offsets(
        "PRIORITY 4: apply configured [x,y,z] offsets")});

  plan.steps.push_back({
      make_move_temp(
        distance_move_type_,
        "PRIORITY 4: move to temporary point using configured offset")});

  if (send_trajectory_interest_command_) {
    plan.steps.push_back({
        make_update_interest(
          "PRIORITY 4: update current origin/target points after distance correction")});
  }

  return plan;
}

ControlDecision::DecisionPlan ControlDecision::make_limit_speed_plan(
  int speed_percent,
  const std::string & reason) const
{
  DecisionPlan plan;
  plan.action = Action::LIMIT_SPEED;
  plan.signature =
    "LIMIT_SPEED|speed=" + speed_signature(speed_percent) +
    "|traj=" + std::to_string(trajectory_generation_);
  plan.reason = reason;

  // Sock_Reinicio is sent only after a previous pause/block/stop has actually
  // been issued. It is never sent continuously during normal operation.
  if (send_restart_before_motion_ && restart_required_) {
    plan.steps.push_back({
        make_variable_set(
          "Sock_Reinicio",
          "TRUE",
          "Release previous pause/block/stop before controlled motion")});
  }

  if (
    send_speed_command_before_motion_ &&
    speed_change_requires_command(speed_percent))
  {
    plan.steps.push_back({
        make_speed_percent(
          speed_percent,
          reason)});
  }

  if (send_trajectory_interest_command_) {
    plan.steps.push_back({
        make_update_interest(
          "PRIORITY 5: execute/update the planned trajectory at reduced speed")});
  }

  return plan;
}

ControlDecision::DecisionPlan ControlDecision::make_normal_operation_plan(
  int speed_percent,
  const std::string & reason) const
{
  DecisionPlan plan;
  plan.action = Action::NORMAL_OPERATION;
  plan.signature =
    "NORMAL_OPERATION|speed=" + speed_signature(speed_percent) +
    "|traj=" + std::to_string(trajectory_generation_);
  plan.reason = reason;

  if (send_restart_before_motion_ && restart_required_) {
    plan.steps.push_back({
        make_variable_set(
          "Sock_Reinicio",
          "TRUE",
          "Release previous pause/block/stop before normal operation")});
  }

  if (
    send_speed_command_before_motion_ &&
    speed_change_requires_command(speed_percent))
  {
    plan.steps.push_back({
        make_speed_percent(
          speed_percent,
          "PRIORITY 6: set VelMax for normal operation")});
  }

  if (send_trajectory_interest_command_) {
    plan.steps.push_back({
        make_update_interest(
          "PRIORITY 6: command execution/update of the planned trajectory")});
  }

  return plan;
}

ControlDecision::DecisionPlan ControlDecision::make_fail_safe_plan(
  const std::string & reason) const
{
  return make_emergency_stop_plan(reason);
}

// ============================================================================
// MAIN DECISION HIERARCHY
// ============================================================================

ControlDecision::DecisionPlan ControlDecision::evaluate_decision()
{
  // --------------------------------------------------------------------------
  // PRIORITY 1 - loss of processed information
  // --------------------------------------------------------------------------

  if (!processed_information_available()) {
    return make_fail_safe_plan(
      "PRIORITY 1: preprocessed operator or interaction information unavailable");
  }

  const auto & operator_state = *latest_operator_state_;
  const auto & supervision = *latest_supervision_;

  // --------------------------------------------------------------------------
  // PRIORITY 1 - invalid supervision/distance
  // --------------------------------------------------------------------------

  if (!supervision.interaction_valid || !supervision.distance_valid) {
    return make_fail_safe_plan(
      "PRIORITY 1: supervised interaction or distance is invalid");
  }

  if (!std::isfinite(supervision.distance) || supervision.distance < 0.0F) {
    return make_fail_safe_plan(
      "PRIORITY 1: supervised distance is not valid");
  }

  const double distance = static_cast<double>(supervision.distance);
  const Zone zone = determine_zone(distance);

  if (zone == Zone::UNKNOWN) {
    return make_fail_safe_plan(
      "PRIORITY 1: operating zone cannot be determined");
  }

  // --------------------------------------------------------------------------
  // PRIORITY 2 - Z3 + operator detected -> SAFE STOP
  // --------------------------------------------------------------------------

  if (zone == Zone::Z3 && operator_is_available()) {
    return make_safe_stop_plan(
      "PRIORITY 2: operator detected in Z3 by at least one perception source");
  }

  // --------------------------------------------------------------------------
  // PRIORITY 2 - Z4/Z5 + critical/dangerous collision risk -> SAFE STOP
  // --------------------------------------------------------------------------

  if (
    (zone == Zone::Z4 || zone == Zone::Z5) &&
    collision_is_critical())
  {
    return make_safe_stop_plan(
      "PRIORITY 2: critical/dangerous collision condition in Z4/Z5");
  }

  // --------------------------------------------------------------------------
  // PRIORITY 3 - operator unavailable from ALL perception sources
  // --------------------------------------------------------------------------

  if (!operator_is_available()) {
    return make_pause_plan(
      "PRIORITY 3: operator not available in operator analysis or interaction perception");
  }

  // --------------------------------------------------------------------------
  // PRIORITY 3 - fatigue/attention anomaly -> CONTROLLED PAUSE
  // --------------------------------------------------------------------------

  if (operator_condition_anomalous()) {
    return make_pause_plan(
      "PRIORITY 3: elevated fatigue or anomalous operator attention condition");
  }

  // --------------------------------------------------------------------------
  // PRIORITY 3 - no valid MoveIt trajectory -> CONTROLLED PAUSE
  // --------------------------------------------------------------------------

  if (!trajectory_target_reachable()) {
    return make_pause_plan(
      "PRIORITY 3: valid MoveIt trajectory unavailable or invalid");
  }

  // --------------------------------------------------------------------------
  // PRIORITY 4 - Z4/Z5 + trajectory cannot maintain separation
  // --------------------------------------------------------------------------

  if (zone == Zone::Z4 || zone == Zone::Z5) {
    if (!trajectory_keeps_separation(zone, distance)) {
      const int speed = compute_effective_speed_percent(distance);
      return make_increase_distance_plan(
        speed,
        "PRIORITY 4: increase separation and apply distance profile with fatigue modifier");
    }
  }

  // --------------------------------------------------------------------------
  // PRIORITY 5 - Z4/Z5 + trajectory safe -> reduced speed
  // --------------------------------------------------------------------------

  if (zone == Zone::Z4 || zone == Zone::Z5) {
    const int speed = compute_effective_speed_percent(distance);
    return make_limit_speed_plan(
      speed,
      "PRIORITY 5: Z4/Z5 trajectory is executable; apply distance profile and fatigue modifier");
  }

  // --------------------------------------------------------------------------
  // PRIORITY 6 - Z6 + no anomaly -> normal operation
  // --------------------------------------------------------------------------

  if (zone == Zone::Z6) {
    const int speed = compute_effective_speed_percent(distance);
    return make_normal_operation_plan(
      speed,
      "PRIORITY 6: Z6 operation; apply distance profile and fatigue modifier");
  }

  return make_pause_plan(
    "Fail-safe fallback: no valid decision rule matched");
}

// ============================================================================
// PLAN EXECUTION
// ============================================================================

bool ControlDecision::plan_is_active() const
{
  return plan_initialized_;
}

bool ControlDecision::same_plan(const DecisionPlan & plan) const
{
  return (
    plan.signature == active_plan_.signature &&
    plan.action == active_plan_.action);
}

void ControlDecision::apply_plan(const DecisionPlan & plan)
{
  active_plan_ = plan;
  active_step_index_ = 0;
  plan_initialized_ = true;

  RCLCPP_INFO(
    get_logger(),
    "DECISION: action=%s | reason=%s | steps=%zu",
    action_to_string(active_plan_.action),
    active_plan_.reason.c_str(),
    active_plan_.steps.size());

  if (active_plan_.steps.empty()) {
    return;
  }

  publish_current_step();
}

void ControlDecision::publish_current_step()
{
  if (active_plan_.steps.empty() ||
      active_step_index_ >= active_plan_.steps.size())
  {
    return;
  }

  const auto & result = active_plan_.steps[active_step_index_].command;

  colab_supervision_interfaces::msg::ControlCommand msg;
  msg.command = result.command;
  msg.object = result.object;
  msg.variable = result.variable;
  msg.value = result.value;

  RCLCPP_INFO(
    get_logger(),
    "COMMAND [ONE-SHOT]: %s | object=%s | variable=%s | value=%s | reason=%s",
    result.command.c_str(),
    result.object.c_str(),
    result.variable.c_str(),
    result.value.c_str(),
    result.reason.c_str());

  command_pub_->publish(msg);

  // State changes are recorded only when the corresponding pulse is actually
  // sent to the parser.
  if (
    result.command == "Variable" &&
    result.object == "SET" &&
    result.value == "TRUE")
  {
    if (
      result.variable == "Sock_Pausa" ||
      result.variable == "Sock_Bloquear" ||
      result.variable == "Sock_Parada")
    {
      restart_required_ = true;
    }
    else if (result.variable == "Sock_Reinicio") {
      restart_required_ = false;
    }
  }

  if (
    result.command == "Velocidad" &&
    result.variable == "VelMax" &&
    result.object == "Porcentaje")
  {
    try {
      last_commanded_speed_percent_ = std::stoi(result.value);
    }
    catch (const std::exception &) {
      RCLCPP_WARN(
        get_logger(),
        "Unable to store commanded speed '%s' as an integer",
        result.value.c_str());
    }
  }
}

// FORMAT HELPERS
// ============================================================================

std::string ControlDecision::format_array3(
  double x,
  double y,
  double z)
{
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(6)
     << "[" << x << "," << y << "," << z << "]";
  return ss.str();
}

std::string ControlDecision::format_joint_configuration(
  const std::vector<double> & joints)
{
  std::ostringstream ss;
  ss << "[";

  for (std::size_t i = 0; i < joints.size(); ++i) {
    if (i > 0) {
      ss << ",";
    }

    ss << std::fixed << std::setprecision(8) << joints[i];
  }

  ss << "],[9E9,9E9,9E9,9E9,9E9,9E9]";

  // The command grammar expects the complete value wrapped as:
  // [[articulaciones],[9E9,...]]
  return "[" + ss.str() + "]";
}

}  // namespace colab_supervision_control

// ============================================================================
// MAIN
// ============================================================================

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<colab_supervision_control::ControlDecision>();
    rclcpp::spin(node);
  }
  catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("control_decision"),
      "Node initialization failed: %s",
      error.what());
  }

  rclcpp::shutdown();
  return 0;
}

#include "colab_supervision_control/control_decision.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
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

  socket_ready_topic_ = declare_parameter<std::string>(
    "socket_ready_topic", "/SockRes/Ready");
  tcp_euler_topic_ = declare_parameter<std::string>(
    "tcp_euler_topic", "/SockRes/TCPeul");
  tcp_quaternion_topic_ = declare_parameter<std::string>(
    "tcp_quaternion_topic", "/SockRes/TCPcua");
  robot_joints_topic_ = declare_parameter<std::string>(
    "robot_joints_topic", "/SockRes/Joints");

  commander_euler_topic_ = declare_parameter<std::string>(
    "commander_euler_topic", "/pose_command");
  commander_quaternion_topic_ = declare_parameter<std::string>(
    "commander_quaternion_topic", "/posecua_command");
  commander_joint_topic_ = declare_parameter<std::string>(
    "commander_joint_topic", "/joint_command");

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
  max_robot_joints_age_sec_ = declare_parameter<double>(
    "max_robot_joints_age_sec", 5.0);
  max_tcp_state_age_sec_ = declare_parameter<double>(
    "max_tcp_state_age_sec", 5.0);

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
  joint_similarity_tolerance_percent_ = declare_parameter<double>(
    "joint_similarity_tolerance_percent", 5.0);
  sock_joints_in_degrees_ = declare_parameter<bool>(
    "sock_joints_in_degrees", false);

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

  commander_pose_mode_ = declare_parameter<std::string>(
    "commander_pose_mode", "quaternion");
  commander_cartesian_path_ = declare_parameter<bool>(
    "commander_cartesian_path", false);
  tcp_position_scale_to_commander_ = declare_parameter<double>(
    "tcp_position_scale_to_commander", 0.001);
  tcp_euler_angles_in_degrees_ = declare_parameter<bool>(
    "tcp_euler_angles_in_degrees", true);
  command_queue_capacity_ = declare_parameter<int>(
    "command_queue_capacity", 20);
  command_queue_ttl_sec_ = declare_parameter<double>(
    "command_queue_ttl_sec", 60.0);

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
  if (max_robot_joints_age_sec_ <= 0.0 || max_tcp_state_age_sec_ <= 0.0) {
    throw std::invalid_argument("Robot joint and TCP state age limits must be positive");
  }
  if (joint_similarity_tolerance_percent_ < 0.0 ||
      joint_similarity_tolerance_percent_ > 100.0) {
    throw std::invalid_argument("joint_similarity_tolerance_percent must be in [0,100]");
  }
  if (command_queue_capacity_ < 1 || command_queue_capacity_ > 20) {
    throw std::invalid_argument("command_queue_capacity must be between 1 and 20");
  }
  if (command_queue_ttl_sec_ <= 0.0 || command_queue_ttl_sec_ > 60.0) {
    throw std::invalid_argument("command_queue_ttl_sec must be in (0, 60] seconds");
  }
  if (commander_pose_mode_ != "quaternion" && commander_pose_mode_ != "euler" &&
      commander_pose_mode_ != "joints") {
    throw std::invalid_argument("commander_pose_mode must be quaternion, euler, or joints");
  }
  if (!std::isfinite(tcp_position_scale_to_commander_) ||
      tcp_position_scale_to_commander_ <= 0.0) {
    throw std::invalid_argument("tcp_position_scale_to_commander must be positive and finite");
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

  socket_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
    socket_ready_topic_, 10,
    std::bind(&ControlDecision::socket_ready_callback, this, std::placeholders::_1));
  tcp_euler_sub_ = create_subscription<colab_supervision_interfaces::msg::PoseCommand>(
    tcp_euler_topic_, 10,
    std::bind(&ControlDecision::tcp_euler_callback, this, std::placeholders::_1));
  tcp_quaternion_sub_ = create_subscription<colab_supervision_interfaces::msg::PoseCuaCommand>(
    tcp_quaternion_topic_, 10,
    std::bind(&ControlDecision::tcp_quaternion_callback, this, std::placeholders::_1));
  robot_joints_sub_ = create_subscription<colab_supervision_interfaces::msg::Joints>(
    robot_joints_topic_, 10,
    std::bind(&ControlDecision::robot_joints_callback, this, std::placeholders::_1));

  // --------------------------------------------------------------------------
  // PUBLISHERS
  // --------------------------------------------------------------------------

  command_pub_ =
    create_publisher<colab_supervision_interfaces::msg::ControlCommand>(
      command_topic_, 10);
  commander_euler_pub_ = create_publisher<colab_supervision_interfaces::msg::PoseCommand>(
    commander_euler_topic_, 10);
  commander_quaternion_pub_ = create_publisher<colab_supervision_interfaces::msg::PoseCuaCommand>(
    commander_quaternion_topic_, 10);
  commander_joint_pub_ = create_publisher<colab_supervision_interfaces::msg::Joints>(
    commander_joint_topic_, 10);

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
  RCLCPP_INFO(get_logger(), "Control command output: %s", command_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Socket ready: %s", socket_ready_topic_.c_str());
  RCLCPP_INFO(get_logger(), "TCP Euler input: %s -> Commander %s",
    tcp_euler_topic_.c_str(), commander_euler_topic_.c_str());
  RCLCPP_INFO(get_logger(), "TCP quaternion input: %s -> Commander %s",
    tcp_quaternion_topic_.c_str(), commander_quaternion_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Robot joints: %s | queue=%d | TTL=%.1f s",
    robot_joints_topic_.c_str(), command_queue_capacity_, command_queue_ttl_sec_);
  RCLCPP_INFO(get_logger(), "Joint endpoint similarity tolerance: %.2f%%",
    joint_similarity_tolerance_percent_);
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
  const auto now = get_clock()->now();
  latest_trajectory_ = *msg;
  trajectory_received_time_ = now;

  latest_trajectory_is_commander_feedback_ =
    commander_feedback_expected_ && now <= commander_feedback_deadline_;
  if (latest_trajectory_is_commander_feedback_) {
    RCLCPP_INFO(get_logger(),
      "Received a trajectory resulting from the stored TCP command to Commander; "
      "feedback loop protection enabled for this trajectory.");
  }
  if (commander_feedback_expected_ && now > commander_feedback_deadline_) {
    commander_feedback_expected_ = false;
  } else if (latest_trajectory_is_commander_feedback_) {
    commander_feedback_expected_ = false;
  }

  const std::string new_signature = trajectory_signature(*latest_trajectory_);
  if (new_signature != last_trajectory_signature_) {
    last_trajectory_signature_ = new_signature;
    ++trajectory_generation_;
  }

  if (trajectory_points_valid() && robot_joints_available()) {
    const auto error = trajectory_endpoint_error_percent();
    if (error.has_value()) {
      RCLCPP_INFO(get_logger(),
        "MoveIt endpoint compared with /SockRes/Joints: normalized mean error=%.3f%%; "
        "threshold=%.3f%%; action=%s",
        *error, joint_similarity_tolerance_percent_,
        (*error < joint_similarity_tolerance_percent_) ? "IGNORE (same configuration)" : "MOTION CANDIDATE");
    }
  } else {
    RCLCPP_WARN(get_logger(),
      "Received MoveIt trajectory but cannot compare its endpoint yet: valid trajectory or fresh /SockRes/Joints unavailable.");
  }
}

void ControlDecision::socket_ready_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  socket_ready_ = msg->data;
  if (socket_ready_) {
    // A Ready=true publication acknowledges that the socket can accept another
    // command. Dispatch exactly one queued command, then lower the local gate.
    command_in_flight_ = false;
    purge_expired_commands();
    dispatch_next_command();
  }
}

void ControlDecision::tcp_euler_callback(
  const colab_supervision_interfaces::msg::PoseCommand::SharedPtr msg)
{
  latest_tcp_euler_ = *msg;
  tcp_euler_received_time_ = get_clock()->now();
}

void ControlDecision::tcp_quaternion_callback(
  const colab_supervision_interfaces::msg::PoseCuaCommand::SharedPtr msg)
{
  latest_tcp_quaternion_ = *msg;
  tcp_quaternion_received_time_ = get_clock()->now();
}

void ControlDecision::robot_joints_callback(
  const colab_supervision_interfaces::msg::Joints::SharedPtr msg)
{
  if (msg->data.size() < 1U) {
    RCLCPP_WARN(get_logger(), "Ignoring empty /SockRes/Joints message.");
    return;
  }
  for (const auto value : msg->data) {
    if (!std::isfinite(value)) {
      RCLCPP_WARN(get_logger(), "Ignoring /SockRes/Joints containing a non-finite value.");
      return;
    }
  }
  latest_robot_joints_ = *msg;
  robot_joints_received_time_ = get_clock()->now();
}

// ============================================================================
// TIMER / PLAN EXECUTION
// ============================================================================

void ControlDecision::decision_timer_callback()
{
  purge_expired_commands();
  const DecisionPlan plan = evaluate_decision();

  // Plans are edge-triggered: enqueue their sequence once, not on each timer
  // tick. Socket readiness callback advances the queue after every response.
  if (!plan_is_active() || !same_plan(plan)) {
    apply_plan(plan);
  }

  if (socket_ready_ && !command_in_flight_) {
    dispatch_next_command();
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
  // A trajectory is eligible for a motion decision only if its endpoint can
  // be compared with a recent robot configuration returned by the socket.
  return trajectory_points_valid() && robot_joints_available() &&
         trajectory_endpoint_error_percent().has_value();
}

bool ControlDecision::robot_joints_available() const
{
  if (!latest_robot_joints_.has_value() || latest_robot_joints_->data.empty()) {
    return false;
  }
  return (get_clock()->now() - robot_joints_received_time_).seconds() <=
         max_robot_joints_age_sec_;
}

std::optional<double> ControlDecision::trajectory_endpoint_error_percent() const
{
  if (!trajectory_points_valid() || !robot_joints_available()) {
    return std::nullopt;
  }

  const auto & joint_trajectory = latest_trajectory_->joint_trajectory;
  const auto & endpoint = joint_trajectory.points.back().positions;
  std::vector<double> measured = latest_robot_joints_->data;

  if (sock_joints_in_degrees_) {
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    for (auto & value : measured) {
      value *= kDegToRad;
    }
  }

  if (endpoint.empty() || endpoint.size() != measured.size()) {
    return std::nullopt;
  }

  // Mean absolute joint error normalized by one full revolution (2*pi rad)
  // per axis. An error <5% means the endpoint is close enough to the current
  // configuration that issuing a new motion would be redundant.
  constexpr double kFullRevolutionRad = 2.0 * 3.14159265358979323846;
  double error_sum = 0.0;
  for (std::size_t i = 0; i < endpoint.size(); ++i) {
    if (!std::isfinite(endpoint[i]) || !std::isfinite(measured[i])) {
      return std::nullopt;
    }
    // All six ABB IRB 140 joints are revolute. Compare the shortest angular
    // distance so, for example, +179 deg and -181 deg are treated as equivalent.
    const double angular_error = std::remainder(
      endpoint[i] - measured[i], kFullRevolutionRad);
    error_sum += std::abs(angular_error);
  }
  const double mean_error = error_sum / static_cast<double>(endpoint.size());
  return (mean_error / kFullRevolutionRad) * 100.0;
}

bool ControlDecision::trajectory_endpoint_matches_current_joints() const
{
  const auto error = trajectory_endpoint_error_percent();
  return error.has_value() && *error < joint_similarity_tolerance_percent_;
}

bool ControlDecision::trajectory_requires_motion() const
{
  const auto error = trajectory_endpoint_error_percent();
  return error.has_value() && *error >= joint_similarity_tolerance_percent_;
}

bool ControlDecision::trajectory_is_current_feedback() const
{
  return latest_trajectory_is_commander_feedback_;
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

  if (send_trajectory_interest_command_ && trajectory_requires_motion() &&
      last_interes_trajectory_signature_ != last_trajectory_signature_) {
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

  if (send_trajectory_interest_command_ && trajectory_requires_motion() &&
      last_interes_trajectory_signature_ != last_trajectory_signature_) {
    plan.steps.push_back({
        make_update_interest(
          "PRIORITY 5: execute/update the planned trajectory at reduced speed")});
  } else if (trajectory_endpoint_matches_current_joints()) {
    RCLCPP_INFO(get_logger(), "Skipping Interes: MoveIt endpoint is within the joint similarity threshold.");
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

  if (send_trajectory_interest_command_ && trajectory_requires_motion() &&
      last_interes_trajectory_signature_ != last_trajectory_signature_) {
    plan.steps.push_back({
        make_update_interest(
          "PRIORITY 6: command execution/update of the planned trajectory")});
  } else if (trajectory_endpoint_matches_current_joints()) {
    RCLCPP_INFO(get_logger(), "Skipping Interes: MoveIt endpoint is within the joint similarity threshold.");
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
  // A different decision supersedes commands that have not reached the socket.
  // Keep the in-flight command untouched, but discard queued steps belonging to
  // the previous plan so stale motion cannot be executed after conditions change.
  const bool pending_interest = std::any_of(
    command_queue_.begin(), command_queue_.end(),
    [](const QueuedCommand & queued) {
      return queued.command.command == "Distancia" &&
        queued.command.object == "CambiarTrayectoria" &&
        queued.command.variable == "ActualizarPuntos" &&
        queued.command.value == "Interes";
    });
  const bool safety_action = plan.action == Action::EMERGENCY_STOP ||
    plan.action == Action::SAFE_STOP || plan.action == Action::PAUSE;
  command_queue_.clear();
  if (pending_interest || safety_action) {
    last_interes_trajectory_signature_.clear();
  }


  active_plan_ = plan;
  active_step_index_ = 0;
  plan_initialized_ = false;

  RCLCPP_INFO(
    get_logger(),
    "DECISION: action=%s | reason=%s | steps=%zu | queue=%zu",
    action_to_string(active_plan_.action), active_plan_.reason.c_str(),
    active_plan_.steps.size(), command_queue_.size());

  enqueue_plan_commands(active_plan_);
  plan_initialized_ = true;

  const bool motion_permitted =
    active_plan_.action == Action::LIMIT_SPEED ||
    active_plan_.action == Action::NORMAL_OPERATION;

  if (motion_permitted && trajectory_requires_motion() &&
      !trajectory_is_current_feedback())
  {
    publish_cached_tcp_to_commander_once();
  }

  if (socket_ready_ && !command_in_flight_) {
    dispatch_next_command();
  }
}

void ControlDecision::enqueue_plan_commands(const DecisionPlan & plan)
{
  for (const auto & step : plan.steps) {
    const auto & command = step.command;
    const bool pulse_already_sent = command.command == "Variable" &&
      command.object == "SET" && command.value == "TRUE" &&
      ((command.variable == "Sock_Pausa" && pause_pulse_sent_) ||
       (command.variable == "Sock_Bloquear" && block_pulse_sent_) ||
       (command.variable == "Sock_Parada" && stop_pulse_sent_));
    if (pulse_already_sent) {
      RCLCPP_DEBUG(get_logger(), "Skipping previously-sent pulse %s while rebuilding plan.",
        command.variable.c_str());
      continue;
    }

    const auto size_before = command_queue_.size();
    enqueue_command(command, false);

    const bool enqueued = command_queue_.size() > size_before;
    if (enqueued && step.command.command == "Distancia" &&
        step.command.object == "CambiarTrayectoria" &&
        step.command.variable == "ActualizarPuntos" &&
        step.command.value == "Interes")
    {
      last_interes_trajectory_signature_ = last_trajectory_signature_;
    }
  }
}

void ControlDecision::enqueue_command(const DecisionResult & command, bool priority)
{
  purge_expired_commands();

  if (priority) {
    // Put safety pulses ahead of noncritical commands. Clear the queue to avoid
    // executing old movement commands after a newly-detected safety event.
    command_queue_.clear();
    command_queue_.push_front({command, std::chrono::steady_clock::now()});
    return;
  }

  if (command_queue_.size() >= static_cast<std::size_t>(command_queue_capacity_)) {
    RCLCPP_WARN(get_logger(),
      "Socket command queue full (%zu/%d). Dropping new command: %s/%s/%s/%s",
      command_queue_.size(), command_queue_capacity_, command.command.c_str(),
      command.object.c_str(), command.variable.c_str(), command.value.c_str());
    return;
  }

  command_queue_.push_back({command, std::chrono::steady_clock::now()});
}

void ControlDecision::purge_expired_commands()
{
  const auto now = std::chrono::steady_clock::now();
  const auto ttl = std::chrono::duration<double>(command_queue_ttl_sec_);
  std::size_t removed = 0;
  bool expired_interest = false;

  for (auto it = command_queue_.begin(); it != command_queue_.end();) {
    if (now - it->enqueued_at > ttl) {
      expired_interest = expired_interest || (
        it->command.command == "Distancia" &&
        it->command.object == "CambiarTrayectoria" &&
        it->command.variable == "ActualizarPuntos" &&
        it->command.value == "Interes");
      it = command_queue_.erase(it);
      ++removed;
    } else {
      ++it;
    }
  }

  if (removed > 0U) {
    // If one step in a command sequence expires, discard the remaining queued
    // steps too. Re-evaluate the current state before composing a fresh plan.
    command_queue_.clear();
    plan_initialized_ = false;
    if (expired_interest) {
      last_interes_trajectory_signature_.clear();
    }
    RCLCPP_WARN(get_logger(),
      "Removed %zu expired socket command(s). Remaining sequence discarded; "
      "decision will be reevaluated before a new command is queued.", removed);
  }
}

void ControlDecision::dispatch_next_command()
{
  if (!socket_ready_ || command_in_flight_) {
    return;
  }

  purge_expired_commands();
  if (command_queue_.empty()) {
    return;
  }

  const auto queued = command_queue_.front();
  command_queue_.pop_front();

  colab_supervision_interfaces::msg::ControlCommand msg;
  msg.command = queued.command.command;
  msg.object = queued.command.object;
  msg.variable = queued.command.variable;
  msg.value = queued.command.value;

  RCLCPP_INFO(get_logger(),
    "SOCKET TX [ONE-SHOT]: %s/%s/%s/%s | reason=%s | pending=%zu",
    queued.command.command.c_str(), queued.command.object.c_str(),
    queued.command.variable.c_str(), queued.command.value.c_str(),
    queued.command.reason.c_str(), command_queue_.size());

  // Publish one command and lower the local Ready gate until the next
  // /SockRes/Ready=true response from the bridge.
  socket_ready_ = false;
  command_in_flight_ = true;
  command_pub_->publish(msg);

  if (msg.command == "Variable" && msg.object == "SET" && msg.value == "TRUE") {
    if (msg.variable == "Sock_Pausa") {
      restart_required_ = true;
      pause_pulse_sent_ = true;
    } else if (msg.variable == "Sock_Bloquear") {
      restart_required_ = true;
      block_pulse_sent_ = true;
    } else if (msg.variable == "Sock_Parada") {
      restart_required_ = true;
      stop_pulse_sent_ = true;
    } else if (msg.variable == "Sock_Reinicio") {
      restart_required_ = false;
      pause_pulse_sent_ = false;
      block_pulse_sent_ = false;
      stop_pulse_sent_ = false;
    }
  }

  if (msg.command == "Velocidad" && msg.object == "Porcentaje" && msg.variable == "VelMax") {
    try {
      last_commanded_speed_percent_ = std::stoi(msg.value);
    } catch (const std::exception &) {
      RCLCPP_WARN(get_logger(), "Unable to store commanded speed '%s' as an integer", msg.value.c_str());
    }
  }
}

void ControlDecision::publish_cached_tcp_to_commander_once()
{
  if (latest_trajectory_.has_value() &&
      last_commander_sent_trajectory_signature_ == last_trajectory_signature_) {
    return;
  }

  const auto now = get_clock()->now();
  if (commander_pose_mode_ == "quaternion") {
    if (!latest_tcp_quaternion_.has_value() ||
        (now - tcp_quaternion_received_time_).seconds() > max_tcp_state_age_sec_) {
      RCLCPP_WARN(get_logger(),
        "Motion is authorized but no fresh /SockRes/TCPcua is available; "
        "Commander pose command was not published.");
      return;
    }
    auto command = *latest_tcp_quaternion_;
    command.pose.x *= tcp_position_scale_to_commander_;
    command.pose.y *= tcp_position_scale_to_commander_;
    command.pose.z *= tcp_position_scale_to_commander_;
    command.cartesian_path = commander_cartesian_path_;
    commander_feedback_expected_ = true;
    commander_feedback_deadline_ = now + rclcpp::Duration::from_seconds(3.0);
    last_commander_sent_trajectory_signature_ = last_trajectory_signature_;
    commander_quaternion_pub_->publish(command);
    RCLCPP_INFO(get_logger(),
      "Published stored TCP quaternion to Commander on %s (one-shot).",
      commander_quaternion_topic_.c_str());
    return;
  }

  if (commander_pose_mode_ == "euler") {
    if (!latest_tcp_euler_.has_value() ||
        (now - tcp_euler_received_time_).seconds() > max_tcp_state_age_sec_) {
      RCLCPP_WARN(get_logger(),
        "Motion is authorized but no fresh /SockRes/TCPeul is available; "
        "Commander pose command was not published.");
      return;
    }
    auto command = *latest_tcp_euler_;
    command.pose.x *= tcp_position_scale_to_commander_;
    command.pose.y *= tcp_position_scale_to_commander_;
    command.pose.z *= tcp_position_scale_to_commander_;
    if (tcp_euler_angles_in_degrees_) {
      constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
      command.pose.roll *= kDegToRad;
      command.pose.pitch *= kDegToRad;
      command.pose.yaw *= kDegToRad;
    }
    command.cartesian_path = commander_cartesian_path_;
    commander_feedback_expected_ = true;
    commander_feedback_deadline_ = now + rclcpp::Duration::from_seconds(3.0);
    last_commander_sent_trajectory_signature_ = last_trajectory_signature_;
    commander_euler_pub_->publish(command);
    RCLCPP_INFO(get_logger(),
      "Published stored TCP Euler pose to Commander on %s (one-shot).",
      commander_euler_topic_.c_str());
    return;
  }

  // In joint mode send the planned endpoint through Commander's Joints input.
  // The two TCP response messages continue to be cached for later decisions.
  if (!trajectory_points_valid()) {
    return;
  }
  colab_supervision_interfaces::msg::Joints command;
  command.data = latest_trajectory_->joint_trajectory.points.back().positions;
  commander_feedback_expected_ = true;
  commander_feedback_deadline_ = now + rclcpp::Duration::from_seconds(3.0);
  last_commander_sent_trajectory_signature_ = last_trajectory_signature_;
  commander_joint_pub_->publish(command);
  RCLCPP_INFO(get_logger(),
    "Published MoveIt endpoint configuration to Commander on %s (one-shot).",
    commander_joint_topic_.c_str());
}

std::string ControlDecision::trajectory_signature(
  const moveit_msgs::msg::RobotTrajectory & trajectory) const
{
  std::ostringstream ss;
  const auto & joint_trajectory = trajectory.joint_trajectory;
  ss << joint_trajectory.joint_names.size() << "|"
     << joint_trajectory.points.size();
  for (const auto & name : joint_trajectory.joint_names) {
    ss << "|" << name;
  }
  // Include every waypoint, not only the endpoint: distinct paths can have the
  // same goal while representing different motion plans.
  for (const auto & point : joint_trajectory.points) {
    ss << "|" << point.positions.size();
    for (const auto value : point.positions) {
      ss << "|" << std::setprecision(9) << value;
    }
  }
  return ss.str();
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

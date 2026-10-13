#ifndef COLAB_SUPERVISION_CONTROL__CONTROL_DECISION_HPP_
#define COLAB_SUPERVISION_CONTROL__CONTROL_DECISION_HPP_

#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <colab_supervision_interfaces/msg/control_command.hpp>
#include <colab_supervision_interfaces/msg/operator_state_analysis.hpp>
#include <colab_supervision_interfaces/msg/robot_operator_supervision.hpp>
#include <colab_supervision_interfaces/msg/joints.hpp>
#include <colab_supervision_interfaces/msg/pose_command.hpp>
#include <colab_supervision_interfaces/msg/pose_cua_command.hpp>

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace colab_supervision_control
{

class ControlDecision : public rclcpp::Node
{
public:
  ControlDecision();
  ~ControlDecision() override = default;

private:
  // ==========================================================================
  // OPERATING ZONES
  //
  // Z3: d < 0.250 m
  // Z4: 0.250 m <= d < 0.400 m
  // Z5: 0.400 m <= d <= 1.600 m
  // Z6: d > 1.600 m
  // ===========================================================================

  enum class Zone
  {
    Z3,
    Z4,
    Z5,
    Z6,
    UNKNOWN
  };

  // ==========================================================================
  // DECISION ACTIONS
  // ===========================================================================

  enum class Action
  {
    EMERGENCY_STOP,
    SAFE_STOP,
    PAUSE,
    INCREASE_DISTANCE,
    LIMIT_SPEED,
    NORMAL_OPERATION,
    UNKNOWN
  };

  // ==========================================================================
  // TRAJECTORY CHANGE PROTOCOL
  // ==========================================================================

  enum class ProtocolStage
  {
    NONE,
    EXIT_WAIT_TCP_INITIAL,
    EXIT_WAIT_TCP_AFTER_MOVE,
    EXIT_WAIT_COMMANDER_TRAJECTORY,
    SAFE_WAIT,
    RECOVERY_WAIT_TCP,
    RECOVERY_WAIT_MOTION
  };


  // ==========================================================================
  // COMMAND REPRESENTATION
  // ===========================================================================

  struct DecisionResult
  {
    std::string command;
    std::string object;
    std::string variable;
    std::string value;
    std::string reason;
  };

  struct CommandStep
  {
    DecisionResult command;
  };

  struct DecisionPlan
  {
    Action action{Action::UNKNOWN};
    std::string signature;
    std::vector<CommandStep> steps;
    std::string reason;
  };

  // ==========================================================================
  // INPUT CALLBACKS
  // ===========================================================================

  void operator_state_callback(
    const colab_supervision_interfaces::msg::OperatorStateAnalysis::SharedPtr msg);

  void supervision_callback(
    const colab_supervision_interfaces::msg::RobotOperatorSupervision::SharedPtr msg);

  void trajectory_callback(
    const moveit_msgs::msg::RobotTrajectory::SharedPtr msg);

  void socket_ready_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void tcp_euler_callback(
    const colab_supervision_interfaces::msg::PoseCommand::SharedPtr msg);
  void tcp_quaternion_callback(
    const colab_supervision_interfaces::msg::PoseCuaCommand::SharedPtr msg);
  void robot_joints_callback(
    const colab_supervision_interfaces::msg::Joints::SharedPtr msg);

  // ==========================================================================
  // TRAJECTORY CHANGE PROTOCOLS
  // ==========================================================================

  bool safety_exit_condition() const;
  bool zone5_recovered() const;
  void start_exit_protocol();
  void continue_exit_protocol_from_tcp();
  void start_recovery_protocol();
  void continue_recovery_protocol_from_tcp();
  void handle_commander_trajectory_for_exit(
    const moveit_msgs::msg::RobotTrajectory & trajectory);
  void publish_safe_posture_name_from_tcp();
  void enqueue_protocol_command(const DecisionResult & command);
  DecisionResult make_set_offsets_value(
    const std::string & offsets,
    const std::string & reason) const;

  // ==========================================================================
  // MAIN DECISION LOOP
  // ===========================================================================

  void decision_timer_callback();
  void speed_timer_callback();
  DecisionPlan evaluate_decision();

  // ==========================================================================
  // INPUT VALIDATION / FRESHNESS
  // ==========================================================================

  bool is_operator_state_fresh() const;
  bool is_supervision_fresh() const;
  bool is_trajectory_fresh() const;
  bool processed_information_available() const;

  // ==========================================================================
  // SUPERVISED CONDITIONS
  // ==========================================================================

  bool operator_condition_anomalous() const;
  bool operator_is_available() const;
  bool collision_is_critical() const;

  // ==========================================================================
  // ZONE EVALUATION
  // ==========================================================================

  Zone determine_zone(double distance) const;
  const char * zone_to_string(Zone zone) const;
  const char * action_to_string(Action action) const;

  // ==========================================================================
  // TRAJECTORY EVALUATION
  // ==========================================================================

  bool trajectory_available() const;
  bool trajectory_target_reachable() const;
  bool trajectory_points_valid() const;
  bool robot_joints_available() const;
  bool trajectory_endpoint_matches_current_joints() const;
  std::optional<double> trajectory_endpoint_error_percent() const;
  bool trajectory_requires_motion() const;
  bool trajectory_is_current_feedback() const;

  bool trajectory_keeps_separation(
    Zone zone,
    double distance) const;

  // ==========================================================================
  // SPEED PROFILE
  // ==========================================================================

  int compute_speed_percent(double distance) const;
  int compute_effective_speed_percent(double distance) const;
  double fatigue_speed_factor() const;
  bool speed_change_requires_command(int desired_speed_percent) const;
  void publish_speed_if_required(
    int desired_speed_percent,
    const std::string & reason);

  // ==========================================================================
  // COMMAND BUILDERS - EXACT COMMAND GRAMMAR
  // ==========================================================================

  DecisionResult make_variable_set(
    const std::string & variable,
    const std::string & value,
    const std::string & reason) const;

  DecisionResult make_speed_percent(
    int speed_percent,
    const std::string & reason) const;

  DecisionResult make_speed_max(
    int speed_percent,
    const std::string & reason) const;

  DecisionResult make_get_tcp_euler(
    const std::string & reason) const;

  DecisionResult make_get_tcp_quaternion(
    const std::string & reason) const;

  DecisionResult make_update_interest(
    const std::string & reason) const;

  DecisionResult make_set_offsets(
    const std::string & reason) const;

  DecisionResult make_save_exit_point(
    const std::string & reason) const;

  DecisionResult make_move_temp(
    const std::string & move_type,
    const std::string & reason) const;

  DecisionResult make_safe_posture_joint(
    const std::string & point_name,
    const std::string & reason) const;

  DecisionResult make_safe_posture_linear(
    const std::string & point_name,
    const std::string & reason) const;

  DecisionResult make_joint_configuration(
    const std::vector<double> & joints,
    const std::string & reason) const;

  // ==========================================================================
  // DECISION PLAN BUILDERS
  // ===========================================================================

  DecisionPlan make_emergency_stop_plan(
    const std::string & reason) const;

  DecisionPlan make_safe_stop_plan(
    const std::string & reason) const;

  DecisionPlan make_pause_plan(
    const std::string & reason) const;

  DecisionPlan make_increase_distance_plan(
    int speed_percent,
    const std::string & reason) const;

  DecisionPlan make_limit_speed_plan(
    int speed_percent,
    const std::string & reason) const;

  DecisionPlan make_normal_operation_plan(
    int speed_percent,
    const std::string & reason) const;

  DecisionPlan make_fail_safe_plan(
    const std::string & reason) const;

  // ==========================================================================
  // PLAN EXECUTION / ONE-SHOT COMMAND STATE
  // ===========================================================================

  void apply_plan(const DecisionPlan & plan);
  void enqueue_plan_commands(const DecisionPlan & plan);
  void enqueue_command(const DecisionResult & command, bool priority = false);
  void purge_expired_commands();
  void dispatch_next_command();
  bool plan_is_active() const;
  bool same_plan(const DecisionPlan & plan) const;

  void publish_cached_tcp_to_commander_once();
  std::string trajectory_signature(
    const moveit_msgs::msg::RobotTrajectory & trajectory) const;

  // ==========================================================================
  // COMMAND HELPERS
  // ===========================================================================

  static std::string format_array3(
    double x,
    double y,
    double z);

  static std::string format_joint_configuration(
    const std::vector<double> & joints);

  // ==========================================================================
  // PARAMETERS
  // ===========================================================================

  std::string operator_state_topic_;
  std::string supervision_topic_;
  std::string trajectory_topic_;
  std::string command_topic_;
  std::string socket_ready_topic_;
  std::string tcp_euler_topic_;
  std::string tcp_quaternion_topic_;
  std::string robot_joints_topic_;
  std::string commander_euler_topic_;
  std::string commander_quaternion_topic_;
  std::string commander_joint_topic_;
  std::string name_command_topic_;

  double decision_rate_hz_;
  double max_input_age_sec_;
  double max_trajectory_age_sec_;
  double max_robot_joints_age_sec_;
  double max_tcp_state_age_sec_;

  // Zones
  double zone3_max_distance_;
  double zone4_max_distance_;
  double zone5_max_distance_;

  // Speed profile
  double full_speed_distance_;
  double half_speed_distance_;
  double minimum_speed_distance_;

  double full_speed_percent_;
  double half_speed_percent_;
  double minimum_speed_percent_;
  // Fixed tolerance in percentage points. Default: +/-3 points.
  double speed_change_threshold_percent_;
  // Periodic VelMax evaluation rate. Default: 0.25 Hz (one evaluation every 4 s).
  double speed_command_rate_hz_;
  double fatigue_medium_speed_factor_;

  // Operator conditions
  bool pause_on_fatigue_high_;
  bool pause_on_attention_other_;
  bool pause_on_attention_unknown_;

  // Trajectory safety
  bool conservative_trajectory_check_;
  double trajectory_min_separation_distance_;
  double joint_similarity_tolerance_percent_;
  bool sock_joints_in_degrees_;

  // Command grammar / trajectory execution
  bool send_trajectory_interest_command_;
  bool send_restart_before_motion_;
  bool send_speed_command_before_motion_;
  bool block_during_distance_correction_;
  std::string distance_move_type_;
  std::string set_offsets_value_;
  std::string commander_pose_mode_;
  bool commander_cartesian_path_;
  double tcp_position_scale_to_commander_;
  bool tcp_euler_angles_in_degrees_;

  // Socket queue configuration. A command is sent only after Ready=true.
  int command_queue_capacity_;
  double command_queue_ttl_sec_;

  // Optional safe posture commands reserved for later integration
  std::string safe_posture_joint_point_;
  std::string safe_posture_linear_point_;

  // ==========================================================================
  // ROS INTERFACES
  // ===========================================================================

  rclcpp::Subscription<
    colab_supervision_interfaces::msg::OperatorStateAnalysis>::SharedPtr
    operator_state_sub_;

  rclcpp::Subscription<
    colab_supervision_interfaces::msg::RobotOperatorSupervision>::SharedPtr
    supervision_sub_;

  rclcpp::Subscription<
    moveit_msgs::msg::RobotTrajectory>::SharedPtr
    trajectory_sub_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    socket_ready_sub_;
  rclcpp::Subscription<
    colab_supervision_interfaces::msg::PoseCommand>::SharedPtr
    tcp_euler_sub_;
  rclcpp::Subscription<
    colab_supervision_interfaces::msg::PoseCuaCommand>::SharedPtr
    tcp_quaternion_sub_;
  rclcpp::Subscription<
    colab_supervision_interfaces::msg::Joints>::SharedPtr
    robot_joints_sub_;

  rclcpp::Publisher<
    colab_supervision_interfaces::msg::ControlCommand>::SharedPtr
    command_pub_;
  rclcpp::Publisher<
    colab_supervision_interfaces::msg::PoseCommand>::SharedPtr
    commander_euler_pub_;
  rclcpp::Publisher<
    colab_supervision_interfaces::msg::PoseCuaCommand>::SharedPtr
    commander_quaternion_pub_;
  rclcpp::Publisher<
    colab_supervision_interfaces::msg::Joints>::SharedPtr
    commander_joint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    name_command_pub_;

  rclcpp::TimerBase::SharedPtr decision_timer_;
  rclcpp::TimerBase::SharedPtr speed_timer_;

  // ==========================================================================
  // LAST PROCESSED STATES
  // ===========================================================================

  std::optional<
    colab_supervision_interfaces::msg::OperatorStateAnalysis>
    latest_operator_state_;

  std::optional<
    colab_supervision_interfaces::msg::RobotOperatorSupervision>
    latest_supervision_;

  std::optional<moveit_msgs::msg::RobotTrajectory>
    latest_trajectory_;
  std::optional<colab_supervision_interfaces::msg::PoseCommand>
    latest_tcp_euler_;
  std::optional<colab_supervision_interfaces::msg::PoseCuaCommand>
    latest_tcp_quaternion_;
  std::optional<colab_supervision_interfaces::msg::Joints>
    latest_robot_joints_;

  rclcpp::Time operator_state_received_time_;
  rclcpp::Time supervision_received_time_;
  rclcpp::Time trajectory_received_time_;
  rclcpp::Time tcp_euler_received_time_;
  rclcpp::Time tcp_quaternion_received_time_;
  rclcpp::Time robot_joints_received_time_;

  // ==========================================================================
  // ACTIVE PLAN STATE
  // ===========================================================================

  DecisionPlan active_plan_;
  std::size_t active_step_index_{0};
  bool plan_initialized_{false};

  // A restart/unlock command is required only after the system has been
  // paused, blocked or stopped. It is cleared when Sock_Reinicio is published.
  bool restart_required_{false};
  bool pause_pulse_sent_{false};
  bool block_pulse_sent_{false};
  bool stop_pulse_sent_{false};

  // Last speed actually sent to the parser. A new VelMax command is emitted
  // only when the calculated value is outside the configured tolerance.
  std::optional<int> last_commanded_speed_percent_;

  // Used to distinguish genuinely new MoveIt plans from the same trajectory
  // message being received again.
  std::string last_trajectory_signature_;
  std::uint64_t trajectory_generation_{0};
  bool latest_trajectory_is_commander_feedback_{false};
  bool commander_feedback_expected_{false};
  rclcpp::Time commander_feedback_deadline_;
  std::string last_commander_sent_trajectory_signature_;
  std::string last_interes_trajectory_signature_;

  struct QueuedCommand
  {
    DecisionResult command;
    std::chrono::steady_clock::time_point enqueued_at;
  };

  std::deque<QueuedCommand> command_queue_;
  bool socket_ready_{true};
  bool command_in_flight_{false};

  // A safety pulse (pause/block/stop) is never allowed to be the first
  // command emitted after node startup. A non-safety command, such as the
  // initial VelMax command, must be sent first.
  bool first_command_sent_{false};

  // ==========================================================================
  // TRAJECTORY CHANGE PROTOCOL STATE
  // ==========================================================================

  ProtocolStage protocol_stage_{ProtocolStage::NONE};
  bool protocol_active_{false};
  bool tcp_request_pending_{false};
  bool tcp_salida_valid_{false};
  bool commander_safe_posture_requested_{false};
  rclcpp::Time tcp_request_time_;

  colab_supervision_interfaces::msg::PoseCuaCommand tcp_salida_{};

  // Fixed offsets defined by the trajectory-change protocol.
  static constexpr double kTcpZThreshold = 0.402;
  static constexpr const char * kOffsetDown = "[0,0,-20]";
  static constexpr const char * kOffsetApproach = "[10,0,-20]";
  static constexpr const char * kOffsetZero = "[0,0,0]";
  static constexpr const char * kSafePosturePositiveY = "PosSeguraYPos";
  static constexpr const char * kSafePostureNegativeY = "PosSeguraYNeg";
};

}  // namespace colab_supervision_control

#endif  // COLAB_SUPERVISION_CONTROL__CONTROL_DECISION_HPP_

#ifndef COLAB_SUPERVISION_CONTROL__CONTROL_DECISION_HPP_
#define COLAB_SUPERVISION_CONTROL__CONTROL_DECISION_HPP_

#include <memory>
#include <optional>
#include <string>
#include <cstdint>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <colab_supervision_interfaces/msg/control_command.hpp>
#include <colab_supervision_interfaces/msg/operator_state_analysis.hpp>
#include <colab_supervision_interfaces/msg/robot_operator_supervision.hpp>

#include <moveit_msgs/msg/robot_trajectory.hpp>

namespace colab_supervision_control
{

class ControlDecision : public rclcpp::Node
{
public:
  ControlDecision();
  ~ControlDecision() override = default;

private:
  // ============================================================================
  // OPERATING ZONES
  //
  // Z3: d < 0.250 m
  // Z4: 0.250 m <= d < 0.400 m
  // Z5: 0.400 m <= d <= 1.600 m
  // Z6: d > 1.600 m
  // ============================================================================

  enum class Zone
  {
    Z3,
    Z4,
    Z5,
    Z6,
    UNKNOWN
  };

  // ============================================================================
  // DECISION ACTIONS
  // ============================================================================

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

  // ============================================================================
  // COMMAND REPRESENTATION
  // ============================================================================

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
    bool repeat{false};
  };

  struct DecisionPlan
  {
    Action action{Action::UNKNOWN};
    std::string signature;
    std::vector<CommandStep> steps;
    std::string reason;
  };

  // ============================================================================
  // INPUT CALLBACKS
  // ============================================================================

  void operator_state_callback(
    const colab_supervision_interfaces::msg::OperatorStateAnalysis::SharedPtr msg);

  void supervision_callback(
    const colab_supervision_interfaces::msg::RobotOperatorSupervision::SharedPtr msg);

  void trajectory_callback(
    const moveit_msgs::msg::RobotTrajectory::SharedPtr msg);

  // ============================================================================
  // MAIN DECISION LOOP
  // ============================================================================

  void decision_timer_callback();

  DecisionPlan evaluate_decision();

  // ============================================================================
  // INPUT VALIDATION / FRESHNESS
  // ============================================================================

  bool is_operator_state_fresh() const;
  bool is_supervision_fresh() const;
  bool is_trajectory_fresh() const;
  bool processed_information_available() const;

  // ============================================================================
  // SUPERVISED CONDITIONS
  // ============================================================================

  bool operator_condition_anomalous() const;
  bool collision_is_critical() const;

  // ============================================================================
  // ZONE EVALUATION
  // ============================================================================

  Zone determine_zone(double distance) const;
  const char * zone_to_string(Zone zone) const;
  const char * action_to_string(Action action) const;

  // ============================================================================
  // TRAJECTORY EVALUATION
  // ============================================================================

  bool trajectory_available() const;
  bool trajectory_target_reachable() const;
  bool trajectory_points_valid() const;

  bool trajectory_keeps_separation(
    Zone zone,
    double distance) const;

  // ============================================================================
  // SPEED PROFILE
  // ============================================================================

  int compute_speed_percent(double distance) const;

  // ============================================================================
  // COMMAND BUILDERS - EXACT COMMAND GRAMMAR
  // ============================================================================

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

  // ============================================================================
  // DECISION PLAN BUILDERS
  // ============================================================================

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

  // ============================================================================
  // PLAN EXECUTION
  // ============================================================================

  void apply_plan(const DecisionPlan & plan);
  void publish_current_step();
  bool plan_is_active() const;
  bool should_repeat_current_step() const;
  bool same_plan(const DecisionPlan & plan) const;

  // ============================================================================
  // COMMAND HELPERS
  // ============================================================================

  static std::string format_array3(
    double x,
    double y,
    double z);

  static std::string format_joint_configuration(
    const std::vector<double> & joints);

  // ============================================================================
  // PARAMETERS
  // ============================================================================

  std::string operator_state_topic_;
  std::string supervision_topic_;
  std::string trajectory_topic_;
  std::string command_topic_;

  double decision_rate_hz_;
  double max_input_age_sec_;
  double max_trajectory_age_sec_;

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

  // Operator conditions
  bool pause_on_fatigue_high_;
  bool pause_on_attention_other_;
  bool pause_on_attention_unknown_;

  // Trajectory safety
  bool conservative_trajectory_check_;
  double trajectory_min_separation_distance_;

  // Command grammar / trajectory execution
  bool send_trajectory_interest_command_;
  bool send_restart_before_motion_;
  bool send_speed_command_before_motion_;
  bool block_during_distance_correction_;
  std::string distance_move_type_;
  std::string set_offsets_value_;

  // Optional safe posture commands reserved for later integration
  std::string safe_posture_joint_point_;
  std::string safe_posture_linear_point_;

  // Timing for one-shot / repeated commands
  double repeat_command_period_sec_;

  // ============================================================================
  // ROS INTERFACES
  // ============================================================================

  rclcpp::Subscription<
    colab_supervision_interfaces::msg::OperatorStateAnalysis>::SharedPtr
    operator_state_sub_;

  rclcpp::Subscription<
    colab_supervision_interfaces::msg::RobotOperatorSupervision>::SharedPtr
    supervision_sub_;

  rclcpp::Subscription<
    moveit_msgs::msg::RobotTrajectory>::SharedPtr
    trajectory_sub_;

  rclcpp::Publisher<
    colab_supervision_interfaces::msg::ControlCommand>::SharedPtr
    command_pub_;

  rclcpp::TimerBase::SharedPtr decision_timer_;

  // ============================================================================
  // LAST PROCESSED STATES
  // ============================================================================

  std::optional<
    colab_supervision_interfaces::msg::OperatorStateAnalysis>
    latest_operator_state_;

  std::optional<
    colab_supervision_interfaces::msg::RobotOperatorSupervision>
    latest_supervision_;

  std::optional<moveit_msgs::msg::RobotTrajectory>
    latest_trajectory_;

  rclcpp::Time operator_state_received_time_;
  rclcpp::Time supervision_received_time_;
  rclcpp::Time trajectory_received_time_;

  // ============================================================================
  // ACTIVE PLAN STATE
  // ============================================================================

  DecisionPlan active_plan_;
  std::size_t active_step_index_{0};
  rclcpp::Time current_step_last_publication_time_;
  bool plan_initialized_{false};

  // Used to distinguish each newly published MoveIt plan.
  std::string last_trajectory_signature_;
  std::uint64_t trajectory_generation_{0};
};

}  // namespace colab_supervision_control

#endif  // COLAB_SUPERVISION_CONTROL__CONTROL_DECISION_HPP_

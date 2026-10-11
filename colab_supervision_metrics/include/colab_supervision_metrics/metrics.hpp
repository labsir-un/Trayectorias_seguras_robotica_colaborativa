#ifndef COLAB_SUPERVISION_METRICS__METRICS_NODE_HPP_
#define COLAB_SUPERVISION_METRICS__METRICS_NODE_HPP_

#include <chrono>
#include <cstdint>
#include <deque>
#include <fstream>
#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"

#include "colab_supervision_interfaces/msg/control_command.hpp"
#include "colab_supervision_interfaces/msg/operator_state_analysis.hpp"
#include "colab_supervision_interfaces/msg/robot_operator_interaction.hpp"
#include "colab_supervision_interfaces/msg/robot_operator_supervision.hpp"

#include "colab_supervision_interfaces/msg/experimental_metrics.hpp"

namespace colab_supervision_metrics
{

class MetricsNode : public rclcpp::Node
{
public:
  MetricsNode();

private:
  struct ObservationSample
  {
    rclcpp::Time time;
    bool operator_valid{false};
    bool robot_valid{false};
    bool interaction_valid{false};
    double operator_x{0.0};
    double operator_y{0.0};
    double operator_z{0.0};
    double robot_x{0.0};
    double robot_y{0.0};
    double robot_z{0.0};
  };

  struct QualityWindowSample
  {
    rclcpp::Time time;
    bool operator_valid{false};
    bool robot_valid{false};
  };

  void interaction_callback(
    const colab_supervision_interfaces::msg::RobotOperatorInteraction::SharedPtr msg);

  void supervision_callback(
    const colab_supervision_interfaces::msg::RobotOperatorSupervision::SharedPtr msg);

  void operator_state_callback(
    const colab_supervision_interfaces::msg::OperatorStateAnalysis::SharedPtr msg);

  void command_callback(
    const colab_supervision_interfaces::msg::ControlCommand::SharedPtr msg);

  void trajectory_callback(
    const moveit_msgs::msg::RobotTrajectory::SharedPtr msg);

  void timer_callback();

  void update_motion_metrics(const ObservationSample & sample);
  void update_adaptation_metric();
  void update_stop_metric();
  void update_quality_window(const ObservationSample & sample);

  bool trajectory_has_valid_joint_points(
    const moveit_msgs::msg::RobotTrajectory & trajectory) const;

  bool current_safety_state_is_compatible() const;

  void publish_metrics();
  void write_csv_row();

  static double euclidean_distance(
    double x1, double y1, double z1,
    double x2, double y2, double z2);

  // Topics
  std::string interaction_topic_;
  std::string supervision_topic_;
  std::string operator_state_topic_;
  std::string command_topic_;
  std::string trajectory_topic_;
  std::string metrics_topic_;

  // Timing
  double publish_rate_hz_;
  double quality_window_sec_;
  double max_observation_age_sec_;
  double stop_speed_threshold_mm_s_;
  double stop_confirmation_time_sec_;
  double adaptation_timeout_sec_;
  double min_trajectory_distance_m_;

  // Subscriptions/publication
  rclcpp::Subscription<
    colab_supervision_interfaces::msg::RobotOperatorInteraction>::SharedPtr interaction_sub_;

  rclcpp::Subscription<
    colab_supervision_interfaces::msg::RobotOperatorSupervision>::SharedPtr supervision_sub_;

  rclcpp::Subscription<
    colab_supervision_interfaces::msg::OperatorStateAnalysis>::SharedPtr operator_state_sub_;

  rclcpp::Subscription<
    colab_supervision_interfaces::msg::ControlCommand>::SharedPtr command_sub_;

  rclcpp::Subscription<
    moveit_msgs::msg::RobotTrajectory>::SharedPtr trajectory_sub_;

  rclcpp::Publisher<
    colab_supervision_interfaces::msg::ExperimentalMetrics>::SharedPtr metrics_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Latest data
  std::optional<ObservationSample> latest_observation_;
  std::optional<colab_supervision_interfaces::msg::RobotOperatorSupervision>
    latest_supervision_;
  std::optional<colab_supervision_interfaces::msg::OperatorStateAnalysis>
    latest_operator_state_;

  rclcpp::Time latest_interaction_receive_time_;
  rclcpp::Time latest_trajectory_receive_time_;
  rclcpp::Time latest_command_receive_time_;

  // Motion metric
  std::optional<ObservationSample> previous_observation_;
  double distance_or_mm_{0.0};
  bool distance_or_valid_{false};
  double robot_speed_mm_s_{0.0};
  bool robot_speed_valid_{false};
  double relative_position_variation_mm_{0.0};
  bool relative_position_variation_valid_{false};

  // Adaptation metric
  bool adaptation_active_{false};
  rclcpp::Time adaptation_start_time_;
  double adaptation_time_ms_{0.0};
  bool adaptation_time_valid_{false};

  // Perception latency
  double perception_time_ms_{0.0};
  bool perception_time_valid_{false};

  std::string last_adaptation_command_signature_;
  std::uint64_t trajectory_generation_{0};
  std::uint64_t trajectory_generation_at_adaptation_start_{0};

  // Stop metric
  bool stop_active_{false};
  rclcpp::Time stop_start_time_;
  std::optional<rclcpp::Time> low_speed_since_;
  double stop_time_ms_{0.0};
  bool stop_time_valid_{false};

  // Trajectory metric
  bool trajectory_feasible_{false};
  bool trajectory_feasible_valid_{false};

  // Perception quality
  std::deque<QualityWindowSample> quality_window_;
  double perception_quality_percent_{0.0};
  bool perception_quality_valid_{false};

  // Last trajectory
  std::optional<moveit_msgs::msg::RobotTrajectory> latest_trajectory_;

  // CSV
  bool csv_enabled_;
  std::string csv_path_;
  std::ofstream csv_file_;
  bool csv_header_written_{false};
};

}  // namespace colab_supervision_metrics

#endif  // COLAB_SUPERVISION_METRICS__METRICS_NODE_HPP_

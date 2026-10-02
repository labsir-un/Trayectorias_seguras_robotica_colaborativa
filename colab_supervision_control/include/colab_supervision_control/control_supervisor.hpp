#ifndef COLAB_SUPERVISION_CONTROL__CONTROL_SUPERVISOR_HPP_
#define COLAB_SUPERVISION_CONTROL__CONTROL_SUPERVISOR_HPP_

#include <deque>
#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <colab_supervision_interfaces/msg/robot_operator_interaction.hpp>

namespace colab_supervision_control
{

class ControlSupervisor : public rclcpp::Node
{
public:
  ControlSupervisor();
  ~ControlSupervisor() override = default;

private:
  struct SampleState
  {
    bool operator_detected{false};
    bool robot_detected{false};
    bool warning{false};
    bool danger{false};
    bool distance_valid{false};
    double distance{0.0};
  };

  enum class Action
  {
    WAIT,
    ALLOW_ACTIONS,
    LIMIT_SPEED,
    PAUSE,
    STOP
  };

  void interaction_callback(
    const colab_supervision_interfaces::msg::RobotOperatorInteraction::SharedPtr msg);

  void evaluate(const colab_supervision_interfaces::msg::RobotOperatorInteraction & msg);

  bool previous_two_warnings() const;
  bool any_warning_last_five() const;
  bool five_consecutive_danger() const;

  void push_history(const SampleState & sample);

  double compute_speed_percent(double distance) const;

  Action determine_action(
    bool operator_detected,
    bool robot_detected,
    bool distance_valid,
    double distance,
    bool previous_two_warning,
    bool any_warning_last_five,
    bool five_danger,
    std::string & reason) const;

  static const char * action_to_string(Action action);

  void publish_decision(
    Action action,
    double speed_percent,
    bool actions_allowed,
    const std::string & reason,
    const colab_supervision_interfaces::msg::RobotOperatorInteraction & msg,
    bool distance_valid,
    double distance);

  std::string interaction_topic_;
  std::string action_topic_;
  std::string speed_topic_;
  std::string actions_allowed_topic_;
  std::string status_topic_;

  double full_speed_distance_;
  double half_speed_distance_;
  double minimum_speed_distance_;
  double full_speed_percent_;
  double half_speed_percent_;
  double minimum_speed_percent_;

  double robot_not_detected_min_distance_;
  double stop_distance_;
  int warning_history_size_;
  int danger_pause_frames_;
  double distance_memory_timeout_;

  rclcpp::Subscription<
    colab_supervision_interfaces::msg::RobotOperatorInteraction>::SharedPtr interaction_sub_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr action_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr actions_allowed_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  std::deque<SampleState> history_;

  std::optional<double> last_valid_distance_;
  rclcpp::Time last_valid_distance_time_;

  bool initialized_{false};
};

}  // namespace colab_supervision_control

#endif  // COLAB_SUPERVISION_CONTROL__CONTROL_SUPERVISOR_HPP_

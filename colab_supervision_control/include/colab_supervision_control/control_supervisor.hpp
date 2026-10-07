#ifndef COLAB_SUPERVISION_CONTROL__CONTROL_SUPERVISOR_HPP_
#define COLAB_SUPERVISION_CONTROL__CONTROL_SUPERVISOR_HPP_

#include <deque>
#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <colab_supervision_interfaces/msg/robot_operator_interaction.hpp>
#include <colab_supervision_interfaces/msg/robot_operator_supervision.hpp>

namespace colab_supervision_control
{

class ControlSupervisor : public rclcpp::Node
{
public:
  ControlSupervisor();
  ~ControlSupervisor() override = default;

private:
  // ========================================================================
  // SAMPLE STORED IN THE TEMPORAL HISTORY
  // ========================================================================

  struct SampleState
  {
    bool operator_detected{false};
    bool robot_detected{false};

    bool warning{false};
    bool danger{false};

    bool distance_valid{false};
    double distance{0.0};
  };

  // ========================================================================
  // ROS CALLBACK
  // ========================================================================

  void interaction_callback(
    const colab_supervision_interfaces::msg::RobotOperatorInteraction::SharedPtr msg);

  // ========================================================================
  // PREPROCESSING / SUPERVISION STATE
  // ========================================================================

  void process_interaction(
    const colab_supervision_interfaces::msg::RobotOperatorInteraction & msg);

  // ========================================================================
  // TEMPORAL CONDITIONS
  // ========================================================================

  bool any_warning_last_window() const;

  bool danger_persistent() const;

  void push_history(const SampleState & sample);

  // ========================================================================
  // DISTANCE MEMORY
  // ========================================================================

  bool get_distance(
    const colab_supervision_interfaces::msg::RobotOperatorInteraction & msg,
    double & distance,
    bool & distance_from_memory);

  // ========================================================================
  // STATE DESCRIPTION
  // ========================================================================

  std::string determine_supervision_state(
    const colab_supervision_interfaces::msg::RobotOperatorInteraction & msg,
    bool distance_valid,
    bool distance_from_memory) const;

  // ========================================================================
  // OUTPUT
  // ========================================================================

  void publish_supervision_state(
    const colab_supervision_interfaces::msg::RobotOperatorInteraction & msg,
    bool distance_valid,
    double distance,
    bool distance_from_memory,
    bool warning_in_window,
    bool danger_is_persistent,
    const std::string & supervision_state);

  // ========================================================================
  // PARAMETERS
  // ========================================================================

  std::string interaction_topic_;
  std::string supervision_topic_;

  // Temporal preprocessing
  int warning_history_size_;
  int danger_persistent_frames_;

  // Last valid distance memory
  double distance_memory_timeout_;

  // ========================================================================
  // ROS INTERFACES
  // ========================================================================

  rclcpp::Subscription<
    colab_supervision_interfaces::msg::RobotOperatorInteraction>::SharedPtr
    interaction_sub_;

  rclcpp::Publisher<
    colab_supervision_interfaces::msg::RobotOperatorSupervision>::SharedPtr
    supervision_pub_;

  // ========================================================================
  // INTERNAL STATE
  // ========================================================================

  std::deque<SampleState> history_;

  std::optional<double> last_valid_distance_;
  rclcpp::Time last_valid_distance_time_;

  bool initialized_{false};
};

}  // namespace colab_supervision_control

#endif  // COLAB_SUPERVISION_CONTROL__CONTROL_SUPERVISOR_HPP_

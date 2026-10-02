#ifndef COLAB_SUPERVISION_CONTROL__OPERATOR_CONDITION_HPP_
#define COLAB_SUPERVISION_CONTROL__OPERATOR_CONDITION_HPP_

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <colab_supervision_interfaces/msg/operator_perception.hpp>
#include <colab_supervision_interfaces/msg/operator_posture.hpp>
#include <colab_supervision_interfaces/msg/operator_state_analysis.hpp>

namespace colab_supervision_control
{

class OperatorCondition : public rclcpp::Node
{
public:
  OperatorCondition();
  ~OperatorCondition() override = default;

private:
  // ==========================================================================
  // MUESTRA TEMPORAL
  // ==========================================================================

  struct OperatorSample
  {
    bool person_detected{false};
    bool person_visible{false};

    float attention_score{0.0F};
    std::string attention_target{"unknown"};

    float fatigue_score{0.0F};
    std::string fatigue_level{"UNKNOWN"};

    float head_yaw{0.0F};
    float head_pitch{0.0F};
    float head_roll{0.0F};
    float head_nodding_score{0.0F};
  };

  struct AggregatedCondition
  {
    bool valid{false};
    std::string condition{"UNKNOWN"};
    float score{0.0F};
  };

  // ==========================================================================
  // CALLBACK / PROCESAMIENTO
  // ==========================================================================

  void operator_perception_callback(
    const colab_supervision_interfaces::msg::OperatorPerception::SharedPtr msg);

  void process_operator_state(
    const colab_supervision_interfaces::msg::OperatorPerception & msg);

  void push_sample(const OperatorSample & sample);

  // ==========================================================================
  // AGREGACION TEMPORAL
  // ==========================================================================

  bool evaluate_presence() const;

  AggregatedCondition evaluate_attention() const;

  AggregatedCondition evaluate_fatigue() const;

  AggregatedCondition evaluate_posture() const;

  std::string majority_string(
    const std::vector<std::string> & values,
    const std::string & default_value) const;

  // ==========================================================================
  // POSTURA
  // ==========================================================================

  bool posture_sample_valid(
    const OperatorSample & sample) const;

  float posture_sample_score(
    const OperatorSample & sample) const;

  std::string posture_sample_condition(
    const OperatorSample & sample) const;

  // ==========================================================================
  // PUBLICACION
  // ==========================================================================

  void publish_conditions(
    bool presence,
    const AggregatedCondition & attention,
    const AggregatedCondition & fatigue,
    const AggregatedCondition & posture);

  // ==========================================================================
  // PARAMETROS
  // ==========================================================================

  std::string perception_topic_;
  std::string state_analysis_topic_;
  std::string posture_topic_;

  int history_size_;

  float posture_max_yaw_rad_;
  float posture_max_pitch_rad_;
  float posture_max_roll_rad_;
  float posture_max_nodding_score_;

  // ==========================================================================
  // ROS 2
  // ==========================================================================

  rclcpp::Subscription<
    colab_supervision_interfaces::msg::OperatorPerception>::SharedPtr
    perception_sub_;

  rclcpp::Publisher<
    colab_supervision_interfaces::msg::OperatorStateAnalysis>::SharedPtr
    state_analysis_pub_;

  rclcpp::Publisher<
    colab_supervision_interfaces::msg::OperatorPosture>::SharedPtr
    posture_pub_;

  // ==========================================================================
  // ESTADO
  // ==========================================================================

  std::deque<OperatorSample> history_;

  bool initialized_{false};
};

}  // namespace colab_supervision_control

#endif  // COLAB_SUPERVISION_CONTROL__OPERATOR_CONDITION_HPP_
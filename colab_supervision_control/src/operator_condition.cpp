#include "colab_supervision_control/operator_condition.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <map>
#include <string>
#include <vector>

namespace colab_supervision_control
{

// ============================================================================
// CONSTRUCTOR
// ============================================================================

OperatorCondition::OperatorCondition()
: Node("operator_condition_node")
{
  // ==========================================================================
  // INPUT
  // ==========================================================================

  perception_topic_ = declare_parameter<std::string>(
    "perception_topic",
    "/state_operator"
  );

  // ==========================================================================
  // OUTPUT
  // ==========================================================================

  state_analysis_topic_ = declare_parameter<std::string>(
    "state_analysis_topic",
    "/operator/state_analisis"
  );

  posture_topic_ = declare_parameter<std::string>(
    "posture_topic",
    "/operator/posture"
  );

  // ==========================================================================
  // TEMPORAL WINDOW
  // ==========================================================================

  history_size_ = declare_parameter<int>(
    "history_size",
    5
  );

  // ==========================================================================
  // POSTURE THRESHOLDS
  //
  // head_yaw / pitch / roll are expressed in radians.
  // Default = 25 degrees.
  // ==========================================================================

  constexpr float kDefaultPostureAngleRad =
    0.436332312F;

  posture_max_yaw_rad_ = declare_parameter<float>(
    "posture_max_yaw_rad",
    kDefaultPostureAngleRad
  );

  posture_max_pitch_rad_ = declare_parameter<float>(
    "posture_max_pitch_rad",
    kDefaultPostureAngleRad
  );

  posture_max_roll_rad_ = declare_parameter<float>(
    "posture_max_roll_rad",
    kDefaultPostureAngleRad
  );

  posture_max_nodding_score_ = declare_parameter<float>(
    "posture_max_nodding_score",
    0.60F
  );

  if (history_size_ < 1) {
    RCLCPP_WARN(
      get_logger(),
      "history_size invalido. Se utiliza 5."
    );
    history_size_ = 5;
  }

  if (posture_max_yaw_rad_ <= 0.0F) {
    posture_max_yaw_rad_ = kDefaultPostureAngleRad;
  }

  if (posture_max_pitch_rad_ <= 0.0F) {
    posture_max_pitch_rad_ = kDefaultPostureAngleRad;
  }

  if (posture_max_roll_rad_ <= 0.0F) {
    posture_max_roll_rad_ = kDefaultPostureAngleRad;
  }

  if (posture_max_nodding_score_ <= 0.0F) {
    posture_max_nodding_score_ = 0.60F;
  }

  // ==========================================================================
  // SUBSCRIBER
  // ==========================================================================

  perception_sub_ =
    create_subscription<
      colab_supervision_interfaces::msg::OperatorPerception
    >(
      perception_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
        &OperatorCondition::operator_perception_callback,
        this,
        std::placeholders::_1
      )
    );

  // ==========================================================================
  // PUBLISHERS
  // ==========================================================================

  state_analysis_pub_ =
    create_publisher<
      colab_supervision_interfaces::msg::OperatorStateAnalysis
    >(
      state_analysis_topic_,
      10
    );

  posture_pub_ =
    create_publisher<
      colab_supervision_interfaces::msg::OperatorPosture
    >(
      posture_topic_,
      10
    );

  // ==========================================================================
  // LOG
  // ==========================================================================

  RCLCPP_INFO(
    get_logger(),
    "===================================================="
  );

  RCLCPP_INFO(
    get_logger(),
    "OPERATOR CONDITION NODE"
  );

  RCLCPP_INFO(
    get_logger(),
    "Input: %s",
    perception_topic_.c_str()
  );

  RCLCPP_INFO(
    get_logger(),
    "Temporal window: %d frames",
    history_size_
  );

  RCLCPP_INFO(
    get_logger(),
    "State analysis: %s",
    state_analysis_topic_.c_str()
  );

  RCLCPP_INFO(
    get_logger(),
    "Posture: %s",
    posture_topic_.c_str()
  );

  RCLCPP_INFO(
    get_logger(),
    "===================================================="
  );
}


// ============================================================================
// CALLBACK
// ============================================================================

void OperatorCondition::operator_perception_callback(
  const colab_supervision_interfaces::msg::OperatorPerception::SharedPtr msg)
{
  process_operator_state(*msg);
}


// ============================================================================
// PROCESSING
// ============================================================================

void OperatorCondition::process_operator_state(
  const colab_supervision_interfaces::msg::OperatorPerception & msg)
{
  OperatorSample sample;

  sample.person_detected =
    msg.person_detected;

  sample.person_visible =
    msg.person_visible;

  sample.attention_score =
    std::clamp(msg.attention_score, 0.0F, 1.0F);

  sample.attention_target =
    msg.attention_target.empty()
      ? "unknown"
      : msg.attention_target;

  sample.fatigue_score =
    std::clamp(msg.fatigue_score, 0.0F, 1.0F);

  sample.fatigue_level =
    msg.fatigue_level.empty()
      ? "UNKNOWN"
      : msg.fatigue_level;

  sample.head_yaw =
    msg.head_yaw;

  sample.head_pitch =
    msg.head_pitch;

  sample.head_roll =
    msg.head_roll;

  sample.head_nodding_score =
    std::clamp(msg.head_nodding_score, 0.0F, 1.0F);

  push_sample(sample);

  const bool presence =
    evaluate_presence();

  const AggregatedCondition attention =
    evaluate_attention();

  const AggregatedCondition fatigue =
    evaluate_fatigue();

  const AggregatedCondition posture =
    evaluate_posture();

  publish_conditions(
    presence,
    attention,
    fatigue,
    posture
  );

  initialized_ = true;
}


// ============================================================================
// HISTORY
// ============================================================================

void OperatorCondition::push_sample(
  const OperatorSample & sample)
{
  history_.push_back(sample);

  while (
    history_.size() >
    static_cast<std::size_t>(history_size_)
  ) {
    history_.pop_front();
  }
}


// ============================================================================
// PRESENCE
//
// The output is true only after the complete 5-frame window is available
// and all frames indicate both detection and visibility.
// ============================================================================

bool OperatorCondition::evaluate_presence() const
{
  if (
    history_.size() <
    static_cast<std::size_t>(history_size_)
  ) {
    return false;
  }

  for (const auto & sample : history_) {

    if (
      !sample.person_detected ||
      !sample.person_visible
    ) {
      return false;
    }
  }

  return true;
}


// ============================================================================
// ATTENTION
//
// The predominant target is calculated over the complete temporal window.
// Canonical output:
//   TASK / HMI / OTHER / UNKNOWN
//
// The score is the arithmetic mean of attention_score.
// ============================================================================

OperatorCondition::AggregatedCondition
OperatorCondition::evaluate_attention() const
{
  AggregatedCondition result;

  if (
    history_.size() <
    static_cast<std::size_t>(history_size_)
  ) {
    return result;
  }

  std::vector<std::string> targets;

  float score_sum =
    0.0F;

  int score_count =
    0;

  for (const auto & sample : history_) {

    if (
      !sample.person_detected ||
      !sample.person_visible
    ) {
      continue;
    }

    std::string target =
      sample.attention_target;

    std::transform(
      target.begin(),
      target.end(),
      target.begin(),
      [](unsigned char c) {
        return static_cast<char>(
          std::tolower(c)
        );
      }
    );

    if (target == "task") {
      targets.push_back("TASK");
    } else if (target == "hmi") {
      targets.push_back("HMI");
    } else if (target == "other") {
      targets.push_back("OTHER");
    } else {
      targets.push_back("UNKNOWN");
    }

    if (std::isfinite(sample.attention_score)) {
      score_sum += sample.attention_score;
      ++score_count;
    }
  }

  if (targets.empty()) {
    return result;
  }

  result.condition =
    majority_string(
      targets,
      "UNKNOWN"
    );

  if (score_count > 0) {

    result.score =
      std::clamp(
        score_sum /
        static_cast<float>(score_count),
        0.0F,
        1.0F
      );

    result.valid = true;
  }

  return result;
}


// ============================================================================
// FATIGUE
//
// The predominant fatigue level is calculated over the complete temporal
// window.
// Canonical output:
//   LOW / MEDIUM / HIGH / UNKNOWN
//
// The score is the arithmetic mean of fatigue_score.
// ============================================================================

OperatorCondition::AggregatedCondition
OperatorCondition::evaluate_fatigue() const
{
  AggregatedCondition result;

  if (
    history_.size() <
    static_cast<std::size_t>(history_size_)
  ) {
    return result;
  }

  std::vector<std::string> levels;

  float score_sum =
    0.0F;

  int score_count =
    0;

  for (const auto & sample : history_) {

    if (
      !sample.person_detected ||
      !sample.person_visible
    ) {
      continue;
    }

    std::string level =
      sample.fatigue_level;

    std::transform(
      level.begin(),
      level.end(),
      level.begin(),
      [](unsigned char c) {
        return static_cast<char>(
          std::toupper(c)
        );
      }
    );

    if (
      level == "LOW" ||
      level == "MEDIUM" ||
      level == "HIGH" ||
      level == "UNKNOWN"
    ) {
      levels.push_back(level);
    } else {
      levels.push_back("UNKNOWN");
    }

    if (std::isfinite(sample.fatigue_score)) {
      score_sum += sample.fatigue_score;
      ++score_count;
    }
  }

  if (levels.empty()) {
    return result;
  }

  result.condition =
    majority_string(
      levels,
      "UNKNOWN"
    );

  if (score_count > 0) {

    result.score =
      std::clamp(
        score_sum /
        static_cast<float>(score_count),
        0.0F,
        1.0F
      );

    result.valid = true;
  }

  return result;
}


// ============================================================================
// POSTURE
//
// Each frame is classified as:
//   POSTURE_OK
//   HEAD_DEVIATED
//   NODDING
//   UNKNOWN
//
// The final condition is the predominant condition over the complete window.
// The posture score is the arithmetic mean over valid frames.
// ============================================================================

OperatorCondition::AggregatedCondition
OperatorCondition::evaluate_posture() const
{
  AggregatedCondition result;

  if (
    history_.size() <
    static_cast<std::size_t>(history_size_)
  ) {
    return result;
  }

  std::vector<std::string> conditions;

  float score_sum =
    0.0F;

  int score_count =
    0;

  for (const auto & sample : history_) {

    if (!posture_sample_valid(sample)) {
      conditions.push_back("UNKNOWN");
      continue;
    }

    conditions.push_back(
      posture_sample_condition(sample)
    );

    score_sum +=
      posture_sample_score(sample);

    ++score_count;
  }

  if (conditions.empty()) {
    return result;
  }

  result.condition =
    majority_string(
      conditions,
      "UNKNOWN"
    );

  if (score_count > 0) {

    result.score =
      std::clamp(
        score_sum /
        static_cast<float>(score_count),
        0.0F,
        1.0F
      );

    result.valid = true;
  }

  return result;
}


// ============================================================================
// MAJORITY
//
// In case of a tie, UNKNOWN is returned.
// ============================================================================

std::string OperatorCondition::majority_string(
  const std::vector<std::string> & values,
  const std::string & default_value) const
{
  if (values.empty()) {
    return default_value;
  }

  std::map<std::string, int> counts;

  for (const auto & value : values) {
    ++counts[value];
  }

  int maximum =
    0;

  for (const auto & [value, count] : counts) {
    maximum =
      std::max(
        maximum,
        count
      );
  }

  std::vector<std::string> winners;

  for (const auto & [value, count] : counts) {

    if (count == maximum) {
      winners.push_back(value);
    }
  }

  if (winners.size() != 1) {
    return default_value;
  }

  return winners.front();
}


// ============================================================================
// POSTURE VALIDATION
// ============================================================================

bool OperatorCondition::posture_sample_valid(
  const OperatorSample & sample) const
{
  return (
    sample.person_detected &&
    sample.person_visible &&
    std::isfinite(sample.head_yaw) &&
    std::isfinite(sample.head_pitch) &&
    std::isfinite(sample.head_roll) &&
    std::isfinite(sample.head_nodding_score)
  );
}


// ============================================================================
// POSTURE SCORE
// ============================================================================

float OperatorCondition::posture_sample_score(
  const OperatorSample & sample) const
{
  const float yaw_ratio =
    std::min(
      1.0F,
      std::abs(sample.head_yaw) /
      posture_max_yaw_rad_
    );

  const float pitch_ratio =
    std::min(
      1.0F,
      std::abs(sample.head_pitch) /
      posture_max_pitch_rad_
    );

  const float roll_ratio =
    std::min(
      1.0F,
      std::abs(sample.head_roll) /
      posture_max_roll_rad_
    );

  const float nodding_ratio =
    std::min(
      1.0F,
      sample.head_nodding_score /
      posture_max_nodding_score_
    );

  const float maximum_deviation =
    std::max(
      std::max(
        yaw_ratio,
        pitch_ratio
      ),
      std::max(
        roll_ratio,
        nodding_ratio
      )
    );

  return std::clamp(
    1.0F - maximum_deviation,
    0.0F,
    1.0F
  );
}


// ============================================================================
// POSTURE CONDITION
// ============================================================================

std::string OperatorCondition::posture_sample_condition(
  const OperatorSample & sample) const
{
  if (
    sample.head_nodding_score >
    posture_max_nodding_score_
  ) {
    return "NODDING";
  }

  if (
    std::abs(sample.head_yaw) >
      posture_max_yaw_rad_ ||
    std::abs(sample.head_pitch) >
      posture_max_pitch_rad_ ||
    std::abs(sample.head_roll) >
      posture_max_roll_rad_
  ) {
    return "HEAD_DEVIATED";
  }

  return "POSTURE_OK";
}


// ============================================================================
// PUBLISH
// ============================================================================

void OperatorCondition::publish_conditions(
  bool presence,
  const AggregatedCondition & attention,
  const AggregatedCondition & fatigue,
  const AggregatedCondition & posture)
{
  // ==========================================================================
  // UNIFIED OPERATOR STATE ANALYSIS
  // ==========================================================================

  colab_supervision_interfaces::msg::OperatorStateAnalysis state_msg;

  state_msg.operator_present =
    presence;

  state_msg.attention_condition =
    attention.condition;

  state_msg.attention_score =
    attention.score;

  state_msg.fatigue_condition =
    fatigue.condition;

  state_msg.fatigue_score =
    fatigue.score;

  state_analysis_pub_->publish(
    state_msg
  );

  // ==========================================================================
  // POSTURE
  // ==========================================================================

  colab_supervision_interfaces::msg::OperatorPosture posture_msg;

  posture_msg.posture_condition =
    posture.condition;

  posture_msg.posture_score =
    posture.score;

  posture_pub_->publish(
    posture_msg
  );

  // ==========================================================================
  // LOG
  // ==========================================================================

  if (
    !initialized_ ||
    !presence ||
    attention.condition == "UNKNOWN" ||
    fatigue.condition == "UNKNOWN" ||
    posture.condition == "UNKNOWN"
  ) {

    RCLCPP_INFO(
      get_logger(),

      "Presence=%s | "
      "Attention=%s (%.2f) | "
      "Fatigue=%s (%.2f) | "
      "Posture=%s (%.2f) | "
      "Window=%zu/%d",

      presence ? "TRUE" : "FALSE",

      attention.condition.c_str(),
      attention.score,

      fatigue.condition.c_str(),
      fatigue.score,

      posture.condition.c_str(),
      posture.score,

      history_.size(),
      history_size_
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
  rclcpp::init(
    argc,
    argv
  );

  try {

    auto node =
      std::make_shared<
        colab_supervision_control::OperatorCondition
      >();

    rclcpp::spin(
      node
    );

  } catch (
    const std::exception & error) {

    RCLCPP_FATAL(
      rclcpp::get_logger(
        "operator_condition_node"
      ),
      "Node initialization failed: %s",
      error.what()
    );
  }

  rclcpp::shutdown();

  return 0;
}
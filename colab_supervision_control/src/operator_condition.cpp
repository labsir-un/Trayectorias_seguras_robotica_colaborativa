#include "colab_supervision_control/operator_condition.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <limits>
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
  // TOPICS DE ENTRADA
  // ==========================================================================

  perception_topic_ = declare_parameter<std::string>(
    "perception_topic",
    "/state_operator"
  );

  // ==========================================================================
  // TOPICS DE SALIDA
  // ==========================================================================

  presence_topic_ = declare_parameter<std::string>(
    "presence_topic",
    "/operator/presence"
  );

  attention_topic_ = declare_parameter<std::string>(
    "attention_topic",
    "/operator/attention"
  );

  attention_score_topic_ = declare_parameter<std::string>(
    "attention_score_topic",
    "/operator/attention_score"
  );

  fatigue_topic_ = declare_parameter<std::string>(
    "fatigue_topic",
    "/operator/fatigue"
  );

  fatigue_score_topic_ = declare_parameter<std::string>(
    "fatigue_score_topic",
    "/operator/fatigue_score"
  );

  posture_topic_ = declare_parameter<std::string>(
    "posture_topic",
    "/operator/posture"
  );

  posture_score_topic_ = declare_parameter<std::string>(
    "posture_score_topic",
    "/operator/posture_score"
  );

  // ==========================================================================
  // VENTANA TEMPORAL
  // ==========================================================================

  history_size_ = declare_parameter<int>(
    "history_size",
    5
  );

  // ==========================================================================
  // UMBRALES DE POSTURA
  //
  // Los valores por defecto representan una condición configurable de
  // orientación de cabeza. Los ángulos del mensaje están expresados en
  // radianes.
  // ==========================================================================

  constexpr float kDefaultPostureAngleRad =
    0.436332312F;  // 25 degrees

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
    history_size_ = 5;
    RCLCPP_WARN(
      get_logger(),
      "history_size invalido. Se utiliza 5."
    );
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

  presence_pub_ =
    create_publisher<std_msgs::msg::Bool>(
      presence_topic_,
      10
    );

  attention_pub_ =
    create_publisher<std_msgs::msg::String>(
      attention_topic_,
      10
    );

  attention_score_pub_ =
    create_publisher<std_msgs::msg::Float32>(
      attention_score_topic_,
      10
    );

  fatigue_pub_ =
    create_publisher<std_msgs::msg::String>(
      fatigue_topic_,
      10
    );

  fatigue_score_pub_ =
    create_publisher<std_msgs::msg::Float32>(
      fatigue_score_topic_,
      10
    );

  posture_pub_ =
    create_publisher<std_msgs::msg::String>(
      posture_topic_,
      10
    );

  posture_score_pub_ =
    create_publisher<std_msgs::msg::Float32>(
      posture_score_topic_,
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
    "Outputs:"
  );

  RCLCPP_INFO(
    get_logger(),
    "  Presence       : %s",
    presence_topic_.c_str()
  );

  RCLCPP_INFO(
    get_logger(),
    "  Attention      : %s",
    attention_topic_.c_str()
  );

  RCLCPP_INFO(
    get_logger(),
    "  Attention score: %s",
    attention_score_topic_.c_str()
  );

  RCLCPP_INFO(
    get_logger(),
    "  Fatigue        : %s",
    fatigue_topic_.c_str()
  );

  RCLCPP_INFO(
    get_logger(),
    "  Fatigue score  : %s",
    fatigue_score_topic_.c_str()
  );

  RCLCPP_INFO(
    get_logger(),
    "  Posture        : %s",
    posture_topic_.c_str()
  );

  RCLCPP_INFO(
    get_logger(),
    "  Posture score  : %s",
    posture_score_topic_.c_str()
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
// PROCESAMIENTO
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
    std::clamp(
      msg.attention_score,
      0.0F,
      1.0F
    );

  sample.attention_target =
    msg.attention_target.empty()
      ? "unknown"
      : msg.attention_target;

  sample.fatigue_score =
    std::clamp(
      msg.fatigue_score,
      0.0F,
      1.0F
    );

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
    std::clamp(
      msg.head_nodding_score,
      0.0F,
      1.0F
    );

  push_sample(sample);

  // ==========================================================================
  // EVALUACION DE LAS CONDICIONES
  // ==========================================================================

  const bool presence =
    evaluate_presence();

  const AggregatedCondition attention =
    evaluate_attention();

  const AggregatedCondition fatigue =
    evaluate_fatigue();

  const AggregatedCondition posture =
    evaluate_posture();

  // ==========================================================================
  // PUBLICACION
  // ==========================================================================

  publish_conditions(
    presence,
    attention,
    fatigue,
    posture
  );

  initialized_ =
    true;
}


// ============================================================================
// HISTORIAL
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
// PRESENCIA
//
// La condición se considera estable cuando todos los frames de la ventana
// contienen una detección y una observación facial visible.
//
// Mientras la ventana no esté completa, se publica false.
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
// ATENCION
//
// Se obtiene el objetivo predominante de los últimos frames.
//
// Valores esperados:
//   task
//   HMI
//   other
//   unknown
//
// Si hay empate, se conserva UNKNOWN para no sobredeterminar la condición.
// La puntuación es el promedio de los scores válidos disponibles.
// ============================================================================

OperatorCondition::AggregatedCondition
OperatorCondition::evaluate_attention() const
{
  AggregatedCondition result;

  if (history_.empty()) {
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
        return static_cast<char>(std::tolower(c));
      }
    );

    if (
      target == "task" ||
      target == "hmi" ||
      target == "other" ||
      target == "unknown"
    ) {
      targets.push_back(target);
    } else {
      targets.push_back("unknown");
    }

    if (
      std::isfinite(sample.attention_score)
    ) {
      score_sum +=
        sample.attention_score;

      ++score_count;
    }
  }

  if (targets.empty()) {
    return result;
  }

  result.condition =
    majority_string(
      targets,
      "unknown"
    );

  if (score_count > 0) {

    result.score =
      std::clamp(
        score_sum /
        static_cast<float>(score_count),
        0.0F,
        1.0F
      );

    result.valid =
      true;
  }

  return result;
}


// ============================================================================
// FATIGA
//
// Se utiliza una votación de los últimos frames.
//
// Ejemplo:
//   LOW LOW MEDIUM LOW MEDIUM -> LOW
//
// Si no existe una mayoría clara, se publica UNKNOWN.
// La puntuación es el promedio del fatigue_score.
// ============================================================================

OperatorCondition::AggregatedCondition
OperatorCondition::evaluate_fatigue() const
{
  AggregatedCondition result;

  if (history_.empty()) {
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
        return static_cast<char>(std::toupper(c));
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

    if (
      std::isfinite(sample.fatigue_score)
    ) {
      score_sum +=
        sample.fatigue_score;

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

    result.valid =
      true;
  }

  return result;
}


// ============================================================================
// POSTURA
//
// La postura se evalúa a partir de la orientación de cabeza y el indicador
// de cabeceo:
//
//   POSTURE_OK
//   HEAD_DEVIATED
//   NODDING
//   UNKNOWN
//
// El score representa una medida de calidad de postura [0,1]:
//   1.0 -> dentro de los límites configurados
//   0.0 -> desviación máxima según los límites configurados
// ============================================================================

OperatorCondition::AggregatedCondition
OperatorCondition::evaluate_posture() const
{
  AggregatedCondition result;

  if (history_.empty()) {
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

    result.valid =
      true;
  }

  return result;
}


// ============================================================================
// MAYORIA
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
// VALIDACION DE POSTURA
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
// SCORE DE POSTURA
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

  // Se utiliza la peor desviación como indicador de calidad de postura.
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
// CONDICION DE POSTURA POR FRAME
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
// PUBLICACION
// ============================================================================

void OperatorCondition::publish_conditions(
  bool presence,
  const AggregatedCondition & attention,
  const AggregatedCondition & fatigue,
  const AggregatedCondition & posture)
{
  // ==========================================================================
  // PRESENCIA
  // ==========================================================================

  std_msgs::msg::Bool presence_msg;

  presence_msg.data =
    presence;

  presence_pub_->publish(
    presence_msg
  );

  // ==========================================================================
  // ATENCION
  // ==========================================================================

  std_msgs::msg::String attention_msg;

  attention_msg.data =
    attention.condition;

  attention_pub_->publish(
    attention_msg
  );

  std_msgs::msg::Float32 attention_score_msg;

  attention_score_msg.data =
    attention.score;

  attention_score_pub_->publish(
    attention_score_msg
  );

  // ==========================================================================
  // FATIGA
  // ==========================================================================

  std_msgs::msg::String fatigue_msg;

  fatigue_msg.data =
    fatigue.condition;

  fatigue_pub_->publish(
    fatigue_msg
  );

  std_msgs::msg::Float32 fatigue_score_msg;

  fatigue_score_msg.data =
    fatigue.score;

  fatigue_score_pub_->publish(
    fatigue_score_msg
  );

  // ==========================================================================
  // POSTURA
  // ==========================================================================

  std_msgs::msg::String posture_msg;

  posture_msg.data =
    posture.condition;

  posture_pub_->publish(
    posture_msg
  );

  std_msgs::msg::Float32 posture_score_msg;

  posture_score_msg.data =
    posture.score;

  posture_score_pub_->publish(
    posture_score_msg
  );

  // ==========================================================================
  // LOG
  // ==========================================================================

  if (
    !initialized_ ||
    !presence ||
    attention.condition == "unknown" ||
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
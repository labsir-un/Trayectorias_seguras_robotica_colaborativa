#include "colab_supervision_control/control_decision.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
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
  // ==========================================================================
  // TOPICS DE ENTRADA
  //
  // Este nodo NO recibe información de cámaras ni datos de percepción de bajo
  // nivel. Las entradas corresponden exclusivamente a estados procesados.
  // ==========================================================================

  operator_state_topic_ = declare_parameter<std::string>(
    "operator_state_topic",
    "/operator/state_analisis"
  );

  interaction_topic_ = declare_parameter<std::string>(
    "interaction_topic",
    "/robot_operator_interaction"
  );

  trajectory_topic_ = declare_parameter<std::string>(
    "trajectory_topic",
    "/control/trajectory"
  );

  // ==========================================================================
  // TOPIC DE SALIDA
  // ==========================================================================

  command_topic_ = declare_parameter<std::string>(
    "command_topic",
    "/control/command"
  );

  // ==========================================================================
  // TEMPORIZACIÓN
  // ==========================================================================

  decision_rate_hz_ = declare_parameter<double>(
    "decision_rate_hz",
    10.0
  );

  max_input_age_sec_ = declare_parameter<double>(
    "max_input_age_sec",
    0.50
  );

  max_trajectory_age_sec_ = declare_parameter<double>(
    "max_trajectory_age_sec",
    2.0
  );

  // ==========================================================================
  // ZONAS
  //
  // Z3: d <  0.250 m
  // Z4: 0.250 m <= d < 0.400 m
  // Z5: 0.400 m <= d <= 1.600 m
  // Z6: d > 1.600 m
  //
  // La distancia ya fue calculada por robot_operator_interaction.
  // ==========================================================================

  zone3_max_distance_ = declare_parameter<double>(
    "zone3_max_distance",
    0.250
  );

  zone4_max_distance_ = declare_parameter<double>(
    "zone4_max_distance",
    0.400
  );

  zone5_max_distance_ = declare_parameter<double>(
    "zone5_max_distance",
    1.600
  );

  // ==========================================================================
  // PERFIL DE VELOCIDAD
  // ==========================================================================

  full_speed_distance_ = declare_parameter<double>(
    "full_speed_distance",
    1.900
  );

  half_speed_distance_ = declare_parameter<double>(
    "half_speed_distance",
    1.600
  );

  minimum_speed_distance_ = declare_parameter<double>(
    "minimum_speed_distance",
    0.400
  );

  full_speed_percent_ = declare_parameter<double>(
    "full_speed_percent",
    100.0
  );

  half_speed_percent_ = declare_parameter<double>(
    "half_speed_percent",
    50.0
  );

  minimum_speed_percent_ = declare_parameter<double>(
    "minimum_speed_percent",
    5.0
  );

  // ==========================================================================
  // CONDICIONES ANÓMALAS DEL OPERADOR
  // ==========================================================================

  pause_on_fatigue_high_ = declare_parameter<bool>(
    "pause_on_fatigue_high",
    true
  );

  pause_on_attention_other_ = declare_parameter<bool>(
    "pause_on_attention_other",
    true
  );

  pause_on_attention_unknown_ = declare_parameter<bool>(
    "pause_on_attention_unknown",
    true
  );

  // ==========================================================================
  // EVALUACIÓN DE TRAYECTORIA
  // ==========================================================================

  conservative_trajectory_check_ = declare_parameter<bool>(
    "conservative_trajectory_check",
    true
  );

  trajectory_min_separation_distance_ = declare_parameter<double>(
    "trajectory_min_separation_distance",
    0.400
  );

  // ==========================================================================
  // VALIDACIÓN DE PARÁMETROS
  // ==========================================================================

  if (decision_rate_hz_ <= 0.0) {
    throw std::invalid_argument(
      "decision_rate_hz must be greater than zero"
    );
  }

  if (max_input_age_sec_ <= 0.0) {
    throw std::invalid_argument(
      "max_input_age_sec must be greater than zero"
    );
  }

  if (max_trajectory_age_sec_ <= 0.0) {
    throw std::invalid_argument(
      "max_trajectory_age_sec must be greater than zero"
    );
  }

  if (zone3_max_distance_ < 0.0) {
    throw std::invalid_argument(
      "zone3_max_distance must be non-negative"
    );
  }

  if (zone4_max_distance_ <= zone3_max_distance_) {
    throw std::invalid_argument(
      "zone4_max_distance must be greater than zone3_max_distance"
    );
  }

  if (zone5_max_distance_ < zone4_max_distance_) {
    throw std::invalid_argument(
      "zone5_max_distance must be greater than or equal to "
      "zone4_max_distance"
    );
  }

  if (full_speed_distance_ <= half_speed_distance_) {
    throw std::invalid_argument(
      "full_speed_distance must be greater than half_speed_distance"
    );
  }

  if (half_speed_distance_ <= minimum_speed_distance_) {
    throw std::invalid_argument(
      "half_speed_distance must be greater than minimum_speed_distance"
    );
  }

  if (
    full_speed_percent_ < 0.0 ||
    half_speed_percent_ < 0.0 ||
    minimum_speed_percent_ < 0.0
  ) {
    throw std::invalid_argument(
      "Speed percentages must be non-negative"
    );
  }

  if (trajectory_min_separation_distance_ < 0.0) {
    throw std::invalid_argument(
      "trajectory_min_separation_distance must be non-negative"
    );
  }

  // ==========================================================================
  // SUBSCRIBER: ESTADO PROCESADO DEL OPERADOR
  // ==========================================================================

  operator_state_sub_ =
    create_subscription<
      colab_supervision_interfaces::msg::OperatorStateAnalysis
    >(
      operator_state_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
        &ControlDecision::operator_state_callback,
        this,
        std::placeholders::_1
      )
    );

  // ==========================================================================
  // SUBSCRIBER: INTERACCIÓN PROCESADA
  // ==========================================================================

  interaction_sub_ =
    create_subscription<
      colab_supervision_interfaces::msg::RobotOperatorInteraction
    >(
      interaction_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
        &ControlDecision::interaction_callback,
        this,
        std::placeholders::_1
      )
    );

  // ==========================================================================
  // SUBSCRIBER: TRAYECTORIA DE MOVEIT
  // ==========================================================================

  trajectory_sub_ =
    create_subscription<
      moveit_msgs::msg::RobotTrajectory
    >(
      trajectory_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
        &ControlDecision::trajectory_callback,
        this,
        std::placeholders::_1
      )
    );

  // ==========================================================================
  // PUBLISHER: COMANDO DE CONTROL
  // ==========================================================================

  command_pub_ =
    create_publisher<
      colab_supervision_interfaces::msg::ControlCommand
    >(
      command_topic_,
      10
    );

  // ==========================================================================
  // TIMER
  // ==========================================================================

  const auto period =
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(
        1.0 / decision_rate_hz_
      )
    );

  decision_timer_ =
    create_wall_timer(
      period,
      std::bind(
        &ControlDecision::decision_timer_callback,
        this
      )
    );

  // ==========================================================================
  // LOG INICIAL
  // ==========================================================================

  RCLCPP_INFO(
    get_logger(),
    "===================================================="
  );

  RCLCPP_INFO(
    get_logger(),
    "CONTROL DECISION NODE"
  );

  RCLCPP_INFO(
    get_logger(),
    "Processed operator state: %s",
    operator_state_topic_.c_str()
  );

  RCLCPP_INFO(
    get_logger(),
    "Processed interaction: %s",
    interaction_topic_.c_str()
  );

  RCLCPP_INFO(
    get_logger(),
    "MoveIt trajectory: %s",
    trajectory_topic_.c_str()
  );

  RCLCPP_INFO(
    get_logger(),
    "Control command: %s",
    command_topic_.c_str()
  );

  RCLCPP_INFO(
    get_logger(),
    "Zones: Z3 < %.3f m | Z4 < %.3f m | Z5 <= %.3f m | Z6 > %.3f m",
    zone3_max_distance_,
    zone4_max_distance_,
    zone5_max_distance_,
    zone5_max_distance_
  );

  RCLCPP_INFO(
    get_logger(),
    "No raw camera information is consumed by this node"
  );

  RCLCPP_INFO(
    get_logger(),
    "===================================================="
  );
}


// ============================================================================
// CALLBACK: ESTADO DEL OPERADOR
// ============================================================================

void ControlDecision::operator_state_callback(
  const colab_supervision_interfaces::msg::OperatorStateAnalysis::SharedPtr msg)
{
  latest_operator_state_ = *msg;
  operator_state_received_time_ = get_clock()->now();
}


// ============================================================================
// CALLBACK: INTERACCIÓN
// ============================================================================

void ControlDecision::interaction_callback(
  const colab_supervision_interfaces::msg::RobotOperatorInteraction::SharedPtr msg)
{
  latest_interaction_ = *msg;
  interaction_received_time_ = get_clock()->now();

  // Se conserva únicamente la última distancia que ya fue calculada y
  // validada por robot_operator_interaction.
  if (
    msg->interaction_valid &&
    std::isfinite(msg->distance) &&
    msg->distance >= 0.0
  ) {
    last_valid_distance_ =
      static_cast<double>(msg->distance);

    last_valid_distance_time_ =
      interaction_received_time_;
  }
}


// ============================================================================
// CALLBACK: TRAYECTORIA
// ============================================================================

void ControlDecision::trajectory_callback(
  const moveit_msgs::msg::RobotTrajectory::SharedPtr msg)
{
  latest_trajectory_ = *msg;
  trajectory_received_time_ = get_clock()->now();
}


// ============================================================================
// TIMER
// ============================================================================

void ControlDecision::decision_timer_callback()
{
  const DecisionResult result =
    evaluate_decision();

  publish_command(result);

  initialized_ = true;
}


// ============================================================================
// FRESHNESS
// ============================================================================

bool ControlDecision::is_operator_state_fresh() const
{
  if (!latest_operator_state_.has_value()) {
    return false;
  }

  return (
    get_clock()->now() -
    operator_state_received_time_
  ).seconds() <= max_input_age_sec_;
}


bool ControlDecision::is_interaction_fresh() const
{
  if (!latest_interaction_.has_value()) {
    return false;
  }

  return (
    get_clock()->now() -
    interaction_received_time_
  ).seconds() <= max_input_age_sec_;
}


bool ControlDecision::is_trajectory_fresh() const
{
  if (!latest_trajectory_.has_value()) {
    return false;
  }

  return (
    get_clock()->now() -
    trajectory_received_time_
  ).seconds() <= max_trajectory_age_sec_;
}


bool ControlDecision::detection_information_available() const
{
  // La decisión solamente confía en los dos estados que provienen de los
  // nodos de preprocesamiento/supervisión.
  return (
    is_operator_state_fresh() &&
    is_interaction_fresh()
  );
}


// ============================================================================
// CONDICIÓN ANÓMALA DEL OPERADOR
// ============================================================================

bool ControlDecision::operator_condition_anomalous() const
{
  if (!latest_operator_state_.has_value()) {
    return true;
  }

  const auto & state =
    *latest_operator_state_;

  if (
    pause_on_fatigue_high_ &&
    state.fatigue_condition == "HIGH"
  ) {
    return true;
  }

  if (
    pause_on_attention_other_ &&
    state.attention_condition == "OTHER"
  ) {
    return true;
  }

  if (
    pause_on_attention_unknown_ &&
    state.attention_condition == "UNKNOWN"
  ) {
    return true;
  }

  return false;
}


// ============================================================================
// RIESGO CRÍTICO
// ============================================================================

bool ControlDecision::collision_is_critical() const
{
  if (!latest_interaction_.has_value()) {
    return true;
  }

  const auto & interaction =
    *latest_interaction_;

  return (
    interaction.collision_imminent ||
    interaction.collision_risk == "CRITICAL"
  );
}


// ============================================================================
// DETERMINACIÓN DE ZONA
// ============================================================================

ControlDecision::Zone
ControlDecision::determine_zone(double distance) const
{
  if (
    !std::isfinite(distance) ||
    distance < 0.0
  ) {
    return Zone::UNKNOWN;
  }

  // Z3: d < 0.250 m
  if (distance < zone3_max_distance_) {
    return Zone::Z3;
  }

  // Z4: 0.250 <= d < 0.400 m
  if (distance < zone4_max_distance_) {
    return Zone::Z4;
  }

  // Z5: 0.400 <= d <= 1.600 m
  if (distance <= zone5_max_distance_) {
    return Zone::Z5;
  }

  // Z6: d > 1.600 m
  return Zone::Z6;
}


const char * ControlDecision::zone_to_string(
  Zone zone) const
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


// ============================================================================
// DISTANCIA
// ============================================================================

bool ControlDecision::get_current_or_last_valid_distance(
  double & distance) const
{
  // --------------------------------------------------------------------------
  // Distancia actual procesada
  // --------------------------------------------------------------------------

  if (latest_interaction_.has_value()) {

    const auto & interaction =
      *latest_interaction_;

    if (
      interaction.interaction_valid &&
      std::isfinite(interaction.distance) &&
      interaction.distance >= 0.0
    ) {
      distance =
        static_cast<double>(
          interaction.distance
        );

      return true;
    }
  }

  // --------------------------------------------------------------------------
  // Última distancia procesada válida
  //
  // No se recalcula ni se promedia.
  // --------------------------------------------------------------------------

  if (last_valid_distance_.has_value()) {

    const double age =
      (
        get_clock()->now() -
        last_valid_distance_time_
      ).seconds();

    if (age <= max_input_age_sec_) {

      distance =
        *last_valid_distance_;

      return true;
    }
  }

  return false;
}


// ============================================================================
// TRAYECTORIA
// ============================================================================

bool ControlDecision::trajectory_available() const
{
  return (
    latest_trajectory_.has_value() &&
    is_trajectory_fresh() &&
    !latest_trajectory_->joint_trajectory.points.empty()
  );
}


bool ControlDecision::trajectory_target_reachable() const
{
  // Una trayectoria fresca con puntos representa, con la información
  // actualmente disponible, un resultado de planificación válido de MoveIt.
  //
  // No se interpreta como una verificación geométrica completa de seguridad.
  return trajectory_available();
}


bool ControlDecision::trajectory_keeps_separation(
  Zone zone,
  double distance) const
{
  // --------------------------------------------------------------------------
  // Z6
  // --------------------------------------------------------------------------

  if (zone == Zone::Z6) {
    return true;
  }

  if (!conservative_trajectory_check_) {
    return false;
  }

  // Si la distancia actual ya es inferior a la separación mínima requerida,
  // la trayectoria no se considera ejecutable bajo esa separación.
  if (distance < trajectory_min_separation_distance_) {
    return false;
  }

  if (!latest_interaction_.has_value()) {
    return false;
  }

  const auto & interaction =
    *latest_interaction_;

  // El nodo de interacción ya clasificó el riesgo.
  if (
    interaction.collision_imminent ||
    interaction.collision_risk == "CRITICAL" ||
    interaction.collision_risk == "DANGEROUS"
  ) {
    return false;
  }

  return true;
}


// ============================================================================
// PERFIL DE VELOCIDAD
// ============================================================================

int ControlDecision::compute_speed_percent(
  double distance) const
{
  double speed;

  // >= 1.900 m -> 100 %
  if (distance >= full_speed_distance_) {

    speed =
      full_speed_percent_;
  }

  // 1.600 <= d < 1.900 -> 100 ... 50 %
  else if (distance >= half_speed_distance_) {

    const double normalized =
      (
        full_speed_distance_ -
        distance
      ) /
      (
        full_speed_distance_ -
        half_speed_distance_
      );

    const double ratio =
      half_speed_percent_ /
      full_speed_percent_;

    speed =
      full_speed_percent_ *
      std::pow(
        ratio,
        normalized
      );
  }

  // 0.400 <= d < 1.600 -> 50 ... 5 %
  else if (distance >= minimum_speed_distance_) {

    const double normalized =
      (
        half_speed_distance_ -
        distance
      ) /
      (
        half_speed_distance_ -
        minimum_speed_distance_
      );

    const double ratio =
      minimum_speed_percent_ /
      half_speed_percent_;

    speed =
      half_speed_percent_ *
      std::pow(
        ratio,
        normalized
      );
  }

  // d < 0.400 -> 5 %
  else {

    speed =
      minimum_speed_percent_;
  }

  speed =
    std::clamp(
      speed,
      minimum_speed_percent_,
      full_speed_percent_
    );

  return static_cast<int>(
    std::lround(speed)
  );
}


// ============================================================================
// COMANDOS
// ============================================================================

ControlDecision::DecisionResult
ControlDecision::make_emergency_stop(
  const std::string & reason) const
{
  return {
    "EMERGENCY_STOP",
    "ROBOT",
    "STATE",
    "EMERGENCY_STOP",
    reason
  };
}


ControlDecision::DecisionResult
ControlDecision::make_safe_stop(
  const std::string & reason) const
{
  return {
    "STOP",
    "ROBOT",
    "STATE",
    "SAFE_STOP",
    reason
  };
}


ControlDecision::DecisionResult
ControlDecision::make_pause(
  const std::string & reason) const
{
  return {
    "PAUSE",
    "ROBOT",
    "STATE",
    "PAUSED",
    reason
  };
}


ControlDecision::DecisionResult
ControlDecision::make_increase_distance(
  int speed_percent,
  const std::string & reason) const
{
  return {
    "INCREASE_DISTANCE",
    "ROBOT",
    "SPEED",
    std::to_string(speed_percent),
    reason
  };
}


ControlDecision::DecisionResult
ControlDecision::make_limit_speed(
  int speed_percent,
  const std::string & reason) const
{
  return {
    "SET_SPEED",
    "ROBOT",
    "SPEED",
    std::to_string(speed_percent),
    reason
  };
}


ControlDecision::DecisionResult
ControlDecision::make_normal_operation(
  int speed_percent,
  const std::string & reason) const
{
  return {
    "EXECUTE",
    "TRAJECTORY",
    "SPEED",
    std::to_string(speed_percent),
    reason
  };
}


// ============================================================================
// JERARQUÍA PRINCIPAL DE DECISIÓN
// ============================================================================

ControlDecision::DecisionResult
ControlDecision::evaluate_decision()
{
  // ==========================================================================
  // PRIORIDAD 1
  //
  // Indisponibilidad de las salidas procesadas de los nodos de percepción.
  // ==========================================================================

  if (!detection_information_available()) {

    return make_emergency_stop(
      "PRIORITY 1: processed detection information unavailable"
    );
  }

  const auto & operator_state =
    *latest_operator_state_;

  const auto & interaction =
    *latest_interaction_;

  const bool operator_present =
    operator_state.operator_present;

  // ==========================================================================
  // DISTANCIA
  // ==========================================================================

  double distance = 0.0;

  const bool distance_available =
    get_current_or_last_valid_distance(
      distance
    );

  // ==========================================================================
  // OPERADOR NO PRESENTE
  //
  // operator_state ya contiene la condición procesada temporalmente.
  // No se realiza una nueva evaluación de los frames.
  // ==========================================================================

  if (!operator_present) {

    if (distance_available) {

      const Zone zone =
        determine_zone(distance);

      // Matriz:
      // Z6 + operador no disponible -> pausa controlada
      if (zone == Zone::Z6) {

        return make_pause(
          "PRIORITY 3: operator unavailable in Z6"
        );
      }
    }

    // Si el operador no está presente y no existe una nueva distancia válida,
    // se evita interpretar esto como una falla de detección.
    return make_pause(
      "PRIORITY 3: operator unavailable; waiting for valid collaborative state"
    );
  }

  // ==========================================================================
  // OPERADOR PRESENTE -> DISTANCIA NECESARIA
  // ==========================================================================

  if (
    !distance_available ||
    !interaction.interaction_valid
  ) {

    return make_emergency_stop(
      "PRIORITY 1: valid processed operator-robot distance unavailable"
    );
  }

  const Zone zone =
    determine_zone(distance);

  if (zone == Zone::UNKNOWN) {

    return make_emergency_stop(
      "PRIORITY 1: operator-robot zone cannot be determined"
    );
  }

  // ==========================================================================
  // PRIORIDAD 2
  //
  // Z3 + operador detectado -> detención segura
  // ==========================================================================

  if (zone == Zone::Z3) {

    return make_safe_stop(
      "PRIORITY 2: operator detected in Z3"
    );
  }

  // ==========================================================================
  // PRIORIDAD 2
  //
  // Z4/Z5 + riesgo crítico -> detención segura
  // ==========================================================================

  if (
    (zone == Zone::Z4 || zone == Zone::Z5) &&
    collision_is_critical()
  ) {

    return make_safe_stop(
      "PRIORITY 2: critical collision risk in Z4/Z5"
    );
  }

  // ==========================================================================
  // PRIORIDAD 3
  //
  // Fatiga elevada o condición anómala
  // ==========================================================================

  if (operator_condition_anomalous()) {

    return make_pause(
      "PRIORITY 3: elevated fatigue or anomalous operator condition"
    );
  }

  // ==========================================================================
  // PRIORIDAD 3
  //
  // Objetivo no alcanzable / trayectoria no disponible
  // ==========================================================================

  const bool target_reachable =
    trajectory_target_reachable();

  if (!target_reachable) {

    if (
      zone == Zone::Z4 ||
      zone == Zone::Z5
    ) {

      return make_pause(
        "PRIORITY 3: target point not reachable under current safety conditions"
      );
    }

    return make_pause(
      "PRIORITY 3: trajectory/target unavailable"
    );
  }

  // ==========================================================================
  // PRIORIDAD 4
  //
  // Z4/Z5 + trayectoria que no puede mantener separación
  //
  // -> aumentar distancia + reducir velocidad
  // ==========================================================================

  if (
    zone == Zone::Z4 ||
    zone == Zone::Z5
  ) {

    const bool maintains_separation =
      trajectory_keeps_separation(
        zone,
        distance
      );

    if (!maintains_separation) {

      const int speed =
        compute_speed_percent(
          distance
        );

      return make_increase_distance(
        speed,
        "PRIORITY 4: trajectory cannot be executed while maintaining "
        "the required separation"
      );
    }
  }

  // ==========================================================================
  // PRIORIDAD 5
  //
  // Z4/Z5 + trayectoria segura
  //
  // -> reducción de velocidad
  // ==========================================================================

  if (
    zone == Zone::Z4 ||
    zone == Zone::Z5
  ) {

    const int speed =
      compute_speed_percent(
        distance
      );

    return make_limit_speed(
      speed,
      "PRIORITY 5: Z4/Z5 trajectory can execute while maintaining separation"
    );
  }

  // ==========================================================================
  // PRIORIDAD 6
  //
  // Z6 + operador presente + sin condición anómala
  //
  // -> operación normal
  // ==========================================================================

  if (zone == Zone::Z6) {

    const int speed =
      compute_speed_percent(
        distance
      );

    return make_normal_operation(
      speed,
      "PRIORITY 6: Z6 operation without anomalous condition"
    );
  }

  // ==========================================================================
  // FAIL-SAFE
  // ==========================================================================

  return make_pause(
    "Fail-safe fallback: no valid decision rule matched"
  );
}


// ============================================================================
// PUBLICACIÓN
// ============================================================================

void ControlDecision::publish_command(
  const DecisionResult & result)
{
  colab_supervision_interfaces::msg::ControlCommand msg;

  msg.command =
    result.command;

  msg.object =
    result.object;

  msg.variable =
    result.variable;

  msg.value =
    result.value;

  // --------------------------------------------------------------------------
  // Firma para diagnóstico: permite saber cuándo cambia realmente la decisión.
  // --------------------------------------------------------------------------

  std::ostringstream signature_stream;

  signature_stream
    << result.command << "|"
    << result.object << "|"
    << result.variable << "|"
    << result.value;

  const std::string signature =
    signature_stream.str();

  if (
    !initialized_ ||
    signature != last_published_signature_
  ) {

    RCLCPP_INFO(
      get_logger(),
      "COMMAND: %s | object=%s | variable=%s | value=%s | reason=%s",
      result.command.c_str(),
      result.object.c_str(),
      result.variable.c_str(),
      result.value.c_str(),
      result.reason.c_str()
    );

    last_published_signature_ =
      signature;
  }

  command_pub_->publish(msg);
}

}  // namespace colab_supervision_control


// ============================================================================
// MAIN
// ============================================================================

int main(
  int argc,
  char * argv[])
{
  rclcpp::init(argc, argv);

  try {

    auto node =
      std::make_shared<
        colab_supervision_control::ControlDecision
      >();

    rclcpp::spin(node);

  } catch (
    const std::exception & error) {

    RCLCPP_FATAL(
      rclcpp::get_logger("control_decision"),
      "Node initialization failed: %s",
      error.what()
    );
  }

  rclcpp::shutdown();

  return 0;
}
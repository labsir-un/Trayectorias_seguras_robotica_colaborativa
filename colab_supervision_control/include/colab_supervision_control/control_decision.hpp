#ifndef COLAB_SUPERVISION_CONTROL__CONTROL_DECISION_HPP_
#define COLAB_SUPERVISION_CONTROL__CONTROL_DECISION_HPP_

#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <colab_supervision_interfaces/msg/control_command.hpp>
#include <colab_supervision_interfaces/msg/operator_state_analysis.hpp>
#include <colab_supervision_interfaces/msg/robot_operator_interaction.hpp>

#include <moveit_msgs/msg/robot_trajectory.hpp>

namespace colab_supervision_control
{

class ControlDecision : public rclcpp::Node
{
public:
  ControlDecision();
  ~ControlDecision() override = default;

private:
  // ===========================================================================
  // ZONAS DE OPERACIÓN
  // ===========================================================================

  enum class Zone
  {
    Z3,
    Z4,
    Z5,
    Z6,
    UNKNOWN
  };

  // ===========================================================================
  // RESULTADO DE DECISIÓN
  // ===========================================================================

  struct DecisionResult
  {
    std::string command;
    std::string object;
    std::string variable;
    std::string value;
    std::string reason;
  };

  // ===========================================================================
  // CALLBACKS DE ENTRADA
  // ===========================================================================

  void operator_state_callback(
    const colab_supervision_interfaces::msg::OperatorStateAnalysis::SharedPtr msg);

  void interaction_callback(
    const colab_supervision_interfaces::msg::RobotOperatorInteraction::SharedPtr msg);

  void trajectory_callback(
    const moveit_msgs::msg::RobotTrajectory::SharedPtr msg);

  // ===========================================================================
  // DECISIÓN
  // ===========================================================================

  void decision_timer_callback();

  DecisionResult evaluate_decision();

  // ===========================================================================
  // VALIDACIÓN DE ENTRADAS YA PROCESADAS
  // ===========================================================================

  bool is_operator_state_fresh() const;
  bool is_interaction_fresh() const;
  bool is_trajectory_fresh() const;

  bool detection_information_available() const;

  // ===========================================================================
  // CONDICIONES DE OPERACIÓN
  // ===========================================================================

  bool operator_condition_anomalous() const;
  bool collision_is_critical() const;

  Zone determine_zone(double distance) const;
  const char * zone_to_string(Zone zone) const;

  // ===========================================================================
  // TRAYECTORIA
  // ===========================================================================

  bool trajectory_available() const;
  bool trajectory_target_reachable() const;

  bool trajectory_keeps_separation(
    Zone zone,
    double distance) const;

  // ===========================================================================
  // DISTANCIA Y VELOCIDAD
  // ===========================================================================

  bool get_current_or_last_valid_distance(
    double & distance) const;

  int compute_speed_percent(
    double distance) const;

  // ===========================================================================
  // CONSTRUCCIÓN DE COMANDOS
  // ===========================================================================

  DecisionResult make_emergency_stop(
    const std::string & reason) const;

  DecisionResult make_safe_stop(
    const std::string & reason) const;

  DecisionResult make_pause(
    const std::string & reason) const;

  DecisionResult make_increase_distance(
    int speed_percent,
    const std::string & reason) const;

  DecisionResult make_limit_speed(
    int speed_percent,
    const std::string & reason) const;

  DecisionResult make_normal_operation(
    int speed_percent,
    const std::string & reason) const;

  // ===========================================================================
  // PUBLICACIÓN
  // ===========================================================================

  void publish_command(
    const DecisionResult & result);

  // ===========================================================================
  // TOPICS
  // ===========================================================================

  std::string operator_state_topic_;
  std::string interaction_topic_;
  std::string trajectory_topic_;
  std::string command_topic_;

  // ===========================================================================
  // TEMPORIZACIÓN
  // ===========================================================================

  double decision_rate_hz_;
  double max_input_age_sec_;
  double max_trajectory_age_sec_;

  // ===========================================================================
  // LÍMITES DE ZONA
  //
  // Z3: d <  0.250 m
  // Z4: 0.250 m <= d < 0.400 m
  // Z5: 0.400 m <= d <= 1.600 m
  // Z6: d > 1.600 m
  // ===========================================================================

  double zone3_max_distance_;
  double zone4_max_distance_;
  double zone5_max_distance_;

  // ===========================================================================
  // PERFIL DE VELOCIDAD
  // ===========================================================================

  double full_speed_distance_;
  double half_speed_distance_;
  double minimum_speed_distance_;

  double full_speed_percent_;
  double half_speed_percent_;
  double minimum_speed_percent_;

  // ===========================================================================
  // CONDICIONES ANÓMALAS DEL OPERADOR
  // ===========================================================================

  bool pause_on_fatigue_high_;
  bool pause_on_attention_other_;
  bool pause_on_attention_unknown_;

  // ===========================================================================
  // EVALUACIÓN CONSERVADORA DE TRAYECTORIA
  // ===========================================================================

  bool conservative_trajectory_check_;
  double trajectory_min_separation_distance_;

  // ===========================================================================
  // SUBSCRIBERS / PUBLISHER / TIMER
  // ===========================================================================

  rclcpp::Subscription<
    colab_supervision_interfaces::msg::OperatorStateAnalysis>::SharedPtr
    operator_state_sub_;

  rclcpp::Subscription<
    colab_supervision_interfaces::msg::RobotOperatorInteraction>::SharedPtr
    interaction_sub_;

  rclcpp::Subscription<
    moveit_msgs::msg::RobotTrajectory>::SharedPtr
    trajectory_sub_;

  rclcpp::Publisher<
    colab_supervision_interfaces::msg::ControlCommand>::SharedPtr
    command_pub_;

  rclcpp::TimerBase::SharedPtr decision_timer_;

  // ===========================================================================
  // ÚLTIMOS ESTADOS PROCESADOS
  // ===========================================================================

  std::optional<
    colab_supervision_interfaces::msg::OperatorStateAnalysis>
    latest_operator_state_;

  std::optional<
    colab_supervision_interfaces::msg::RobotOperatorInteraction>
    latest_interaction_;

  std::optional<
    moveit_msgs::msg::RobotTrajectory>
    latest_trajectory_;

  rclcpp::Time operator_state_received_time_;
  rclcpp::Time interaction_received_time_;
  rclcpp::Time trajectory_received_time_;

  // ===========================================================================
  // ÚLTIMA DISTANCIA VÁLIDA PROCESADA
  //
  // No realiza filtrado ni agregación de percepción.
  // Solamente conserva la última distancia que ya fue calculada por
  // robot_operator_interaction.
  // ===========================================================================

  std::optional<double> last_valid_distance_;
  rclcpp::Time last_valid_distance_time_;

  // ===========================================================================
  // CONTROL DE PUBLICACIÓN / DIAGNÓSTICO
  // ===========================================================================

  std::string last_published_signature_;
  bool initialized_{false};
};

}  // namespace colab_supervision_control

#endif  // COLAB_SUPERVISION_CONTROL__CONTROL_DECISION_HPP_
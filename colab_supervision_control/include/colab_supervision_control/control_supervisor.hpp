#ifndef COLAB_SUPERVISION_CONTROL__CONTROL_SUPERVISOR_HPP_
#define COLAB_SUPERVISION_CONTROL__CONTROL_SUPERVISOR_HPP_

#include <deque>
#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
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
  // ==========================================================================
  // ESTADO DE UNA MUESTRA
  // ==========================================================================

  struct SampleState
  {
    bool operator_detected{false};
    bool robot_detected{false};

    bool warning{false};
    bool danger{false};

    bool distance_valid{false};
    double distance{0.0};
  };

  // ==========================================================================
  // ACCIONES DE SUPERVISION
  // ==========================================================================

  enum class Action
  {
    WAIT,
    ALLOW_ACTIONS,
    LIMIT_SPEED,
    PAUSE,
    STOP
  };

  // ==========================================================================
  // CALLBACK ROS
  // ==========================================================================

  void interaction_callback(
    const colab_supervision_interfaces::msg::RobotOperatorInteraction::SharedPtr msg);

  // ==========================================================================
  // EVALUACION PRINCIPAL
  // ==========================================================================

  void evaluate(
    const colab_supervision_interfaces::msg::RobotOperatorInteraction & msg);

  // ==========================================================================
  // CONDICIONES TEMPORALES
  // ==========================================================================

  bool previous_two_warnings() const;

  bool any_warning_last_five() const;

  bool five_consecutive_danger() const;

  void push_history(const SampleState & sample);

  // ==========================================================================
  // PERFIL DE VELOCIDAD
  // ==========================================================================

  double compute_speed_percent(double distance) const;

  // ==========================================================================
  // DECISION DE CONTROL
  // ==========================================================================

  Action determine_action(
    bool operator_detected,
    bool robot_detected,
    bool distance_valid,
    double distance,
    bool previous_two_warning,
    bool warning_in_last_five,
    bool five_danger,
    std::string & reason) const;

  static const char * action_to_string(Action action);

  // ==========================================================================
  // PUBLICACION DE RESULTADOS
  // ==========================================================================

  void publish_decision(
    Action action,
    double speed_percent,
    bool actions_allowed,
    const std::string & reason,
    const colab_supervision_interfaces::msg::RobotOperatorInteraction & msg,
    bool distance_valid,
    double distance);

  // ==========================================================================
  // PARAMETROS
  // ==========================================================================

  std::string interaction_topic_;
  std::string action_topic_;
  std::string speed_topic_;
  std::string actions_allowed_topic_;
  std::string status_topic_;

  // Perfil de velocidad
  double full_speed_distance_;
  double half_speed_distance_;
  double minimum_speed_distance_;

  double full_speed_percent_;
  double half_speed_percent_;
  double minimum_speed_percent_;

  // Condiciones de deteccion/control
  double robot_not_detected_min_distance_;
  double stop_distance_;

  // Condiciones temporales
  int warning_history_size_;
  int danger_pause_frames_;

  // Memoria de la ultima distancia valida
  double distance_memory_timeout_;

  // ==========================================================================
  // INTERFACES ROS
  // ==========================================================================

  rclcpp::Subscription<
    colab_supervision_interfaces::msg::RobotOperatorInteraction>::SharedPtr
    interaction_sub_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr action_pub_;

  // Porcentaje de velocidad publicado como entero: 0..100 %
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr speed_pub_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr actions_allowed_pub_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  // ==========================================================================
  // ESTADO INTERNO
  // ==========================================================================

  std::deque<SampleState> history_;

  std::optional<double> last_valid_distance_;

  rclcpp::Time last_valid_distance_time_;

  // Ultima velocidad calculada con operador detectado.
  // Se conserva internamente, pero NO se publica mientras el operador
  // no esté detectado.
  double last_speed_percent_{0.0};

  bool initialized_{false};
};

}  // namespace colab_supervision_control

#endif  // COLAB_SUPERVISION_CONTROL__CONTROL_SUPERVISOR_HPP_
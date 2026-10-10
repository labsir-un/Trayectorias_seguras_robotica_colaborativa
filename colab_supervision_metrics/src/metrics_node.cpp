#include "colab_supervision_metrics/metrics_node.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace colab_supervision_metrics
{

MetricsNode::MetricsNode()
: Node("metrics_node")
{
  interaction_topic_ = declare_parameter<std::string>(
    "interaction_topic", "/robot_operator_interaction");
  supervision_topic_ = declare_parameter<std::string>(
    "supervision_topic", "/control/robot_operator_state");
  operator_state_topic_ = declare_parameter<std::string>(
    "operator_state_topic", "/operator/state_analisis");
  command_topic_ = declare_parameter<std::string>(
    "command_topic", "/control/command");
  trajectory_topic_ = declare_parameter<std::string>(
    "trajectory_topic", "/control/trajectory");
  metrics_topic_ = declare_parameter<std::string>(
    "metrics_topic", "/metrics/experimental");

  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
  quality_window_sec_ = declare_parameter<double>("quality_window_sec", 15.0);
  max_observation_age_sec_ = declare_parameter<double>("max_observation_age_sec", 0.50);
  stop_speed_threshold_mm_s_ = declare_parameter<double>(
    "stop_speed_threshold_mm_s", 5.0);
  stop_confirmation_time_sec_ = declare_parameter<double>(
    "stop_confirmation_time_sec", 0.30);
  adaptation_timeout_sec_ = declare_parameter<double>(
    "adaptation_timeout_sec", 30.0);
  min_trajectory_distance_m_ = declare_parameter<double>(
    "min_trajectory_distance_m", 0.400);

  csv_enabled_ = declare_parameter<bool>("csv_enabled", true);
  csv_path_ = declare_parameter<std::string>(
    "csv_path", "metrics_experiment.csv");

  if (publish_rate_hz_ <= 0.0) {
    throw std::invalid_argument("publish_rate_hz must be > 0");
  }

  if (quality_window_sec_ <= 0.0) {
    throw std::invalid_argument("quality_window_sec must be > 0");
  }

  interaction_sub_ =
    create_subscription<colab_supervision_interfaces::msg::RobotOperatorInteraction>(
    interaction_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(
      &MetricsNode::interaction_callback,
      this,
      std::placeholders::_1));

  supervision_sub_ =
    create_subscription<colab_supervision_interfaces::msg::RobotOperatorSupervision>(
    supervision_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(
      &MetricsNode::supervision_callback,
      this,
      std::placeholders::_1));

  operator_state_sub_ =
    create_subscription<colab_supervision_interfaces::msg::OperatorStateAnalysis>(
    operator_state_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(
      &MetricsNode::operator_state_callback,
      this,
      std::placeholders::_1));

  command_sub_ =
    create_subscription<colab_supervision_interfaces::msg::ControlCommand>(
    command_topic_,
    20,
    std::bind(
      &MetricsNode::command_callback,
      this,
      std::placeholders::_1));

  trajectory_sub_ =
    create_subscription<moveit_msgs::msg::RobotTrajectory>(
    trajectory_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(
      &MetricsNode::trajectory_callback,
      this,
      std::placeholders::_1));

  metrics_pub_ =
    create_publisher<colab_supervision_metrics::msg::ExperimentalMetrics>(
    metrics_topic_,
    10);

  const auto period =
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));

  timer_ = create_wall_timer(
    period,
    std::bind(&MetricsNode::timer_callback, this));

  if (csv_enabled_) {
    csv_file_.open(csv_path_, std::ios::out | std::ios::app);
    if (!csv_file_.is_open()) {
      RCLCPP_WARN(
        get_logger(),
        "No se pudo abrir CSV en '%s'. Se mantiene la publicación ROS.",
        csv_path_.c_str());
      csv_enabled_ = false;
    }
  }

  RCLCPP_INFO(get_logger(), "====================================================");
  RCLCPP_INFO(get_logger(), "COLAB SUPERVISION METRICS - C++");
  RCLCPP_INFO(get_logger(), "Interaction: %s", interaction_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Supervision: %s", supervision_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Operator state: %s", operator_state_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Control command: %s", command_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Trajectory: %s", trajectory_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Metrics: %s", metrics_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Quality window: %.1f s", quality_window_sec_);
  RCLCPP_INFO(get_logger(), "Stop threshold: %.2f mm/s", stop_speed_threshold_mm_s_);
  RCLCPP_INFO(get_logger(), "====================================================");
}

void MetricsNode::interaction_callback(
  const colab_supervision_interfaces::msg::RobotOperatorInteraction::SharedPtr msg)
{
  ObservationSample sample;
  sample.time = msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0
    ? get_clock()->now()
    : rclcpp::Time(msg->header.stamp);

  sample.operator_valid =
    msg->operator_detected && msg->operator_position_valid &&
    std::isfinite(msg->operator_position.x) &&
    std::isfinite(msg->operator_position.y) &&
    std::isfinite(msg->operator_position.z);

  sample.robot_valid =
    msg->robot_detected && msg->robot_position_valid &&
    std::isfinite(msg->robot_position.x) &&
    std::isfinite(msg->robot_position.y) &&
    std::isfinite(msg->robot_position.z);

  sample.interaction_valid =
    msg->interaction_valid &&
    std::isfinite(msg->distance) &&
    msg->distance >= 0.0;

  if (sample.operator_valid) {
    sample.operator_x = msg->operator_position.x;
    sample.operator_y = msg->operator_position.y;
    sample.operator_z = msg->operator_position.z;
  }

  if (sample.robot_valid) {
    sample.robot_x = msg->robot_position.x;
    sample.robot_y = msg->robot_position.y;
    sample.robot_z = msg->robot_position.z;
  }

  latest_observation_ = sample;
  latest_interaction_receive_time_ = get_clock()->now();

  update_motion_metrics(sample);
  update_quality_window(sample);
}

void MetricsNode::supervision_callback(
  const colab_supervision_interfaces::msg::RobotOperatorSupervision::SharedPtr msg)
{
  latest_supervision_ = *msg;
}

void MetricsNode::operator_state_callback(
  const colab_supervision_interfaces::msg::OperatorStateAnalysis::SharedPtr msg)
{
  latest_operator_state_ = *msg;
}

void MetricsNode::command_callback(
  const colab_supervision_interfaces::msg::ControlCommand::SharedPtr msg)
{
  latest_command_receive_time_ = get_clock()->now();

  // --------------------------------------------------------------------------
  // Adaptación de trayectoria
  //
  // La métrica inicia al emitirse una orden de modificación de trayectoria.
  // El cierre se produce al recibir una nueva trayectoria MoveIt.
  // --------------------------------------------------------------------------
  const bool is_adaptation_command =
    msg->command == "Distancia" &&
    msg->object == "CambiarTrayectoria";

  if (is_adaptation_command && !adaptation_active_) {
    adaptation_active_ = true;
    adaptation_start_time_ = get_clock()->now();
    trajectory_generation_at_adaptation_start_ = trajectory_generation_;
    adaptation_time_valid_ = false;
    RCLCPP_INFO(
      get_logger(),
      "Inicio de medición de adaptación de trayectoria.");
  }

  // --------------------------------------------------------------------------
  // Detención
  //
  // Se considera inicio de detención cuando control_decision emite:
  // Sock_Pausa, Sock_Parada o Sock_Bloquear.
  // --------------------------------------------------------------------------
  const bool is_stop_command =
    msg->command == "Variable" &&
    msg->object == "SET" &&
    (msg->variable == "Sock_Pausa" ||
     msg->variable == "Sock_Parada" ||
     msg->variable == "Sock_Bloquear") &&
    msg->value == "TRUE";

  if (is_stop_command && !stop_active_) {
    stop_active_ = true;
    stop_start_time_ = get_clock()->now();
    stop_time_valid_ = false;
    low_speed_since_.reset();

    RCLCPP_INFO(
      get_logger(),
      "Inicio de medición de detención: %s / %s",
      msg->variable.c_str(),
      msg->value.c_str());
  }
}

void MetricsNode::trajectory_callback(
  const moveit_msgs::msg::RobotTrajectory::SharedPtr msg)
{
  latest_trajectory_ = *msg;
  ++trajectory_generation_;

  trajectory_feasible_valid_ = true;
  trajectory_feasible_ =
    trajectory_has_valid_joint_points(*msg) &&
    current_safety_state_is_compatible();

  update_adaptation_metric();
}

void MetricsNode::timer_callback()
{
  update_adaptation_metric();
  update_stop_metric();
  publish_metrics();

  if (csv_enabled_) {
    write_csv_row();
  }
}

void MetricsNode::update_motion_metrics(const ObservationSample & sample)
{
  distance_or_valid_ = false;
  robot_speed_valid_ = false;
  relative_position_variation_valid_ = false;

  if (sample.interaction_valid && sample.operator_valid && sample.robot_valid) {
    distance_or_mm_ =
      euclidean_distance(
      sample.operator_x, sample.operator_y, sample.operator_z,
      sample.robot_x, sample.robot_y, sample.robot_z) * 1000.0;
    distance_or_valid_ = std::isfinite(distance_or_mm_);
  }

  if (previous_observation_.has_value()) {
    const auto & previous = *previous_observation_;
    const double dt = (sample.time - previous.time).seconds();

    if (dt > 1e-6 && dt < 10.0 && sample.robot_valid && previous.robot_valid) {
      const double displacement =
        euclidean_distance(
        sample.robot_x, sample.robot_y, sample.robot_z,
        previous.robot_x, previous.robot_y, previous.robot_z);

      robot_speed_mm_s_ = (displacement / dt) * 1000.0;
      robot_speed_valid_ = std::isfinite(robot_speed_mm_s_);
    }

    if (
      dt > 1e-6 && dt < 10.0 &&
      sample.operator_valid && sample.robot_valid &&
      previous.operator_valid && previous.robot_valid)
    {
      const double current_rx = sample.operator_x - sample.robot_x;
      const double current_ry = sample.operator_y - sample.robot_y;
      const double current_rz = sample.operator_z - sample.robot_z;

      const double previous_rx = previous.operator_x - previous.robot_x;
      const double previous_ry = previous.operator_y - previous.robot_y;
      const double previous_rz = previous.operator_z - previous.robot_z;

      relative_position_variation_mm_ =
        euclidean_distance(
        current_rx, current_ry, current_rz,
        previous_rx, previous_ry, previous_rz) * 1000.0;

      relative_position_variation_valid_ =
        std::isfinite(relative_position_variation_mm_);
    }
  }

  previous_observation_ = sample;
}

void MetricsNode::update_adaptation_metric()
{
  if (!adaptation_active_) {
    return;
  }

  const auto now = get_clock()->now();

  if (trajectory_generation_ > trajectory_generation_at_adaptation_start_) {
    adaptation_time_ms_ =
      (now - adaptation_start_time_).seconds() * 1000.0;
    adaptation_time_valid_ = true;
    adaptation_active_ = false;

    RCLCPP_INFO(
      get_logger(),
      "Adaptación medida: %.3f ms.",
      adaptation_time_ms_);

    return;
  }

  if ((now - adaptation_start_time_).seconds() > adaptation_timeout_sec_) {
    adaptation_active_ = false;
    adaptation_time_valid_ = false;

    RCLCPP_WARN(
      get_logger(),
      "La medición de adaptación superó el timeout de %.2f s.",
      adaptation_timeout_sec_);
  }
}

void MetricsNode::update_stop_metric()
{
  if (!stop_active_ || !robot_speed_valid_) {
    return;
  }

  const auto now = get_clock()->now();

  if (robot_speed_mm_s_ <= stop_speed_threshold_mm_s_) {
    if (!low_speed_since_.has_value()) {
      low_speed_since_ = now;
    }

    if ((now - *low_speed_since_).seconds() >= stop_confirmation_time_sec_) {
      stop_time_ms_ =
        (now - stop_start_time_).seconds() * 1000.0;
      stop_time_valid_ = true;
      stop_active_ = false;

      RCLCPP_INFO(
        get_logger(),
        "Detención medida: %.3f ms.",
        stop_time_ms_);
    }
  } else {
    low_speed_since_.reset();
  }
}

void MetricsNode::update_quality_window(const ObservationSample & sample)
{
  const auto now = get_clock()->now();

  quality_window_.push_back(
    QualityWindowSample{
      sample.time,
      sample.operator_valid,
      sample.robot_valid});

  while (!quality_window_.empty() &&
    (now - quality_window_.front().time).seconds() > quality_window_sec_)
  {
    quality_window_.pop_front();
  }

  if (quality_window_.empty()) {
    perception_quality_percent_ = 0.0;
    perception_quality_valid_ = false;
    return;
  }

  std::size_t evaluated = 0;
  std::size_t valid = 0;

  for (const auto & item : quality_window_) {
    // Se evalúan observaciones realmente disponibles.
    // La ausencia válida de operador no se considera automáticamente fallo;
    // aquí se mide la validez geométrica de la observación recibida.
    if (item.operator_valid || item.robot_valid) {
      ++evaluated;
      if (item.operator_valid && item.robot_valid) {
        ++valid;
      }
    }
  }

  if (evaluated > 0) {
    perception_quality_percent_ =
      100.0 * static_cast<double>(valid) /
      static_cast<double>(evaluated);
    perception_quality_valid_ = true;
  } else {
    perception_quality_percent_ = 0.0;
    perception_quality_valid_ = false;
  }
}

bool MetricsNode::trajectory_has_valid_joint_points(
  const moveit_msgs::msg::RobotTrajectory & trajectory) const
{
  const auto & points = trajectory.joint_trajectory.points;

  if (points.empty()) {
    return false;
  }

  if (trajectory.joint_trajectory.joint_names.empty()) {
    return false;
  }

  for (const auto & point : points) {
    if (point.positions.size() != trajectory.joint_trajectory.joint_names.size()) {
      return false;
    }

    for (const auto value : point.positions) {
      if (!std::isfinite(value)) {
        return false;
      }
    }
  }

  return true;
}

bool MetricsNode::current_safety_state_is_compatible() const
{
  if (!latest_supervision_.has_value()) {
    return false;
  }

  const auto & state = *latest_supervision_;

  if (!state.distance_valid) {
    return false;
  }

  if (state.collision_imminent) {
    return false;
  }

  if (
    state.collision_risk == "CRITICAL" ||
    state.collision_risk == "DANGEROUS")
  {
    return false;
  }

  return static_cast<double>(state.distance) >= min_trajectory_distance_m_;
}

void MetricsNode::publish_metrics()
{
  colab_supervision_metrics::msg::ExperimentalMetrics msg;

  const auto now = get_clock()->now();
  msg.header.stamp = now;

  msg.distance_or_mm = static_cast<float>(distance_or_mm_);
  msg.distance_or_valid = distance_or_valid_;

  msg.robot_speed_mm_s = static_cast<float>(robot_speed_mm_s_);
  msg.robot_speed_valid = robot_speed_valid_;

  msg.relative_position_variation_mm =
    static_cast<float>(relative_position_variation_mm_);
  msg.relative_position_variation_valid =
    relative_position_variation_valid_;

  msg.adaptation_time_ms = static_cast<float>(adaptation_time_ms_);
  msg.adaptation_time_valid = adaptation_time_valid_;

  msg.perception_time_ms = 0.0F;
  msg.perception_time_valid = false;

  // No se inventa un tiempo interno de procesamiento de percepción:
  // el mensaje de interacción contiene timestamp de adquisición, pero no
  // timestamp de inicio/fin del procesamiento interno.
  if (latest_interaction_receive_time_.nanoseconds() > 0 &&
    latest_observation_.has_value())
  {
    const double latency =
      (latest_interaction_receive_time_ - latest_observation_->time).seconds();

    if (latency >= 0.0 && std::isfinite(latency)) {
      msg.perception_time_ms = static_cast<float>(latency * 1000.0);
      msg.perception_time_valid = true;
    }
  }

  msg.stop_time_ms = static_cast<float>(stop_time_ms_);
  msg.stop_time_valid = stop_time_valid_;

  msg.trajectory_feasible = trajectory_feasible_;
  msg.trajectory_feasible_valid = trajectory_feasible_valid_;

  msg.perception_quality_percent =
    static_cast<float>(perception_quality_percent_);
  msg.perception_quality_valid =
    perception_quality_valid_;

  msg.operator_observation_valid =
    latest_observation_.has_value() &&
    latest_observation_->operator_valid;

  msg.robot_observation_valid =
    latest_observation_.has_value() &&
    latest_observation_->robot_valid;

  msg.adaptation_active = adaptation_active_;
  msg.stop_active = stop_active_;

  metrics_pub_->publish(msg);
}

void MetricsNode::write_csv_row()
{
  if (!csv_enabled_ || !csv_file_.is_open()) {
    return;
  }

  if (!csv_header_written_) {
    csv_file_
      << "timestamp,"
      << "distance_or_mm,"
      << "distance_or_valid,"
      << "robot_speed_mm_s,"
      << "robot_speed_valid,"
      << "relative_position_variation_mm,"
      << "relative_position_variation_valid,"
      << "adaptation_time_ms,"
      << "adaptation_time_valid,"
      << "perception_time_ms,"
      << "perception_time_valid,"
      << "stop_time_ms,"
      << "stop_time_valid,"
      << "trajectory_feasible,"
      << "trajectory_feasible_valid,"
      << "perception_quality_percent,"
      << "perception_quality_valid,"
      << "operator_observation_valid,"
      << "robot_observation_valid,"
      << "adaptation_active,"
      << "stop_active\n";

    csv_header_written_ = true;
  }

  const auto now = get_clock()->now();

  csv_file_
    << std::fixed << std::setprecision(6)
    << now.seconds() << ","
    << distance_or_mm << ","
    << distance_or_valid_ << ","
    << robot_speed_mm_s_ << ","
    << robot_speed_valid_ << ","
    << relative_position_variation_mm_ << ","
    << relative_position_variation_valid_ << ","
    << adaptation_time_ms_ << ","
    << adaptation_time_valid_ << ","
    << "0.0" << ","
    << "0" << ","
    << stop_time_ms_ << ","
    << stop_time_valid_ << ","
    << trajectory_feasible_ << ","
    << trajectory_feasible_valid_ << ","
    << perception_quality_percent_ << ","
    << perception_quality_valid_ << ","
    << (latest_observation_.has_value() && latest_observation_->operator_valid) << ","
    << (latest_observation_.has_value() && latest_observation_->robot_valid) << ","
    << adaptation_active_ << ","
    << stop_active_ << "\n";

  csv_file_.flush();
}

double MetricsNode::euclidean_distance(
  double x1, double y1, double z1,
  double x2, double y2, double z2)
{
  const double dx = x1 - x2;
  const double dy = y1 - y2;
  const double dz = z1 - z2;

  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace colab_supervision_metrics

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    auto node =
      std::make_shared<colab_supervision_metrics::MetricsNode>();

    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("metrics_node"),
      "Node initialization failed: %s",
      error.what());
  }

  rclcpp::shutdown();
  return 0;
}

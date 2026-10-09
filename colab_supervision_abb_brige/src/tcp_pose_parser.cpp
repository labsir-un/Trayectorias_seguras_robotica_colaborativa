#include "colab_supervision_abb_brige/tcp_pose_parser.hpp"

#include <cmath>
#include <sstream>
#include <string>
#include <vector>

namespace
{

bool extract_block(
  const std::string & text,
  const std::string & key,
  std::string & content)
{
  const std::string key_pos = key + ":";

  const std::size_t key_start = text.find(key_pos);

  if (key_start == std::string::npos) {
    return false;
  }

  const std::size_t open_brace = text.find('{', key_start);

  if (open_brace == std::string::npos) {
    return false;
  }

  const std::size_t close_brace = text.find('}', open_brace);

  if (close_brace == std::string::npos) {
    return false;
  }

  content = text.substr(
    open_brace + 1,
    close_brace - open_brace - 1);

  return true;
}


bool parse_numbers(
  const std::string & text,
  std::vector<double> & values)
{
  values.clear();

  std::stringstream ss(text);
  std::string token;

  while (std::getline(ss, token, ',')) {

    // Eliminar espacios y saltos de línea
    const std::size_t first =
      token.find_first_not_of(" \t\r\n");

    const std::size_t last =
      token.find_last_not_of(" \t\r\n");

    if (first == std::string::npos) {
      continue;
    }

    const std::string value =
      token.substr(first, last - first + 1);

    try {
      values.push_back(std::stod(value));
    }
    catch (const std::exception &) {
      return false;
    }
  }

  return true;
}

}  // namespace


TcpPoseParserNode::TcpPoseParserNode()
: Node("tcp_pose_parser")
{
  cartesian_path_ =
    this->declare_parameter<bool>("cartesian_path", false);

  /*
   * Suscripción al socket TCP
   */
  receive_subscription_ =
    this->create_subscription<std_msgs::msg::String>(
    "/tcp_socket/receive",
    rclcpp::QoS(10),
    std::bind(
      &TcpPoseParserNode::receive_callback,
      this,
      std::placeholders::_1));

  /*
   * Publicador TCP con cuaterniones
   */
  tcp_cua_publisher_ =
    this->create_publisher<
      colab_supervision_interfaces::msg::PoseCuaCommand>(
      "/SockRes/TCPcua",
      rclcpp::QoS(10));

  /*
   * Publicador TCP con RPY
   */
  tcp_eul_publisher_ =
    this->create_publisher<
      colab_supervision_interfaces::msg::PoseCommand>(
      "/SockRes/TCPeul",
      rclcpp::QoS(10));

  RCLCPP_INFO(
    this->get_logger(),
    "==============================================");

  RCLCPP_INFO(
    this->get_logger(),
    "ABB TCP Pose Parser iniciado");

  RCLCPP_INFO(
    this->get_logger(),
    "Entrada : /tcp_socket/receive");

  RCLCPP_INFO(
    this->get_logger(),
    "Salida CUA : /SockRes/TCPcua");

  RCLCPP_INFO(
    this->get_logger(),
    "Salida EUL : /SockRes/TCPeul");

  RCLCPP_INFO(
    this->get_logger(),
    "cartesian_path: %s",
    cartesian_path_ ? "true" : "false");

  RCLCPP_INFO(
    this->get_logger(),
    "==============================================");
}


void TcpPoseParserNode::receive_callback(
  const std_msgs::msg::String::SharedPtr msg)
{
  const std::string & text = msg->data;

  /*
   * Mostrar el mensaje recibido para diagnóstico.
   */
  RCLCPP_INFO(
    this->get_logger(),
    "Mensaje recibido: [%s]",
    text.c_str());


  /*
   * Determinar el tipo de mensaje.
   */
  const bool is_tcp_cua =
    text.find("TCPcua") != std::string::npos;

  const bool is_tcp_eul =
    text.find("TCPeul") != std::string::npos;


  if (!is_tcp_cua && !is_tcp_eul) {

    RCLCPP_WARN(
      this->get_logger(),
      "Mensaje recibido pero no corresponde a TCPcua ni TCPeul");

    return;
  }


  /*
   * Extraer Trans.
   *
   * Ejemplo:
   *
   * Trans: {788.429,0,593.609}
   */
  std::string trans_block;

  if (!extract_block(text, "Trans", trans_block)) {

    RCLCPP_WARN(
      this->get_logger(),
      "No se pudo extraer el bloque Trans");

    return;
  }


  std::vector<double> translation;

  if (!parse_numbers(trans_block, translation)) {

    RCLCPP_WARN(
      this->get_logger(),
      "No se pudieron convertir los valores de Trans");

    return;
  }


  if (translation.size() != 3) {

    RCLCPP_WARN(
      this->get_logger(),
      "Trans debe contener 3 valores. Se recibieron %zu",
      translation.size());

    return;
  }


  /*
   * Conversión:
   *
   * ABB -> milímetros
   * ROS -> metros
   */
  const double x =
    translation[0] / 1000.0;

  const double y =
    translation[1] / 1000.0;

  const double z =
    translation[2] / 1000.0;


  /*
   * ==========================================================
   * TCPcua
   * ==========================================================
   *
   * Entrada:
   *
   * Rot: {w,x,y,z}
   *
   * Salida:
   *
   * q1 = x
   * q2 = y
   * q3 = z
   * q4 = w
   */
  if (is_tcp_cua) {

    std::string rot_block;

    if (!extract_block(text, "Rot", rot_block)) {

      RCLCPP_WARN(
        this->get_logger(),
        "No se pudo extraer el bloque Rot de TCPcua");

      return;
    }


    std::vector<double> quaternion;

    if (!parse_numbers(rot_block, quaternion)) {

      RCLCPP_WARN(
        this->get_logger(),
        "No se pudieron convertir los valores de quaternion");

      return;
    }


    if (quaternion.size() != 4) {

      RCLCPP_WARN(
        this->get_logger(),
        "El quaternion debe contener 4 valores. "
        "Se recibieron %zu",
        quaternion.size());

      return;
    }


    /*
     * Entrada ABB:
     *
     * [w,x,y,z]
     */
    const double w = quaternion[0];
    const double qx = quaternion[1];
    const double qy = quaternion[2];
    const double qz = quaternion[3];


    /*
     * Salida PoseCua:
     *
     * [q1,q2,q3,q4] = [x,y,z,w]
     */
    colab_supervision_interfaces::msg::PoseCuaCommand output;

    output.pose.x = x;
    output.pose.y = y;
    output.pose.z = z;

    output.pose.q1 = qx;
    output.pose.q2 = qy;
    output.pose.q3 = qz;
    output.pose.q4 = w;

    output.cartesian_path =
      cartesian_path_;


    /*
     * Publicar
     */
    tcp_cua_publisher_->publish(output);


    RCLCPP_INFO(
      this->get_logger(),
      "TCPcua PUBLICADO -> "
      "P=[%.6f, %.6f, %.6f] m | "
      "Q=[%.6f, %.6f, %.6f, %.6f]",
      output.pose.x,
      output.pose.y,
      output.pose.z,
      output.pose.q1,
      output.pose.q2,
      output.pose.q3,
      output.pose.q4);

    return;
  }


  /*
   * ==========================================================
   * TCPeul
   * ==========================================================
   *
   * Entrada:
   *
   * Rot: {roll,pitch,yaw}
   *
   * ABB -> grados
   *
   * ROS -> radianes
   */
  if (is_tcp_eul) {

    std::string rot_block;

    if (!extract_block(text, "Rot", rot_block)) {

      RCLCPP_WARN(
        this->get_logger(),
        "No se pudo extraer el bloque Rot de TCPeul");

      return;
    }


    std::vector<double> rpy_deg;

    if (!parse_numbers(rot_block, rpy_deg)) {

      RCLCPP_WARN(
        this->get_logger(),
        "No se pudieron convertir los valores RPY");

      return;
    }


    if (rpy_deg.size() != 3) {

      RCLCPP_WARN(
        this->get_logger(),
        "RPY debe contener 3 valores. "
        "Se recibieron %zu",
        rpy_deg.size());

      return;
    }


    /*
     * Conversión grados -> radianes
     */
    constexpr double DEG_TO_RAD =
      3.14159265358979323846 / 180.0;


    const double roll =
      rpy_deg[0] * DEG_TO_RAD;

    const double pitch =
      rpy_deg[1] * DEG_TO_RAD;

    const double yaw =
      rpy_deg[2] * DEG_TO_RAD;


    /*
     * Crear mensaje
     */
    colab_supervision_interfaces::msg::PoseCommand output;

    output.pose.x = x;
    output.pose.y = y;
    output.pose.z = z;

    output.pose.roll = roll;
    output.pose.pitch = pitch;
    output.pose.yaw = yaw;

    output.cartesian_path =
      cartesian_path_;


    /*
     * Publicar
     */
    tcp_eul_publisher_->publish(output);


    RCLCPP_INFO(
      this->get_logger(),
      "TCPeul PUBLICADO -> "
      "P=[%.6f, %.6f, %.6f] m | "
      "RPY=[%.6f, %.6f, %.6f] rad",
      output.pose.x,
      output.pose.y,
      output.pose.z,
      output.pose.roll,
      output.pose.pitch,
      output.pose.yaw);
  }
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<TcpPoseParserNode>());

  rclcpp::shutdown();

  return 0;
}
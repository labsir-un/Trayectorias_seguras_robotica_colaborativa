#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <colab_supervision_interfaces/msg/joints.hpp>

#include <cmath>
#include <sstream>
#include <string>
#include <vector>


class TcpJointsNode : public rclcpp::Node
{
public:

  TcpJointsNode()
  : Node("tcp_joints")
  {
    /*
     * Suscripción a las respuestas del socket
     */
    receive_subscription_ =
      this->create_subscription<std_msgs::msg::String>(
        "/tcp_socket/receive",
        rclcpp::QoS(10),
        std::bind(
          &TcpJointsNode::receive_callback,
          this,
          std::placeholders::_1));


    /*
     * Publicador de posiciones articulares
     */
    joints_publisher_ =
      this->create_publisher<
        colab_supervision_interfaces::msg::Joints>(
        "/SockRes/Joints",
        rclcpp::QoS(10));


    RCLCPP_INFO(
      this->get_logger(),
      "==============================================");

    RCLCPP_INFO(
      this->get_logger(),
      "TCP Joints Node iniciado");

    RCLCPP_INFO(
      this->get_logger(),
      "Entrada : /tcp_socket/receive");

    RCLCPP_INFO(
      this->get_logger(),
      "Salida  : /SockRes/Joints");

    RCLCPP_INFO(
      this->get_logger(),
      "Formato esperado: T_ROB1 : J1,J2,J3,J4,J5,J6");

    RCLCPP_INFO(
      this->get_logger(),
      "==============================================");
  }


private:

  void receive_callback(
    const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string & text = msg->data;


    RCLCPP_INFO(
      this->get_logger(),
      "Mensaje recibido: [%s]",
      text.c_str());


    /*
     * Buscar T_ROB1
     */
    const std::size_t rob_index =
      text.find("T_ROB1");

    if (rob_index == std::string::npos) {

      /*
       * No es un mensaje de posiciones articulares.
       */
      return;
    }


    /*
     * Buscar ':' después de T_ROB1
     */
    const std::size_t colon_index =
      text.find(':', rob_index);

    if (colon_index == std::string::npos) {

      RCLCPP_WARN(
        this->get_logger(),
        "Mensaje T_ROB1 sin ':'");

      return;
    }


    /*
     * Obtener únicamente la parte de los valores.
     *
     * Ejemplo:
     *
     * -6.4246,-25.9943,16.9358,
     * 5.6817,54.3766,-8.6824
     */
    const std::string values_text =
      text.substr(colon_index + 1);


    /*
     * Separar los seis valores
     */
    std::stringstream ss(values_text);

    std::string token;

    std::vector<double> joints_deg;


    while (std::getline(ss, token, ','))
    {
      /*
       * Eliminar espacios y saltos de línea
       */
      const std::size_t first =
        token.find_first_not_of(" \t\r\n");

      const std::size_t last =
        token.find_last_not_of(" \t\r\n");


      if (first == std::string::npos) {
        continue;
      }


      const std::string value =
        token.substr(
          first,
          last - first + 1);


      try {

        joints_deg.push_back(
          std::stod(value));

      }
      catch (const std::exception &) {

        RCLCPP_WARN(
          this->get_logger(),
          "Valor articular inválido: [%s]",
          value.c_str());

        return;
      }
    }


    /*
     * El ABB IRB 140 tiene seis articulaciones.
     */
    if (joints_deg.size() != 6)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Se esperaban 6 articulaciones, "
        "pero se recibieron %zu",
        joints_deg.size());

      return;
    }


    /*
     * Conversión:
     *
     * grados -> radianes
     */
    constexpr double DEG_TO_RAD =
      3.14159265358979323846 / 180.0;


    std::vector<double> joints_rad;

    joints_rad.reserve(6);


    for (const double joint_deg : joints_deg)
    {
      joints_rad.push_back(
        joint_deg * DEG_TO_RAD);
    }


    /*
     * Crear mensaje Joints
     */
    colab_supervision_interfaces::msg::Joints output;


    /*
     * Publicar los valores en el mismo orden:
     *
     * J1
     * J2
     * J3
     * J4
     * J5
     * J6
     */
    output.data = joints_rad;


    /*
     * Publicar
     */
    joints_publisher_->publish(output);


    /*
     * Mostrar resultado
     */
    RCLCPP_INFO(
      this->get_logger(),
      "Joints PUBLICADO -> "
      "[%.6f, %.6f, %.6f, %.6f, %.6f, %.6f] rad",
      joints_rad[0],
      joints_rad[1],
      joints_rad[2],
      joints_rad[3],
      joints_rad[4],
      joints_rad[5]);
  }


  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    receive_subscription_;


  rclcpp::Publisher<
    colab_supervision_interfaces::msg::Joints>::SharedPtr
    joints_publisher_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<TcpJointsNode>());

  rclcpp::shutdown();

  return 0;
}
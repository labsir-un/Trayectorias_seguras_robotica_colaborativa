#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

class TcpReadyNode : public rclcpp::Node
{
public:
  TcpReadyNode()
  : Node("tcp_ready")
  {
    /*
     * Suscripción a las respuestas del socket
     */
    receive_subscription_ =
      this->create_subscription<std_msgs::msg::String>(
        "/tcp_socket/receive",
        rclcpp::QoS(10),
        std::bind(
          &TcpReadyNode::receive_callback,
          this,
          std::placeholders::_1));

    /*
     * Publicador de la bandera de disponibilidad
     */
    ready_publisher_ =
      this->create_publisher<std_msgs::msg::Bool>(
        "/SockRes/Ready",
        rclcpp::QoS(10));

    RCLCPP_INFO(
      this->get_logger(),
      "==============================================");

    RCLCPP_INFO(
      this->get_logger(),
      "TCP Ready Node iniciado");

    RCLCPP_INFO(
      this->get_logger(),
      "Entrada : /tcp_socket/receive");

    RCLCPP_INFO(
      this->get_logger(),
      "Salida  : /SockRes/Ready");

    RCLCPP_INFO(
      this->get_logger(),
      "==============================================");
  }

private:

  void receive_callback(
    const std_msgs::msg::String::SharedPtr msg)
  {
    /*
     * Se recibió cualquier respuesta del socket.
     *
     * No importa cuál sea el contenido.
     */
    RCLCPP_INFO(
      this->get_logger(),
      "Respuesta recibida del socket: [%s]",
      msg->data.c_str());

    /*
     * Generar un único evento TRUE.
     */
    std_msgs::msg::Bool ready_msg;

    ready_msg.data = true;

    ready_publisher_->publish(ready_msg);

    RCLCPP_INFO(
      this->get_logger(),
      "Evento /SockRes/Ready -> TRUE");
  }


  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    receive_subscription_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
    ready_publisher_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<TcpReadyNode>());

  rclcpp::shutdown();

  return 0;
}
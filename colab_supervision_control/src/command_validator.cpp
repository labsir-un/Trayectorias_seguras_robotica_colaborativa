#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "colab_supervision_interfaces/msg/control_command.hpp"

#include "colab_supervision_control/command_validator.hpp"

using ControlCommand =
  colab_supervision_interfaces::msg::ControlCommand;

using namespace colab_supervision_control;


class ControlCommandValidatorNode : public rclcpp::Node
{
public:

  ControlCommandValidatorNode()
  : Node("ControlCommand_validator_node")
  {
    ControlCommand_subscriber_ =
      this->create_subscription<ControlCommand>(
        "/ControlCommand/request",
        10,
        std::bind(
          &ControlCommandValidatorNode::ControlCommandCallback,
          this,
          std::placeholders::_1));

    ControlCommand_publisher_ =
      this->create_publisher<ControlCommand>(
        "/ControlCommand/validated",
        10);

    RCLCPP_INFO(
      this->get_logger(),
      "ControlCommand Validator iniciado");
  }


private:

  void ControlCommandCallback(
    const ControlCommand::SharedPtr msg)
  {
    const auto result =
      CommandValidator::validate(
        msg->command,
        msg->object,
        msg->variable,
        msg->value);

    if (result.valid)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Comando valido: [%s] [%s] [%s] [%s]",
        msg->command.c_str(),
        msg->object.c_str(),
        msg->variable.c_str(),
        msg->value.c_str());

      ControlCommand_publisher_->publish(*msg);
    }
    else
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Comando rechazado: [%s] [%s] [%s] [%s]",
        msg->command.c_str(),
        msg->object.c_str(),
        msg->variable.c_str(),
        msg->value.c_str());

      RCLCPP_WARN(
        this->get_logger(),
        "Motivo: %s",
        result.reason.c_str());
    }
  }


  rclcpp::Subscription<ControlCommand>::SharedPtr
    ControlCommand_subscriber_;

  rclcpp::Publisher<ControlCommand>::SharedPtr
    ControlCommand_publisher_;
};


int main(
  int argc,
  char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<ControlCommandValidatorNode>());

  rclcpp::shutdown();

  return 0;
}
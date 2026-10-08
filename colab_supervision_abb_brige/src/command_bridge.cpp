
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <colab_supervision_interfaces/msg/control_command.hpp>

#include <string>
#include <unordered_map>
#include <unordered_set>

class ControlCommandBridge : public rclcpp::Node
{
public:
    ControlCommandBridge()
        : Node("control_command_bridge")
    {
        // ============================================================
        // PARAMETROS
        // ============================================================

        input_topic_ = this->declare_parameter<std::string>(
            "input_topic", "/control/command_validated");

        output_topic_ = this->declare_parameter<std::string>(
            "output_topic", "/tcp_socket/send");

        // ============================================================
        // PUBLICADOR
        // ============================================================

        publisher_ = this->create_publisher<std_msgs::msg::String>(
            output_topic_, 10);

        // ============================================================
        // SUSCRIPTOR
        // ============================================================

        subscription_ =
            this->create_subscription<
                colab_supervision_interfaces::msg::ControlCommand>(
                input_topic_,
                10,
                std::bind(
                    &ControlCommandBridge::command_callback,
                    this,
                    std::placeholders::_1));

        RCLCPP_INFO(
            this->get_logger(),
            "ControlCommandBridge iniciado. Suscrito a: %s",
            input_topic_.c_str());

        RCLCPP_INFO(
            this->get_logger(),
            "Publicando comandos en: %s",
            output_topic_.c_str());
    }

private:

    // ================================================================
    // COMBINACIONES VALIDAS: COMANDO -> OBJETIVOS
    //
    // La validacion se limita a command y object.
    // variable y value son responsabilidad del programa RAPID.
    // ================================================================

    const std::unordered_map<
        std::string,
        std::unordered_set<std::string>> valid_commands_ = {

        {
            "Velocidad",
            {
                "Porcentaje",
                "Max",
                "ACTVEL"
            }
        },

        {
            "Distancia",
            {
                "GetPostura",
                "CambiarTrayectoria",
                "Postura_segura"
            }
        },

        {
            "Variable",
            {
                "GET",
                "SET"
            }
        }
    };

    // ================================================================
    // VALIDACION DEL COMANDO Y OBJETIVO
    // ================================================================

    bool is_valid_command(
        const std::string & command,
        const std::string & object)
    {
        const auto command_it = valid_commands_.find(command);

        // Verificar que exista el comando
        if (command_it == valid_commands_.end())
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Comando no reconocido: '%s'",
                command.c_str());

            return false;
        }

        // Verificar que el objetivo sea compatible con el comando
        if (command_it->second.find(object) ==
            command_it->second.end())
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Objetivo '%s' no valido para el comando '%s'",
                object.c_str(),
                command.c_str());

            return false;
        }

        return true;
    }

    // ================================================================
    // CALLBACK DEL MENSAJE PERSONALIZADO
    // ================================================================

    void command_callback(
        const colab_supervision_interfaces::msg::ControlCommand::SharedPtr msg)
    {
        // ------------------------------------------------------------
        // 1. Validar comando y objetivo
        // ------------------------------------------------------------

        if (!is_valid_command(msg->command, msg->object))
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Comando rechazado. No se enviara por TCP.");

            return;
        }

        // ------------------------------------------------------------
        // 2. Construir mensaje de texto
        //
        // Formato:
        // Comando/Objetivo/Variable/Valor
        // ------------------------------------------------------------

        const std::string command_string =
            msg->command + "/" +
            msg->object + "/" +
            msg->variable + "/" +
            msg->value;

        // ------------------------------------------------------------
        // 3. Publicar mensaje para el nodo TCP
        // ------------------------------------------------------------

        std_msgs::msg::String output_msg;
        output_msg.data = command_string;

        publisher_->publish(output_msg);

        RCLCPP_INFO(
            this->get_logger(),
            "Comando publicado: %s",
            command_string.c_str());
    }

    // ================================================================
    // MIEMBROS DEL NODO
    // ================================================================

    std::string input_topic_;
    std::string output_topic_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    rclcpp::Subscription<
        colab_supervision_interfaces::msg::ControlCommand>::SharedPtr
        subscription_;
};


// ====================================================================
// MAIN
// ====================================================================

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<ControlCommandBridge>());

    rclcpp::shutdown();
    return 0;
}

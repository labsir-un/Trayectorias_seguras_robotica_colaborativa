
/*
 * ============================================================
 * TCP Socket Node
 * Package: colab_supervision_abb_brige
 * ============================================================
 *
 * ROS 2 TCP client for bidirectional communication.
 *
 * Parameters:
 *   server_ip
 *   server_port
 *   tx_topic
 *   rx_topic
 *   reconnect_period
 *   receive_buffer_size
 *
 * Topics:
 *   /tcp_socket/send
 *   /tcp_socket/receive
 *
 * Outgoing messages are terminated with '\n'.
 *
 * ============================================================
 */

// ============================================================
// IMPORTS
// ============================================================

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>


// ============================================================
// TCP SOCKET NODE
// ============================================================

class TCPSocketNode : public rclcpp::Node
{
public:

    // --------------------------------------------------------
    // Constructor
    // --------------------------------------------------------

    TCPSocketNode()
    : Node("tcp_socket_node"),
      socket_fd_(-1),
      running_(true)
    {
        // ----------------------------------------------------
        // Declare ROS parameters
        // ----------------------------------------------------

        server_ip_ = this->declare_parameter<std::string>(
            "server_ip",
            "172.25.64.1"
        );

        server_port_ = this->declare_parameter<int>(
            "server_port",
            5000
        );

        tx_topic_ = this->declare_parameter<std::string>(
            "tx_topic",
            "/tcp_socket/send"
        );

        rx_topic_ = this->declare_parameter<std::string>(
            "rx_topic",
            "/tcp_socket/receive"
        );

        reconnect_period_ = this->declare_parameter<double>(
            "reconnect_period",
            2.0
        );

        receive_buffer_size_ = this->declare_parameter<int>(
            "receive_buffer_size",
            4096
        );

        // ----------------------------------------------------
        // Validate parameters
        // ----------------------------------------------------

        if (server_port_ < 1 || server_port_ > 65535)
        {
            throw std::invalid_argument(
                "server_port must be between 1 and 65535"
            );
        }

        if (reconnect_period_ <= 0.0)
        {
            throw std::invalid_argument(
                "reconnect_period must be greater than zero"
            );
        }

        if (receive_buffer_size_ <= 0)
        {
            throw std::invalid_argument(
                "receive_buffer_size must be greater than zero"
            );
        }

        // ----------------------------------------------------
        // ROS publisher
        // ----------------------------------------------------

        rx_publisher_ =
            this->create_publisher<std_msgs::msg::String>(
                rx_topic_,
                10
            );

        // ----------------------------------------------------
        // ROS subscriber
        // ----------------------------------------------------

        tx_subscriber_ =
            this->create_subscription<std_msgs::msg::String>(
                tx_topic_,
                10,
                std::bind(
                    &TCPSocketNode::send_message_callback,
                    this,
                    std::placeholders::_1
                )
            );

        // ----------------------------------------------------
        // Start TCP worker thread
        // ----------------------------------------------------

        connection_thread_ = std::thread(
            &TCPSocketNode::connection_loop,
            this
        );

        // ----------------------------------------------------
        // Initialization logs
        // ----------------------------------------------------

        RCLCPP_INFO(
            this->get_logger(),
            "TCP Socket Node initialized."
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Target server: %s:%d",
            server_ip_.c_str(),
            server_port_
        );

        RCLCPP_INFO(
            this->get_logger(),
            "TX topic: %s",
            tx_topic_.c_str()
        );

        RCLCPP_INFO(
            this->get_logger(),
            "RX topic: %s",
            rx_topic_.c_str()
        );
    }


    // --------------------------------------------------------
    // Destructor
    // --------------------------------------------------------

    ~TCPSocketNode() override
    {
        running_ = false;

        close_socket();

        if (connection_thread_.joinable())
        {
            connection_thread_.join();
        }
    }


private:

    // ========================================================
    // ROS PARAMETERS
    // ========================================================

    std::string server_ip_;
    int server_port_;

    std::string tx_topic_;
    std::string rx_topic_;

    double reconnect_period_;
    int receive_buffer_size_;


    // ========================================================
    // ROS INTERFACES
    // ========================================================

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
        rx_publisher_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        tx_subscriber_;


    // ========================================================
    // TCP STATE
    // ========================================================

    int socket_fd_;

    std::atomic<bool> running_;

    std::mutex socket_mutex_;

    std::thread connection_thread_;


    // ========================================================
    // CONNECTION LOOP
    // ========================================================

    void connection_loop()
    {
        while (running_)
        {
            if (!connect_to_server())
            {
                wait_before_reconnect();
                continue;
            }

            RCLCPP_INFO(
                this->get_logger(),
                "TCP connection established."
            );

            receive_loop();

            close_socket();

            if (running_)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "TCP connection lost. Reconnecting..."
                );

                wait_before_reconnect();
            }
        }
    }


    // ========================================================
    // CONNECT TO SERVER
    // ========================================================

    bool connect_to_server()
    {
        int new_socket = socket(
            AF_INET,
            SOCK_STREAM,
            0
        );

        if (new_socket < 0)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to create socket: %s",
                std::strerror(errno)
            );

            return false;
        }

        sockaddr_in server_address{};

        server_address.sin_family = AF_INET;
        server_address.sin_port = htons(server_port_);

        // ----------------------------------------------------
        // Convert IPv4 address
        // ----------------------------------------------------

        if (inet_pton(
                AF_INET,
                server_ip_.c_str(),
                &server_address.sin_addr
            ) <= 0)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Invalid IPv4 address: %s",
                server_ip_.c_str()
            );

            close(new_socket);
            return false;
        }

        // ----------------------------------------------------
        // Establish connection
        // ----------------------------------------------------

        RCLCPP_INFO(
            this->get_logger(),
            "Connecting to %s:%d ...",
            server_ip_.c_str(),
            server_port_
        );

        if (connect(
                new_socket,
                reinterpret_cast<sockaddr*>(&server_address),
                sizeof(server_address)
            ) < 0)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Connection failed: %s",
                std::strerror(errno)
            );

            close(new_socket);
            return false;
        }

        // ----------------------------------------------------
        // Store socket
        // ----------------------------------------------------

        {
            std::lock_guard<std::mutex> lock(socket_mutex_);
            socket_fd_ = new_socket;
        }

        return true;
    }


    // ========================================================
    // RECEIVE LOOP
    // ========================================================

    void receive_loop()
    {
        std::string receive_buffer;
        receive_buffer.resize(receive_buffer_size_);

        while (running_)
        {
            int current_socket;

            {
                std::lock_guard<std::mutex> lock(socket_mutex_);

                if (socket_fd_ < 0)
                {
                    break;
                }

                current_socket = socket_fd_;
            }

            // ------------------------------------------------
            // Receive TCP data
            // ------------------------------------------------

            const ssize_t bytes_received = recv(
                current_socket,
                receive_buffer.data(),
                receive_buffer.size(),
                0
            );

            if (bytes_received > 0)
            {
                std_msgs::msg::String message;

                message.data.assign(
                    receive_buffer.data(),
                    static_cast<std::size_t>(bytes_received)
                );

                rx_publisher_->publish(message);

                RCLCPP_DEBUG(
                    this->get_logger(),
                    "Received %zd bytes.",
                    bytes_received
                );
            }
            else if (bytes_received == 0)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Remote server closed the connection."
                );

                break;
            }
            else
            {
                if (errno == EINTR)
                {
                    continue;
                }

                RCLCPP_WARN(
                    this->get_logger(),
                    "TCP receive error: %s",
                    std::strerror(errno)
                );

                break;
            }
        }
    }


    // ========================================================
    // SEND MESSAGE CALLBACK
    // ========================================================

    void send_message_callback(
        const std_msgs::msg::String::SharedPtr msg)
    {
        // ----------------------------------------------------
        // Validate message
        // ----------------------------------------------------

        if (msg->data.empty())
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Empty message. Not sent."
            );

            return;
        }

        // ----------------------------------------------------
        // Prepare message with newline terminator
        // ----------------------------------------------------

        std::string message = msg->data;

        // Append '\n' only if it is not already present.
        if (message.back() != '\n')
        {
            message += '\n';
        }

        // ----------------------------------------------------
        // Check socket connection
        // ----------------------------------------------------

        std::lock_guard<std::mutex> lock(socket_mutex_);

        if (socket_fd_ < 0)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "TCP disconnected. Message not sent."
            );

            return;
        }

        // ----------------------------------------------------
        // Send complete message
        // ----------------------------------------------------

        const char* data = message.data();

        const std::size_t total_bytes = message.size();

        std::size_t sent_bytes = 0;

        while (sent_bytes < total_bytes)
        {
            const ssize_t result = send(
                socket_fd_,
                data + sent_bytes,
                total_bytes - sent_bytes,
                MSG_NOSIGNAL
            );

            if (result < 0)
            {
                if (errno == EINTR)
                {
                    continue;
                }

                RCLCPP_ERROR(
                    this->get_logger(),
                    "TCP send error: %s",
                    std::strerror(errno)
                );

                close_socket_unlocked();
                return;
            }

            if (result == 0)
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "TCP send returned zero bytes."
                );

                close_socket_unlocked();
                return;
            }

            sent_bytes += static_cast<std::size_t>(result);
        }

        // ----------------------------------------------------
        // Send log
        // ----------------------------------------------------

        RCLCPP_DEBUG(
            this->get_logger(),
            "Sent %zu bytes, including newline terminator.",
            sent_bytes
        );
    }


    // ========================================================
    // SOCKET CLEANUP
    // ========================================================

    void close_socket()
    {
        std::lock_guard<std::mutex> lock(socket_mutex_);

        close_socket_unlocked();
    }


    void close_socket_unlocked()
    {
        if (socket_fd_ >= 0)
        {
            shutdown(socket_fd_, SHUT_RDWR);
            close(socket_fd_);

            socket_fd_ = -1;
        }
    }


    // ========================================================
    // RECONNECTION WAIT
    // ========================================================

    void wait_before_reconnect()
    {
        const auto wait_duration =
            std::chrono::duration<double>(reconnect_period_);

        const auto start_time =
            std::chrono::steady_clock::now();

        while (
            running_ &&
            std::chrono::steady_clock::now() - start_time
                < wait_duration
        )
        {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(100)
            );
        }
    }
};


// ============================================================
// MAIN
// ============================================================

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<TCPSocketNode>();

        rclcpp::spin(node);
    }
    catch (const std::exception& error)
    {
        RCLCPP_FATAL(
            rclcpp::get_logger("tcp_socket_node"),
            "Node initialization failed: %s",
            error.what()
        );
    }

    rclcpp::shutdown();

    return 0;
}

#ifndef COLAB_SUPERVISION_ABB_BRIGE__TCP_POSE_PARSER_NODE_HPP_
#define COLAB_SUPERVISION_ABB_BRIGE__TCP_POSE_PARSER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <colab_supervision_interfaces/msg/pose_command.hpp>
#include <colab_supervision_interfaces/msg/pose_cua_command.hpp>

#include <array>
#include <optional>
#include <string>

class TcpPoseParserNode : public rclcpp::Node
{
public:
  TcpPoseParserNode();

private:
  void receive_callback(const std_msgs::msg::String::SharedPtr msg);

  std::optional<std::array<double, 6>> parse_pose(const std::string & text) const;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr receive_subscription_;
  rclcpp::Publisher<colab_supervision_interfaces::msg::PoseCommand>::SharedPtr tcp_eul_publisher_;
  rclcpp::Publisher<colab_supervision_interfaces::msg::PoseCuaCommand>::SharedPtr tcp_cua_publisher_;

  bool cartesian_path_{false};
};

#endif  // COLAB_SUPERVISION_ABB_BRIGE__TCP_POSE_PARSER_NODE_HPP_

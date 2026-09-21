cmake_minimum_required(VERSION 3.8)
project(colab_supervision_abb_brige)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(colab_supervision_interfaces REQUIRED)

add_executable(tcp_socket_node src/socket.cpp)
ament_target_dependencies(tcp_socket_node
  rclcpp
  std_msgs
)

add_executable(command_bridge src/command_bridge.cpp)
ament_target_dependencies(command_bridge
  rclcpp
  std_msgs
  colab_supervision_interfaces
)

install(
  TARGETS tcp_socket_node command_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

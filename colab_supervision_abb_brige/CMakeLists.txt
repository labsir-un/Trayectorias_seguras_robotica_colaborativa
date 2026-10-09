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

add_executable(tcp_pose_parser src/tcp_pose_parser.cpp)
ament_target_dependencies(tcp_pose_parser
  rclcpp
  std_msgs
  colab_supervision_interfaces
)

target_include_directories(tcp_pose_parser PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


install(
  TARGETS tcp_socket_node command_bridge tcp_pose_parser
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()

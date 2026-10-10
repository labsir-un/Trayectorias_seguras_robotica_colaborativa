cmake_minimum_required(VERSION 3.8)
project(colab_supervision_metrics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(colab_supervision_interfaces REQUIRED)

add_executable(metrics_node
  src/metrics_node.cpp
)

target_include_directories(metrics_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(metrics
  rclcpp
  std_msgs
  moveit_msgs
  colab_supervision_interfaces
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(metrics
  "${cpp_typesupport_target}"
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS metrics
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

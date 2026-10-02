cmake_minimum_required(VERSION 3.16)
project(colab_supervision_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(colab_supervision_interfaces REQUIRED)

add_executable(control_supervisor src/control_supervisor.cpp)

target_include_directories(control_supervisor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(control_supervisor
  rclcpp
  std_msgs
  colab_supervision_interfaces
)

install(TARGETS control_supervisor
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()

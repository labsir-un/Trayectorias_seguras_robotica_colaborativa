cmake_minimum_required(VERSION 3.8)
project(colab_supervision_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(colab_supervision_interfaces REQUIRED)


add_executable(test_moveit src/test_moveit.cpp)
ament_target_dependencies(test_moveit 
    rclcpp
    moveit_ros_planning_interface
    )

add_executable(commander src/commander_template.cpp)
ament_target_dependencies(commander 
    rclcpp
    moveit_ros_planning_interface
    colab_supervision_interfaces
    )

install(TARGETS
  test_moveit
  commander
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()

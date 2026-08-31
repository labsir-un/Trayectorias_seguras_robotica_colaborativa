cmake_minimum_required(VERSION 3.8)
project(colab_supervision_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Joints.msg"
  "msg/MultiArrayDimension.msg"
  "msg/MultiArrayLayout.msg"
  "msg/Name.msg"
  "msg/OperatorPerception.msg"
  "msg/Pose.msg"
  "msg/PoseCommand.msg"
  "msg/RobotOperatorInteraction.msg"
  #"srv/ComputeRectangleArea.srv"
 )

ament_package()

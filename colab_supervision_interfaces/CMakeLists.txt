cmake_minimum_required(VERSION 3.8)
project(colab_supervision_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlCommand.msg"
  "msg/ExperimentalMetrics.msg"
  "msg/Joints.msg"
  "msg/MultiArrayDimension.msg"
  "msg/MultiArrayLayout.msg"
  "msg/Name.msg"
  "msg/OperatorPerception.msg"
  "msg/OperatorPosture.msg"
  "msg/OperatorStateAnalysis.msg"
  "msg/Pose.msg"
  "msg/PoseCommand.msg"
  "msg/PoseCua.msg"
  "msg/PoseCuaCommand.msg"
  "msg/RobotDetection.msg"
  "msg/RobotOperatorInteraction.msg"
  "msg/RobotOperatorSupervision.msg"
  #"srv/ComputeRectangleArea.srv"

   DEPENDENCIES
    std_msgs
    geometry_msgs
 )

ament_package()

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char **argv)
{   
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("test_moveit");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    auto irb140 = moveit::planning_interface::MoveGroupInterface(node, "arm_abb_irb_140");
    // irb140.setEndEffectorLink("tcp_joint");
    // RCLCPP_INFO(node->get_logger(), "TCP actual: %s", irb140.getEndEffectorLink().c_str());

    // auto end_effector = moveit::planning_interface::MoveGroupInterface(node, "prototipe_jsdaleman");

    irb140.setMaxVelocityScalingFactor(0.5);
    irb140.setMaxAccelerationScalingFactor(0.5);

    // //Name goal

    // irb140.setStartStateToCurrentState();
    // irb140.setNamedTarget("transition");

    // moveit::planning_interface::MoveGroupInterface::Plan plan1;
    // bool success = (irb140.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    // if (success) {
    //     irb140.execute(plan1);
    // }

    // irb140.setStartStateToCurrentState();
    // irb140.setNamedTarget("home");

    // moveit::planning_interface::MoveGroupInterface::Plan plan2;
    // bool success2 = (irb140.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);

    // if (success2) {
    //     irb140.execute(plan2);
    // }

    // -------------------------------------------------------------------------------

    // // Join Goal

    // std::vector<double> joints = {1.5, 0.5, 0.0, 1.5, 0.0, -0.7};

    // irb140.setStartStateToCurrentState();
    // irb140.setJointValueTarget(joints);

    // moveit::planning_interface::MoveGroupInterface::Plan plan1;
    // bool success = (irb140.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    // if (success) {
    //     irb140.execute(plan1);
    // }

    // Pose Goal

    tf2::Quaternion q;
    // q.setRPY(0.0, 1.57, 0.0);
    q.setRPY(3.14, 0.0, 0.0);
    q = q.normalize();

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.header.stamp = node->now();
    target_pose.pose.position.x = 0;
    target_pose.pose.position.y = -0.7;
    target_pose.pose.position.z = 0.4;
    target_pose.pose.orientation.x = q.getX();
    target_pose.pose.orientation.y = q.getY();
    target_pose.pose.orientation.z = q.getZ();
    target_pose.pose.orientation.w = q.getW();

    irb140.setStartStateToCurrentState();
    irb140.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success = (irb140.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        irb140.execute(plan1);
    }

    // Cartian Path

    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose pose1 = irb140.getCurrentPose().pose;
    pose1.position.z += -0.2;
    waypoints.push_back(pose1);

    moveit_msgs::msg:: RobotTrajectory trajectory;

    double fraction = irb140.computeCartesianPath(waypoints, 0.01, trajectory);

    if (fraction == 1){
        irb140.execute(trajectory);
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}
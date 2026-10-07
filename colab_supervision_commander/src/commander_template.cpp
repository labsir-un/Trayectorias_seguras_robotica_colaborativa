#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <colab_supervision_interfaces/msg/joints.hpp>
#include <colab_supervision_interfaces/msg/pose_command.hpp>
#include <colab_supervision_interfaces/msg/pose_cua_command.hpp>
#include <colab_supervision_interfaces/msg/name.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using FloatArray = colab_supervision_interfaces::msg::Joints;
using PoseCmd = colab_supervision_interfaces::msg::PoseCommand;
using PoseCuaCmd = colab_supervision_interfaces::msg::PoseCuaCommand;
using NameCmd = colab_supervision_interfaces::msg::Name;
using namespace std::placeholders;

class Commander
{
    public:
        Commander(std::shared_ptr<rclcpp::Node> node)
        {
            node_ = node;
            irb140_ = std::make_shared<MoveGroupInterface>(node_, "arm_abb_irb_140");
            irb140_->setEndEffectorLink("tcp_link");
            RCLCPP_INFO(node_->get_logger(), "TCP actual: %s", irb140_->getEndEffectorLink().c_str());
            irb140_->setMaxVelocityScalingFactor(0.5);
            irb140_->setMaxAccelerationScalingFactor(0.5);

            trajectory_pub_ =
                node_->create_publisher<moveit_msgs::msg::RobotTrajectory>(
                    "/control/trajectory", 10);

            named_target_sub_ = node_->create_subscription<NameCmd>(
                "name_command", 10, std::bind(&Commander::nameCmdCallback, this, _1));

            joint_cmd_sub_ = node_->create_subscription<FloatArray>(
                "joint_command", 10, std::bind(&Commander::jointCmdCallback, this, _1));

            pose_cmd_sub_ = node_->create_subscription<PoseCmd>(
                "pose_command", 10, std::bind(&Commander::poseCmdCallback, this, _1));

            posecua_cmd_sub_ = node_->create_subscription<PoseCuaCmd>(
                "posecua_command", 10, std::bind(&Commander::poseCuaCmdCallback, this, _1));
        }

        void goToNamedTarget(const std::string &name)
        {
            irb140_->setStartStateToCurrentState();
            irb140_->setNamedTarget(name);
            planAndExecute(irb140_);
        }

        void goToJointTarget(std::vector<double> &joints)
        {
            irb140_->setStartStateToCurrentState();
            irb140_->setJointValueTarget(joints);
            planAndExecute(irb140_);
        }

        void goToPoseTarge(double x, double y, double z,
                           double roll, double pitch, double yaw, bool cartesian_path=false)
        {
            tf2::Quaternion q;
            q.setRPY(roll, pitch, yaw);
            q = q.normalize();

            geometry_msgs::msg::PoseStamped target_pose;
            target_pose.header.frame_id = "base_link";
            target_pose.header.stamp = node_->now();
            target_pose.pose.position.x = x;
            target_pose.pose.position.y = y;
            target_pose.pose.position.z = z;
            target_pose.pose.orientation.x = q.getX();
            target_pose.pose.orientation.y = q.getY();
            target_pose.pose.orientation.z = q.getZ();
            target_pose.pose.orientation.w = q.getW();

            irb140_->setStartStateToCurrentState();
            if (!cartesian_path){
                irb140_->setPoseTarget(target_pose);
                planAndExecute(irb140_);
            }
            else{
                std::vector<geometry_msgs::msg::Pose> waypoints;
                waypoints.push_back(target_pose.pose);
                moveit_msgs::msg:: RobotTrajectory trajectory;

                double fraction = irb140_->computeCartesianPath(waypoints, 0.01, trajectory);

                if (fraction == 1){
                    trajectory_pub_->publish(trajectory);
                    irb140_->execute(trajectory);
                }
            }
        }

        void goToPoseTargeCua(double x, double y, double z,
                           double q1, double q2, double q3, double q4, bool cartesian_path=false)
        {

            geometry_msgs::msg::PoseStamped target_pose;
            target_pose.header.frame_id = "base_link";
            target_pose.header.stamp = node_->now();
            target_pose.pose.position.x = x;
            target_pose.pose.position.y = y;
            target_pose.pose.position.z = z;
            target_pose.pose.orientation.x = q1;
            target_pose.pose.orientation.y = q2;
            target_pose.pose.orientation.z = q3;
            target_pose.pose.orientation.w = q4;

            irb140_->setStartStateToCurrentState();
            if (!cartesian_path){
                irb140_->setPoseTarget(target_pose);
                planAndExecute(irb140_);
            }
            else{
                std::vector<geometry_msgs::msg::Pose> waypoints;
                waypoints.push_back(target_pose.pose);
                moveit_msgs::msg:: RobotTrajectory trajectory;

                double fraction = irb140_->computeCartesianPath(waypoints, 0.01, trajectory);

                if (fraction == 1){
                    trajectory_pub_->publish(trajectory);
                    irb140_->execute(trajectory);
                }
            }
        }

    private:
        
        void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface)
        {
            MoveGroupInterface::Plan plan;
            bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

            if (success) {
                trajectory_pub_->publish(plan.trajectory);
                interface->execute(plan);
            }
        }

        void nameCmdCallback(const NameCmd &msg)
        {
            goToNamedTarget(msg.name);
        }

        void jointCmdCallback(const FloatArray &msg)
        {
            auto joints = msg.data;

            if (joints.size() == 6) {
                goToJointTarget(joints);
            }
        }

        void poseCmdCallback(const PoseCmd &msg)
        {
            goToPoseTarge(msg.pose.x, msg.pose.y, msg.pose.z,
                        msg.pose.roll, msg.pose.pitch, msg.pose.yaw,
                        msg.cartesian_path);
        }

        void poseCuaCmdCallback(const PoseCuaCmd &msg)
        {
            goToPoseTargeCua(msg.pose.x, msg.pose.y, msg.pose.z,
                        msg.pose.q1, msg.pose.q2, msg.pose.q3, msg.pose.q4,
                        msg.cartesian_path);
        }

        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<MoveGroupInterface> irb140_;

        rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr trajectory_pub_;
        
        rclcpp::Subscription<NameCmd>::SharedPtr named_target_sub_;
        rclcpp::Subscription<FloatArray>::SharedPtr joint_cmd_sub_;
        rclcpp::Subscription<PoseCmd>::SharedPtr pose_cmd_sub_;
        rclcpp::Subscription<PoseCuaCmd>::SharedPtr posecua_cmd_sub_;
};

int main(int argc, char **argv)
{   
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("commander");
    auto commander = Commander(node);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
#!/usr/bin/env python3
import rclpy
import tf_transformations
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from tf2_msgs.msg import TFMessage

class MoveCamera(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("move_camera") # MODIFY NAME
        self.declare_parameter("camera_namespace", "camera")
        self.declare_parameter("camera_name", "camera")
        self.camera_namespace_ = self.get_parameter("camera_namespace").value
        self.camera_name_ = self.get_parameter("camera_name").value
        self.subscriber_ = self.create_subscription(Twist, f"move_camera/{self.camera_namespace_}/{self.camera_name_}", self.callback_move_camera_tf, 10)
        self.publisher_ = self.create_publisher(TFMessage, f"tf_static", 10)
        self.get_logger().info("Move Camera node has been started.")

    def callback_move_camera_tf(self, msg: Twist):
        q = tf_transformations.quaternion_from_euler(msg.angular.x, msg.angular.y, msg.angular.z)
        t = TransformStamped()
        tf_msg = TFMessage()

        t.transform.translation.x = msg.linear.x
        t.transform.translation.y = msg.linear.y
        t.transform.translation.z = msg.linear.z
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        tf_msg.transforms.append(t)
        self.publisher_.publish(tf_msg)
        self.get_logger().info(f"Published camera movement: {tf_msg.transforms}")


def main(args=None):
    rclpy.init(args=args)
    node = MoveCamera() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

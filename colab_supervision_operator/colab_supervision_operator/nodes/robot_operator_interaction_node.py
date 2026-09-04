"""
ROS 2 node for robot-operator interaction analysis.

This node coordinates the RealSense D435i perception,
robot state acquisition and collision-risk estimation.
"""

from typing import Optional
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from colab_supervision_operator.perception.depth_person_detector import DepthPersonDetector
from colab_supervision_operator.perception.robot_state import RobotState
from colab_supervision_operator.perception.collision_estimator import CollisionEstimator

class RobotOperatorInteractionNode(Node):
    """
    ROS 2 node responsible for robot-operator interaction.

    The node coordinates:

        - RealSense D435i operator perception
        - ABB robot state
        - Collision-risk estimation

    Domain logic is implemented by the perception modules.
    """

    def __init__(self) -> None:
        super().__init__("robot_operator_interaction_node")

        # ==================================================
        # PARAMETERS
        # ==================================================

        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_link")
        self.declare_parameter("operator_detection_rate", 30.0)

        # ==================================================
        # READ PARAMETERS
        # ==================================================

        self._joint_states_topic = self.get_parameter("joint_states_topic").value
        self._robot_base_frame = self.get_parameter("robot_base_frame").value
        self._camera_frame = self.get_parameter("camera_frame").value
        self._operator_detection_rate = self.get_parameter("operator_detection_rate").value

        # ==================================================
        # PERCEPTION MODULES
        # ==================================================

        self._depth_person_detector = DepthPersonDetector()
        self._robot_state =  RobotState(number_of_joints=6)
        self._collision_estimator = CollisionEstimator()

        # ==================================================
        # CURRENT OPERATOR STATE
        # ==================================================

        self._operator_position: Optional[np.ndarray] = None
        self._operator_velocity: Optional[np.ndarray] = None
        self._operator_confidence = 0.0

        # ==================================================
        # ROS 2 SUBSCRIBERS
        # ==================================================

        self._joint_state_subscription = self.create_subscription(JointState, self._joint_states_topic, self._joint_state_callback, 10)

        # ==================================================
        # TIMER
        # ==================================================

        timer_period =  1.0 / self._operator_detection_rate
        self._timer = self.create_timer(timer_period, self._process_interaction)

        self.get_logger().info("Robot operator interaction node initialized")
        self.get_logger().info(f"Joint states topic: {self._joint_states_topic}")
        self.get_logger().info(f"Robot base frame: {self._robot_base_frame}")
        self.get_logger().info(f"Camera frame: {self._camera_frame}")

    # ======================================================
    # JOINT STATE CALLBACK
    # ======================================================

    def _joint_state_callback(self,msg: JointState) -> None:
        """
        Process the ABB robot joint state.
        """

        if len(msg.position) != 6:
            self.get_logger().warning(f"Expected 6 robot joints, received {len(msg.position)}")
            return

        joint_positions = np.asarray(msg.position, dtype=float)

        if len(msg.velocity) == 6:
            joint_velocities = np.asarray(msg.velocity, dtype=float)

        else:
            joint_velocities = np.zeros(6, dtype=float)

        self._robot_state.update_joint_state(joint_positions, joint_velocities)

    # ======================================================
    # INTERACTION PROCESSING
    # ======================================================

    def _process_interaction(self) -> None:
        """
        Process the current robot-operator interaction.
        """
        robot_result = self._robot_state.get_result()

        if not robot_result.state_valid:
            self.get_logger().debug("Robot state is not available")
            return

        # --------------------------------------------------
        # Obtain operator observation
        # --------------------------------------------------

        operator_observation = self._get_operator_observation()

        if operator_observation is None:
            self.get_logger().debug("Operator observation unavailable")
            return

        position = operator_observation["position"]
        confidence = operator_observation["confidence"]
        timestamp = self.get_clock().now().nanoseconds / 1e9

        # --------------------------------------------------
        # Update depth detector
        # --------------------------------------------------

        depth_result = self._depth_person_detector.update(
            position=position,
            confidence=confidence,
            timestamp=timestamp
            )

        if not depth_result.detection_valid:
            self.get_logger().debug("Operator depth detection invalid")
            return

        if not depth_result.person_detected:
            self.get_logger().debug("Operator not detected")
            return

        # --------------------------------------------------
        # Operator state
        # --------------------------------------------------

        operator_position = depth_result.position
        operator_velocity = depth_result.velocity

        if operator_position is None:
            return

        if operator_velocity is None:
            operator_velocity = np.zeros(3, dtype=float)

        # --------------------------------------------------
        # Robot state
        # --------------------------------------------------

        robot_position = robot_result.tcp_position
        robot_velocity = robot_result.tcp_linear_velocity

        if robot_position is None:
            self.get_logger().debug("Robot TCP position unavailable")
            return

        if robot_velocity is None:
            robot_velocity = np.zeros(3, dtype=float)

        # --------------------------------------------------
        # Collision estimation
        # --------------------------------------------------

        collision_result = self._collision_estimator.update(
            robot_position=robot_position,
            robot_velocity=robot_velocity,
            operator_position=operator_position,
            operator_velocity=operator_velocity
            )

        # --------------------------------------------------
        # Publish result
        # --------------------------------------------------

        self._publish_interaction_result(collision_result)

    # ======================================================
    # OPERATOR PERCEPTION INTERFACE
    # ======================================================

    def _get_operator_observation(self) -> Optional[dict]:
        """
        Obtain the current operator observation from the
        RealSense D435i.

        This method will later be connected to the
        RealSense perception pipeline.
        """
        return None

    # ======================================================
    # ROS 2 OUTPUT
    # ======================================================

    def _publish_interaction_result(self, collision_result) -> None:
        """
        Publish the robot-operator interaction result.

        Custom ROS 2 messages will be added once the message
        interface is defined.
        """
        self.get_logger().debug(f"Collision risk: {collision_result.collision_risk}, distance={collision_result.distance:.3f} m, TTC={collision_result.time_to_collision}")


def main(args=None) -> None:

    rclpy.init(args=args)
    node = RobotOperatorInteractionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":

    main()
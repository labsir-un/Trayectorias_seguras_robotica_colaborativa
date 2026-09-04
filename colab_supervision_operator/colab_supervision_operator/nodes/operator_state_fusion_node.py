"""
ROS 2 node for operator state fusion.

This node combines operator fatigue, attention and
robot-operator interaction information into a unified
operator state.
"""

from dataclasses import dataclass
from typing import Optional
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import String


@dataclass
class OperatorState:
    """
    Unified state of the operator.

    This structure represents the information generated
    by the perception nodes.
    """

    fatigue_level: str
    fatigue_score: float

    attention_target: str
    attention_score: float

    operator_present: bool

    robot_distance: Optional[float]
    approach_velocity: Optional[float]

    time_to_collision: Optional[float]

    collision_risk: str
    collision_risk_score: float


class OperatorStateFusionNode(Node):
    """
    ROS 2 node responsible for fusing operator perception
    and robot-operator interaction information.
    """

    def __init__(self) -> None:
        super().__init__("operator_state_fusion_node")

        # ==================================================
        # PARAMETERS
        # ==================================================

        self.declare_parameter("fatigue_level_topic", "/operator/fatigue/level")
        self.declare_parameter("fatigue_score_topic", "/operator/fatigue/score")
        self.declare_parameter("attention_target_topic", "/operator/attention/target")
        self.declare_parameter("attention_score_topic", "/operator/attention/score")
        self.declare_parameter("operator_present_topic", "/robot_operator/operator_present")
        self.declare_parameter("robot_distance_topic", "/robot_operator/distance")
        self.declare_parameter("approach_velocity_topic", "/robot_operator/approach_velocity")
        self.declare_parameter("ttc_topic", "/robot_operator/ttc")
        self.declare_parameter("collision_risk_topic", "/robot_operator/collision_risk")
        self.declare_parameter("collision_risk_score_topic", "/robot_operator/collision_risk_score")
        self.declare_parameter("operator_state_topic","/operator/state")

        # ==================================================
        # READ PARAMETERS
        # ==================================================

        self._fatigue_level_topic = self.get_parameter("fatigue_level_topic").value
        self._fatigue_score_topic = self.get_parameter("fatigue_score_topic").value
        self._attention_target_topic = self.get_parameter("attention_target_topic").value
        self._attention_score_topic = self.get_parameter("attention_score_topic").value
        self._operator_present_topic = self.get_parameter("operator_present_topic").value
        self._robot_distance_topic = self.get_parameter("robot_distance_topic").value
        self._approach_velocity_topic = self.get_parameter("approach_velocity_topic").value
        self._ttc_topic = self.get_parameter("ttc_topic").value
        self._collision_risk_topic = self.get_parameter("collision_risk_topic").value
        self._collision_risk_score_topic = self.get_parameter("collision_risk_score_topic").value
        self._operator_state_topic = self.get_parameter("operator_state_topic").value

        # ==================================================
        # CURRENT FATIGUE STATE
        # ==================================================

        self._fatigue_level = "UNKNOWN"
        self._fatigue_score = 0.0

        # ==================================================
        # CURRENT ATTENTION STATE
        # ==================================================

        self._attention_target = "UNKNOWN"
        self._attention_score = 0.0

        # ==================================================
        # CURRENT OPERATOR STATE
        # ==================================================

        self._operator_present = False

        # ==================================================
        # CURRENT ROBOT INTERACTION STATE
        # ==================================================

        self._robot_distance: Optional[float] = None
        self._approach_velocity: Optional[float] = None
        self._time_to_collision: Optional[float] = None
        self._collision_risk = "UNKNOWN"
        self._collision_risk_score = 0.0

        # ==================================================
        # SUBSCRIBERS
        # ==================================================

        self._fatigue_level_subscription = self.create_subscription(
            String,
            self._fatigue_level_topic,
            self._fatigue_level_callback,
            10
        )

        self._fatigue_score_subscription = self.create_subscription(
            Float32,
            self._fatigue_score_topic,
            self._fatigue_score_callback,
            10
        )

        self._attention_target_subscription = self.create_subscription(
            String,
            self._attention_target_topic,
            self._attention_target_callback,
            10
        )

        self._attention_score_subscription = self.create_subscription(
            Float32,
            self._attention_score_topic,
            self._attention_score_callback,
            10
        )

        self._operator_present_subscription = self.create_subscription(
            Bool,
            self._operator_present_topic,
            self._operator_present_callback,
            10
        )

        self._robot_distance_subscription = self.create_subscription(
            Float32,
            self._robot_distance_topic,
            self._robot_distance_callback,
            10
        )

        self._approach_velocity_subscription = self.create_subscription(
            Float32,
            self._approach_velocity_topic,
            self._approach_velocity_callback,
            10
        )

        self._ttc_subscription = self.create_subscription(
            Float32,
            self._ttc_topic,
            self._ttc_callback,
            10
        )

        self._collision_risk_subscription = self.create_subscription(
            String,
            self._collision_risk_topic,
            self._collision_risk_callback,
            10
        )

        self._collision_risk_score_subscription = self.create_subscription(
            Float32,
            self._collision_risk_score_topic,
            self._collision_risk_score_callback,
            10
        )

        # ==================================================
        # PUBLISHER
        # ==================================================

        self._operator_state_publisher = self.create_publisher(String, self._operator_state_topic, 10)

        # ==================================================
        # TIMER
        # ==================================================

        self._fusion_timer = self.create_timer(0.05, self._publish_fused_state)
        self.get_logger().info("Operator state fusion node initialized")

    # ======================================================
    # CALLBACKS - FATIGUE
    # ======================================================

    def _fatigue_level_callback(self, msg: String) -> None:
        self._fatigue_level = msg.data

    def _fatigue_score_callback(self, msg: Float32) -> None:
        self._fatigue_score = msg.data

    # ======================================================
    # CALLBACKS - ATTENTION
    # ======================================================

    def _attention_target_callback(self, msg: String) -> None:
        self._attention_target = msg.data

    def _attention_score_callback(self, msg: Float32) -> None:
        self._attention_score = msg.data

    # ======================================================
    # CALLBACK - OPERATOR
    # ======================================================

    def _operator_present_callback(self, msg: Bool) -> None:
        self._operator_present = msg.data

    # ======================================================
    # CALLBACKS - ROBOT INTERACTION
    # ======================================================

    def _robot_distance_callback(self, msg: Float32) -> None:
        self._robot_distance = msg.data

    def _approach_velocity_callback(self, msg: Float32) -> None:
        self._approach_velocity = msg.data

    def _ttc_callback(self, msg: Float32) -> None:
        self._time_to_collision = msg.data

    def _collision_risk_callback(self, msg: String) -> None:
        self._collision_risk = msg.data

    def _collision_risk_score_callback(self, msg: Float32) -> None:
        self._collision_risk_score = msg.data

    # ======================================================
    # FUSION
    # ======================================================

    def _create_operator_state(self) -> OperatorState:
        """
        Create the current unified operator state.
        """

        return OperatorState(
            fatigue_level=self._fatigue_level,
            fatigue_score=self._fatigue_score,
            attention_target=self._attention_target,
            attention_score=self._attention_score,
            operator_present=self._operator_present,
            robot_distance=self._robot_distance,
            approach_velocity=self._approach_velocity,
            time_to_collision=self._time_to_collision,
            collision_risk=self._collision_risk,
            collision_risk_score=self._collision_risk_score
        )

    # ======================================================
    # PUBLISH
    # ======================================================

    def _publish_fused_state(self) -> None:
        """
        Publish the current fused operator state.

        A custom ROS 2 message should replace the temporary
        String message in the final implementation.
        """

        state = self._create_operator_state()

        message = String()
        message.data = f"fatigue_level={state.fatigue_level};fatigue_score={state.fatigue_score:.3f}; attention_target={state.attention_target}; attention_score={state.attention_score:.3f}; operator_present={state.operator_present}; robot_distance={state.robot_distance}; approach_velocity={state.approach_velocity}; ttc={state.time_to_collision}; collision_risk={state.collision_risk}; collision_risk_score={state.collision_risk_score:.3f}"

        self._operator_state_publisher.publish(message)


def main(args=None) -> None:

    rclpy.init(args=args)
    node = OperatorStateFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
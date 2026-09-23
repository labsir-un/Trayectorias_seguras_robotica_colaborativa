"""
ROS 2 node for operator fatigue analysis.

This node acquires images from an RGB camera and coordinates
the perception modules used to estimate operator fatigue
and attention.
"""

from typing import Optional
import cv2
import rclpy
from rclpy.node import Node
from colab_supervision_operator.perception.fatigue_analyzer import FatigueAnalyzer
from colab_supervision_operator.perception.blink_analyzer import BlinkAnalyzer
from colab_supervision_operator.perception.expression_analyzer import ExpressionAnalyzer
from colab_supervision_operator.perception.head_pose_estimator import HeadPoseEstimator
from colab_supervision_operator.perception.attention_analyzer import AttentionAnalyzer
from colab_supervision_operator.perception.person_tracker import PersonTracker


class OperatorFatigueNode(Node):
    """
    ROS 2 node responsible for RGB-based operator perception.

    The node acquires images from an RGB camera and coordinates
    the perception modules responsible for:

        - Person detection and tracking
        - Eye openness
        - Blink detection
        - Facial expression
        - Head pose
        - Attention
        - Fatigue

    The node is responsible for ROS 2 communication and sensor
    acquisition. Domain logic remains inside the perception
    modules.
    """

    def __init__(self) -> None:
        super().__init__("operator_fatigue_node")

        # ==================================================
        # PARAMETERS
        # ==================================================

        self.declare_parameter("camera_device", "/dev/video0")
        self.declare_parameter("camera_width", 1280)
        self.declare_parameter("camera_height", 720)
        self.declare_parameter("camera_fps",30.0)

        # ==================================================
        # READ PARAMETERS
        # ==================================================

        self._camera_device = self.get_parameter("camera_device").value
        self._camera_width = self.get_parameter("camera_width").value
        self._camera_height = self.get_parameter("camera_height").value
        self._camera_fps = self.get_parameter("camera_fps").value

        # ==================================================
        # PERCEPTION MODULES
        # ==================================================

        self._person_tracker = PersonTracker()
        self._blink_analyzer = BlinkAnalyzer()
        self._expression_analyzer = ExpressionAnalyzer()
        self._head_pose_estimator = HeadPoseEstimator()
        self._attention_analyzer = AttentionAnalyzer()
        self._fatigue_analyzer = FatigueAnalyzer()

        # ==================================================
        # CAMERA
        # ==================================================

        self._camera: Optional[cv2.VideoCapture] = None
        self._initialize_camera()

        # ==================================================
        # TIMER
        # ==================================================

        timer_period = 1.0 / self._camera_fps
        self._timer = self.create_timer(timer_period, self._process_frame)
        self.get_logger().info("Operator fatigue node initialized")
        self.get_logger().info(f"Camera device: {self._camera_device}")

    # ======================================================
    # CAMERA
    # ======================================================

    def _initialize_camera(self) -> None:
        """
        Initialize the RGB camera.
        """

        self._camera = cv2.VideoCapture(self._camera_device, cv2.CAP_V4L2)

        if not self._camera.isOpened():
            self.get_logger().error(f"Unable to open RGB camera: {self._camera_device}")
            self._camera = None
            return

        self._camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self._camera.set(cv2.CAP_PROP_FRAME_WIDTH, self._camera_width)
        self._camera.set(cv2.CAP_PROP_FRAME_HEIGHT,self._camera_height)
        self._camera.set(cv2.CAP_PROP_FPS, self._camera_fps)
        self.get_logger().info("RGB camera opened successfully")

        # --------------------------------------------------
        # Read negotiated configuration
        # --------------------------------------------------


        width = self._camera.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = self._camera.get(cv2.CAP_PROP_FRAME_HEIGHT)
        fps = self._camera.get(cv2.CAP_PROP_FPS)
        fourcc = int(self._camera.get(cv2.CAP_PROP_FOURCC))

        fourcc_string = "".join(
            [
                chr((fourcc >> 0) & 0xFF),
                chr((fourcc >> 8) & 0xFF),
                chr((fourcc >> 16) & 0xFF),
                chr((fourcc >> 24) & 0xFF),
            ]
        )

        self.get_logger().info("RGB camera opened successfully")
        self.get_logger().info(f"Camera device: {self._camera_device}")
        self.get_logger().info(f"Camera resolution: {int(width)}x{int(height)}")
        self.get_logger().info(f"Camera FPS: {fps:.1f}")
        self.get_logger().info(f"Camera format: {fourcc_string}")

    # ======================================================
    # FRAME PROCESSING
    # ======================================================

    def _process_frame(self) -> None:
        """
        Acquire and process one RGB frame.
        """

        if self._camera is None:
            self.get_logger().warning("RGB camera is not available")
            return

        success, frame = (self._camera.read())

        if not success:
            self.get_logger().warning("Failed to read RGB frame")
            return

        # --------------------------------------------------
        # Person detection
        # --------------------------------------------------

        detections = self._detect_persons(frame)
        person_result =  self._person_tracker.update(detections)

        # --------------------------------------------------
        # No person detected
        # --------------------------------------------------

        if not person_result.person_detected:
            self.get_logger().debug("No operator detected")
            return

        # --------------------------------------------------
        # Select operator
        # --------------------------------------------------

        operator = self._person_tracker.get_primary_person()

        if operator is None:
            return

        # --------------------------------------------------
        # Extract operator face
        # --------------------------------------------------

        face_data = self._extract_face_data(frame, operator)

        if face_data is None:
            return

        # --------------------------------------------------
        # Blink analysis
        # --------------------------------------------------

        timestamp = self.get_clock().now().nanoseconds/ 1e9
        blink_result = self._blink_analyzer.update(face_data["eye_openness"], timestamp)

        # --------------------------------------------------
        # Expression analysis
        # --------------------------------------------------

        expression_result = (self._expression_analyzer.update(
            face_data["expression"],
            face_data["expression_confidence"]
        ))

        # --------------------------------------------------
        # Head pose
        # --------------------------------------------------

        head_pose_result = self._head_pose_estimator.update(face_data["image_points"],(
            frame.shape[1],
            frame.shape[0]
        ))

        if not head_pose_result.pose_valid:
            return

        # --------------------------------------------------
        # Attention analysis
        # --------------------------------------------------

        attention_result = self._attention_analyzer.update(
            head_pose_result.yaw,
            head_pose_result.pitch,
            head_pose_result.roll
        )

        # --------------------------------------------------
        # Fatigue analysis
        # --------------------------------------------------

        fatigue_result = self._fatigue_analyzer.update(
            blink_result.eye_openness,
            blink_result.blink_frequency,
            expression_result.expression
        )

        # --------------------------------------------------
        # Publish results
        # --------------------------------------------------

        self._publish_results(
            fatigue_result,
            attention_result
        )

    # ======================================================
    # PERCEPTION INTERFACE
    # ======================================================

    def _detect_persons(self, frame) -> list[dict]:
        """
        Detect persons in the RGB frame.

        This method will be connected to the selected
        perception model.
        """
        return []

    def _extract_face_data(self,frame, person) -> Optional[dict]:
        """
        Extract facial information from the detected operator.

        This method will later be connected to the facial
        landmark/expression perception pipeline.
        """
        return None

    # ======================================================
    # ROS 2 OUTPUT
    # ======================================================

    def _publish_results(self, fatigue_result, attention_result) -> None:
        """
        Publish fatigue and attention results.

        ROS 2 publishers will be added once the custom
        message definitions are established.
        """
        self.get_logger().debug(f"Fatigue: \"{fatigue_result.fatigue_level}, score={fatigue_result.fatigue_score:.2f}")
        self.get_logger().debug(f"Attention: {attention_result.attention_target}, score={attention_result.attention_score:.2f}")

    # ======================================================
    # SHUTDOWN
    # ======================================================

    def destroy_node(self) -> None:
        """
        Release the camera before shutting down the node.
        """
        if self._camera is not None:
            self._camera.release()
            self._camera = None

        super().destroy_node()


def main(args=None) -> None:

    rclpy.init(args=args)
    node = OperatorFatigueNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":

    main()
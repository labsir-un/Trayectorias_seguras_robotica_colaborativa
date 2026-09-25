#!/usr/bin/env python3

"""
ROS 2 node for RGB-D robot/operator interaction analysis.

Current responsibilities:
    - Receive the RealSense D435i RGB image.
    - Detect the operator in RGB using the existing PersonDetector.
    - Detect the robot in RGB using an application-specific model.
    - Receive the RealSense PointCloud2.
    - Convert RGB detections to 3D positions.
    - Transform 3D positions to the robot base frame using TF2.
    - Estimate operator/robot spatial interaction.
"""

from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, JointState, PointCloud2

import tf2_ros
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point

from colab_supervision_operator.perception.person_detector import (
    PersonDetector,
)
from colab_supervision_operator.perception.person_tracker import (
    PersonTracker,
)
from colab_supervision_operator.perception.depth_person_detector import (
    DepthPersonDetector,
)
from colab_supervision_operator.perception.robot_detector import (
    RobotDetector,
)
from colab_supervision_operator.perception.pointcloud_object_detector import (
    PointCloudObjectDetector,
)
from colab_supervision_operator.perception.robot_state import RobotState
from colab_supervision_operator.perception.collision_estimator import (
    CollisionEstimator,
)


class RobotOperatorInteractionNode(Node):
    """Coordinate RealSense RGB-D perception and interaction analysis."""

    def __init__(self) -> None:
        super().__init__("robot_operator_interaction_node")

        # ============================================================
        # ROS TOPICS / FRAMES
        # ============================================================

        self.declare_parameter(
            "rgb_topic",
            "/IRB140_Cain/D435i/color/image_raw",
        )
        self.declare_parameter(
            "pointcloud_topic",
            "/IRB140_Cain/D435i/depth/color/points",
        )
        self.declare_parameter(
            "joint_states_topic",
            "/joint_states",
        )
        self.declare_parameter(
            "robot_base_frame",
            "base_link",
        )
        self.declare_parameter(
            "robot_reference_frame",
            "tool0",
        )

        # ============================================================
        # GENERAL
        # ============================================================

        self.declare_parameter("processing_rate", 15.0)
        self.declare_parameter("visualizar", False)
        self.declare_parameter("visualizar_detecciones", True)
        self.declare_parameter("visualizar_posiciones_3d", True)

        # ============================================================
        # OPERATOR MODEL
        # ============================================================

        self.declare_parameter("operator_model", "yolo26n.pt")
        self.declare_parameter("operator_confidence", 0.50)
        self.declare_parameter("operator_imgsz", 640)
        self.declare_parameter("operator_device", "cpu")

        # ============================================================
        # ROBOT MODEL
        # ============================================================

        self.declare_parameter("robot_model", "")
        self.declare_parameter("robot_confidence", 0.50)
        self.declare_parameter("robot_imgsz", 640)
        self.declare_parameter("robot_device", "cpu")
        self.declare_parameter("robot_class_id", 0)

        # ============================================================
        # POINT CLOUD
        # ============================================================

        self.declare_parameter("pointcloud_sample_half_size", 8)
        self.declare_parameter("pointcloud_minimum_valid_points", 10)
        self.declare_parameter("pointcloud_maximum_range", 5.0)

        # ============================================================
        # PARAMETERS -> ATTRIBUTES
        # ============================================================

        self._rgb_topic = self.get_parameter("rgb_topic").value
        self._pointcloud_topic = self.get_parameter(
            "pointcloud_topic"
        ).value
        self._joint_states_topic = self.get_parameter(
            "joint_states_topic"
        ).value
        self._robot_base_frame = self.get_parameter(
            "robot_base_frame"
        ).value
        self._robot_reference_frame = self.get_parameter(
            "robot_reference_frame"
        ).value

        self._processing_rate = float(
            self.get_parameter("processing_rate").value
        )
        self._visualizar = bool(
            self.get_parameter("visualizar").value
        )
        self._visualizar_detecciones = bool(
            self.get_parameter("visualizar_detecciones").value
        )
        self._visualizar_posiciones_3d = bool(
            self.get_parameter("visualizar_posiciones_3d").value
        )

        # ============================================================
        # SENSOR INTERFACES
        # ============================================================

        self._bridge = CvBridge()

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer,
            self,
        )

        # ============================================================
        # OPERATOR RGB DETECTION
        # ============================================================

        operator_model = self.get_parameter("operator_model").value
        operator_confidence = float(
            self.get_parameter("operator_confidence").value
        )
        operator_imgsz = int(
            self.get_parameter("operator_imgsz").value
        )
        operator_device = self.get_parameter("operator_device").value

        if operator_device == "":
            operator_device = None

        self._person_detector = PersonDetector(
            model_path=operator_model,
            confidence_threshold=operator_confidence,
            image_size=operator_imgsz,
            device=operator_device,
        )
        self._person_tracker = PersonTracker()
        self._depth_person_detector = DepthPersonDetector()

        # ============================================================
        # ROBOT RGB DETECTION
        # ============================================================

        robot_model = self.get_parameter("robot_model").value
        robot_confidence = float(
            self.get_parameter("robot_confidence").value
        )
        robot_imgsz = int(
            self.get_parameter("robot_imgsz").value
        )
        robot_device = self.get_parameter("robot_device").value
        robot_class_id = int(
            self.get_parameter("robot_class_id").value
        )

        if robot_device == "":
            robot_device = None

        self._robot_detector: Optional[RobotDetector] = None

        if robot_model:
            self._robot_detector = RobotDetector(
                model_path=robot_model,
                confidence_threshold=robot_confidence,
                image_size=robot_imgsz,
                device=robot_device,
                robot_class_id=robot_class_id,
            )
        else:
            self.get_logger().warning(
                "robot_model no está configurado. "
                "La identificación RGB del robot queda deshabilitada."
            )

        # ============================================================
        # POINT CLOUD
        # ============================================================

        self._pointcloud_detector = PointCloudObjectDetector(
            sample_half_size=int(
                self.get_parameter(
                    "pointcloud_sample_half_size"
                ).value
            ),
            minimum_valid_points=int(
                self.get_parameter(
                    "pointcloud_minimum_valid_points"
                ).value
            ),
            maximum_range=float(
                self.get_parameter(
                    "pointcloud_maximum_range"
                ).value
            ),
        )

        # ============================================================
        # ROBOT STATE / COLLISION
        # ============================================================

        self._robot_state = RobotState(number_of_joints=6)
        self._collision_estimator = CollisionEstimator()

        # ============================================================
        # CURRENT DATA
        # ============================================================

        self._latest_rgb_frame: Optional[np.ndarray] = None
        self._latest_pointcloud: Optional[PointCloud2] = None

        self._operator_position_base: Optional[np.ndarray] = None
        self._robot_position_rgb_base: Optional[np.ndarray] = None

        self._last_robot_reference_position: Optional[np.ndarray] = None
        self._last_robot_reference_timestamp: Optional[float] = None

        # ============================================================
        # SUBSCRIPTIONS
        # ============================================================

        self._rgb_subscription = self.create_subscription(
            Image,
            self._rgb_topic,
            self._rgb_callback,
            10,
        )

        self._pointcloud_subscription = self.create_subscription(
            PointCloud2,
            self._pointcloud_topic,
            self._pointcloud_callback,
            10,
        )

        self._joint_state_subscription = self.create_subscription(
            JointState,
            self._joint_states_topic,
            self._joint_state_callback,
            10,
        )

        # ============================================================
        # TIMER
        # ============================================================

        timer_period = 1.0 / self._processing_rate
        self._timer = self.create_timer(
            timer_period,
            self._process_scene,
        )

        self.get_logger().info(
            "=================================================="
        )
        self.get_logger().info(
            "Robot Operator Interaction Node iniciado"
        )
        self.get_logger().info(
            f"RGB topic: {self._rgb_topic}"
        )
        self.get_logger().info(
            f"PointCloud2 topic: {self._pointcloud_topic}"
        )
        self.get_logger().info(
            f"Robot base frame: {self._robot_base_frame}"
        )
        self.get_logger().info(
            f"Robot reference frame: {self._robot_reference_frame}"
        )
        self.get_logger().info(
            f"Procesamiento: {self._processing_rate:.1f} Hz"
        )
        self.get_logger().info(
            "=================================================="
        )

    # ================================================================
    # CALLBACKS
    # ================================================================

    def _rgb_callback(self, msg: Image) -> None:
        """Store the latest RealSense RGB frame."""

        try:
            self._latest_rgb_frame = self._bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )
        except CvBridgeError as exc:
            self.get_logger().error(
                f"Error convirtiendo imagen RGB: {exc}"
            )

    def _pointcloud_callback(self, msg: PointCloud2) -> None:
        """Store the latest RealSense PointCloud2."""

        self._latest_pointcloud = msg

    def _joint_state_callback(self, msg: JointState) -> None:
        """Update robot joint state."""

        if len(msg.position) != 6:
            self.get_logger().warning(
                "Se esperaban 6 articulaciones; "
                f"se recibieron {len(msg.position)}."
            )
            return

        positions = np.asarray(
            msg.position,
            dtype=float,
        )

        if len(msg.velocity) == 6:
            velocities = np.asarray(
                msg.velocity,
                dtype=float,
            )
        else:
            velocities = np.zeros(
                6,
                dtype=float,
            )

        self._robot_state.update_joint_state(
            positions,
            velocities,
        )

    # ================================================================
    # SCENE PROCESSING
    # ================================================================

    def _process_scene(self) -> None:
        """Process the latest RGB-D scene."""

        if self._latest_rgb_frame is None:
            return

        if self._latest_pointcloud is None:
            return

        frame = self._latest_rgb_frame.copy()
        cloud = self._latest_pointcloud

        timestamp = (
            self.get_clock().now().nanoseconds / 1e9
        )

        # ============================================================
        # OPERATOR DETECTION
        # ============================================================

        try:
            persons = self._person_detector.detect(frame)
            tracking_result = self._person_tracker.update(persons)
        except Exception as exc:
            self.get_logger().error(
                f"Error detectando operador: {exc}"
            )
            return

        primary_person = (
            self._person_tracker.get_primary_person()
            if tracking_result.person_detected
            else None
        )

        # ============================================================
        # OPERATOR 3D
        # ============================================================

        operator_position_base = None

        if primary_person is not None:
            operator_bbox = (
                primary_person.x1,
                primary_person.y1,
                primary_person.x2,
                primary_person.y2,
            )

            operator_3d = self._pointcloud_detector.bbox_to_3d(
                cloud,
                operator_bbox,
            )

            if operator_3d.position is not None:
                operator_position_base = (
                    self._transform_position_to_base(
                        operator_3d.position,
                        cloud.header.frame_id,
                    )
                )

                if operator_position_base is not None:
                    self._operator_position_base = (
                        operator_position_base
                    )

        operator_depth_result = self._depth_person_detector.update(
            position=operator_position_base,
            confidence=(
                1.0
                if operator_position_base is not None
                else 0.0
            ),
            timestamp=timestamp,
        )

        # ============================================================
        # ROBOT RGB DETECTION
        # ============================================================

        robot_position_base = None
        robot_detection = None

        if self._robot_detector is not None:
            try:
                robot_detections = self._robot_detector.detect(
                    frame
                )

                if robot_detections:
                    robot_detection = robot_detections[0]

                    robot_3d = (
                        self._pointcloud_detector.bbox_to_3d(
                            cloud,
                            robot_detection.bbox,
                        )
                    )

                    if robot_3d.position is not None:
                        robot_position_base = (
                            self._transform_position_to_base(
                                robot_3d.position,
                                cloud.header.frame_id,
                            )
                        )

                        if robot_position_base is not None:
                            self._robot_position_rgb_base = (
                                robot_position_base
                            )

            except Exception as exc:
                self.get_logger().error(
                    f"Error detectando robot RGB: {exc}"
                )

        # ============================================================
        # ROBOT REFERENCE FROM TF
        # ============================================================

        robot_reference_position = (
            self._get_robot_reference_position()
        )

        # ============================================================
        # COLLISION ESTIMATION
        # ============================================================

        if (
            robot_reference_position is not None
            and operator_depth_result.position is not None
        ):
            operator_position = operator_depth_result.position

            operator_velocity = operator_depth_result.velocity
            if operator_velocity is None:
                operator_velocity = np.zeros(
                    3,
                    dtype=float,
                )

            robot_velocity = (
                self._estimate_robot_reference_velocity(
                    robot_reference_position,
                    timestamp,
                )
            )

            try:
                collision_result = (
                    self._collision_estimator.update(
                        robot_position=robot_reference_position,
                        robot_velocity=robot_velocity,
                        operator_position=operator_position,
                        operator_velocity=operator_velocity,
                    )
                )

                self.get_logger().info(
                    "Interacción | "
                    f"Distancia={collision_result.distance:.3f} m | "
                    f"Riesgo={collision_result.collision_risk} | "
                    f"TTC={collision_result.time_to_collision}"
                )

            except Exception as exc:
                self.get_logger().error(
                    f"Error en CollisionEstimator: {exc}"
                )

        # ============================================================
        # VISUALIZATION
        # ============================================================

        if self._visualizar:
            self._draw_scene(
                frame=frame,
                primary_person=primary_person,
                robot_detection=robot_detection,
                operator_position=operator_position_base,
                robot_position=robot_position_base,
            )
            self._show_frame(frame)

    # ================================================================
    # TF2
    # ================================================================

    def _transform_position_to_base(
        self,
        position: np.ndarray,
        source_frame: str,
    ) -> Optional[np.ndarray]:
        """Transform a point from the PointCloud frame to base_link."""

        point = PointStamped()
        point.header.frame_id = source_frame
        point.header.stamp = self.get_clock().now().to_msg()

        point.point.x = float(position[0])
        point.point.y = float(position[1])
        point.point.z = float(position[2])

        try:
            transform = self._tf_buffer.lookup_transform(
                self._robot_base_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(
                    seconds=0.10
                ),
            )

            transformed = do_transform_point(
                point,
                transform,
            )

            return np.array(
                [
                    transformed.point.x,
                    transformed.point.y,
                    transformed.point.z,
                ],
                dtype=float,
            )

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:

            self.get_logger().debug(
                f"TF2 no disponible {source_frame} -> "
                f"{self._robot_base_frame}: {exc}"
            )
            return None

    def _get_robot_reference_position(
        self,
    ) -> Optional[np.ndarray]:
        """Get the robot reference point from TF2."""

        try:
            transform = self._tf_buffer.lookup_transform(
                self._robot_base_frame,
                self._robot_reference_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(
                    seconds=0.05
                ),
            )

            translation = transform.transform.translation

            return np.array(
                [
                    translation.x,
                    translation.y,
                    translation.z,
                ],
                dtype=float,
            )

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return None

    def _estimate_robot_reference_velocity(
        self,
        current_position: np.ndarray,
        timestamp: float,
    ) -> np.ndarray:
        """Estimate robot reference velocity from consecutive poses."""

        if (
            self._last_robot_reference_position is None
            or self._last_robot_reference_timestamp is None
        ):
            self._last_robot_reference_position = (
                current_position.copy()
            )
            self._last_robot_reference_timestamp = timestamp

            return np.zeros(
                3,
                dtype=float,
            )

        dt = (
            timestamp
            - self._last_robot_reference_timestamp
        )

        if dt <= 1e-4:
            return np.zeros(
                3,
                dtype=float,
            )

        velocity = (
            current_position
            - self._last_robot_reference_position
        ) / dt

        self._last_robot_reference_position = (
            current_position.copy()
        )
        self._last_robot_reference_timestamp = timestamp

        return velocity

    # ================================================================
    # VISUALIZATION
    # ================================================================

    def _draw_scene(
        self,
        frame: np.ndarray,
        primary_person,
        robot_detection,
        operator_position,
        robot_position,
    ) -> None:
        """Draw RGB detections and 3D coordinates."""

        if self._visualizar_detecciones:
            if primary_person is not None:
                x1 = int(primary_person.x1)
                y1 = int(primary_person.y1)
                x2 = int(primary_person.x2)
                y2 = int(primary_person.y2)

                cv2.rectangle(
                    frame,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    2,
                )
                cv2.putText(
                    frame,
                    "Operator",
                    (x1, max(25, y1 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )

            if robot_detection is not None:
                x1, y1, x2, y2 = (
                    int(value)
                    for value in robot_detection.bbox
                )

                cv2.rectangle(
                    frame,
                    (x1, y1),
                    (x2, y2),
                    (255, 0, 0),
                    2,
                )
                cv2.putText(
                    frame,
                    (
                        f"Robot "
                        f"{robot_detection.confidence:.2f}"
                    ),
                    (x1, max(25, y1 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 0, 0),
                    2,
                )

        if self._visualizar_posiciones_3d:
            y = 35

            if operator_position is not None:
                cv2.putText(
                    frame,
                    (
                        "Operator XYZ [m]: "
                        f"{operator_position[0]:.2f}, "
                        f"{operator_position[1]:.2f}, "
                        f"{operator_position[2]:.2f}"
                    ),
                    (20, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 0),
                    2,
                )
                y += 25

            if robot_position is not None:
                cv2.putText(
                    frame,
                    (
                        "Robot RGB XYZ [m]: "
                        f"{robot_position[0]:.2f}, "
                        f"{robot_position[1]:.2f}, "
                        f"{robot_position[2]:.2f}"
                    ),
                    (20, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 0, 0),
                    2,
                )

    def _show_frame(self, frame: np.ndarray) -> None:
        """Display the current scene."""

        cv2.imshow(
            "Robot Operator Interaction",
            frame,
        )

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q") or key == 27:
            rclpy.shutdown()

    # ================================================================
    # SHUTDOWN
    # ================================================================

    def destroy_node(self) -> None:
        """Release OpenCV resources."""

        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    node = None

    try:
        node = RobotOperatorInteractionNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as exc:
        if node is not None:
            node.get_logger().error(
                f"Error en RobotOperatorInteractionNode: {exc}"
            )
        else:
            print(
                "Error inicializando "
                f"RobotOperatorInteractionNode: {exc}"
            )

    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

"""
Integrated ROS 2 node for robot/operator interaction.

Pipeline:
    RealSense RGB
        -> PersonDetector / PersonTracker
        -> RobotDetector (ABB IRB 140)
        -> PointCloud2
        -> 3D positions
        -> TF2 -> base_link
        -> robot reference (tool0) from TF
        -> CollisionEstimator
        -> /robot_operator_interaction

The YOLO + PointCloud robot position is a visual observation.
The collision estimator uses the configurable robot_reference_frame
(default: tool0) because it is tied to the robot kinematic state.
"""

from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, PointStamped, Vector3
from sensor_msgs.msg import Image, JointState, PointCloud2

import tf2_ros
from tf2_geometry_msgs import do_transform_point

from colab_supervision_interfaces.msg import RobotOperatorInteraction
from colab_supervision_operator.perception.person_detector import PersonDetector
from colab_supervision_operator.perception.person_tracker import PersonTracker
from colab_supervision_operator.perception.depth_person_detector import DepthPersonDetector
from colab_supervision_operator.perception.robot_detector import RobotDetector
from colab_supervision_operator.perception.pointcloud_object_detector import PointCloudObjectDetector
from colab_supervision_operator.perception.robot_state import RobotState
from colab_supervision_operator.perception.collision_estimator import CollisionEstimator


class RobotOperatorInteractionNode(Node):
    """Central node for robot/operator spatial interaction analysis."""

    def __init__(self) -> None:
        super().__init__("robot_operator_interaction")

        # ============================================================
        # TOPICS / FRAMES
        # ============================================================
        self.declare_parameter("rgb_topic", "/IRB140_Cain/D435i/color/image_raw")
        self.declare_parameter("pointcloud_topic", "/IRB140_Cain/D435i/depth/color/points")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("interaction_topic", "/robot_operator_interaction")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("robot_reference_frame", "tool0")

        # ============================================================
        # PROCESSING
        # ============================================================
        self.declare_parameter("processing_rate", 10.0)
        self.declare_parameter("max_sensor_age", 1.0)
        self.declare_parameter("max_rgb_pointcloud_dt", 0.15)

        # ============================================================
        # OPERATOR YOLO
        # ============================================================
        self.declare_parameter("operator_model", "yolo26n.pt")
        self.declare_parameter("operator_confidence", 0.50)
        self.declare_parameter("operator_imgsz", 640)
        self.declare_parameter("operator_device", "cpu")

        # ============================================================
        # ROBOT YOLO
        # ============================================================
        package_share = get_package_share_directory("colab_supervision_operator")
        default_robot_model = str(Path(package_share) / "models" / "abb_irb140_yolo.pt")

        self.declare_parameter("robot_model", default_robot_model)
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
        # OPERATOR DEPTH TRACKING
        # ============================================================
        self.declare_parameter("operator_depth_minimum_confidence", 0.50)
        self.declare_parameter("operator_depth_maximum_distance", 5.0)

        # ============================================================
        # COLLISION THRESHOLDS
        # ============================================================
        self.declare_parameter("warning_distance", 1.00)
        self.declare_parameter("critical_distance", 0.50)
        self.declare_parameter("warning_time_to_collision", 2.00)
        self.declare_parameter("critical_time_to_collision", 1.00)

        # ============================================================
        # VISUALIZATION
        # ============================================================
        self.declare_parameter("visualizar", True)
        self.declare_parameter("visualizar_detecciones", True)
        self.declare_parameter("visualizar_posiciones_3d", True)
        self.declare_parameter("visualizar_interaccion", True)
        self.declare_parameter("window_width", 1280)
        self.declare_parameter("window_height", 720)

        # ============================================================
        # READ PARAMETERS
        # ============================================================
        self._rgb_topic = str(self.get_parameter("rgb_topic").value)
        self._pointcloud_topic = str(self.get_parameter("pointcloud_topic").value)
        self._joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        self._interaction_topic = str(self.get_parameter("interaction_topic").value)
        self._robot_base_frame = str(self.get_parameter("robot_base_frame").value)
        self._robot_reference_frame = str(self.get_parameter("robot_reference_frame").value)

        self._processing_rate = float(self.get_parameter("processing_rate").value)
        self._max_sensor_age = float(self.get_parameter("max_sensor_age").value)
        self._max_rgb_pointcloud_dt = float(self.get_parameter("max_rgb_pointcloud_dt").value)

        self._visualizar = bool(self.get_parameter("visualizar").value)
        self._visualizar_detecciones = bool(self.get_parameter("visualizar_detecciones").value)
        self._visualizar_posiciones_3d = bool(self.get_parameter("visualizar_posiciones_3d").value)
        self._visualizar_interaccion = bool(self.get_parameter("visualizar_interaccion").value)
        self._window_width = int(self.get_parameter("window_width").value)
        self._window_height = int(self.get_parameter("window_height").value)

        if self._processing_rate <= 0.0:
            raise ValueError("processing_rate must be greater than zero")

        # ============================================================
        # PERCEPTION MODULES
        # ============================================================
        operator_device = str(self.get_parameter("operator_device").value).strip() or None
        self._person_detector = PersonDetector(
            model_path=str(self.get_parameter("operator_model").value),
            confidence_threshold=float(self.get_parameter("operator_confidence").value),
            image_size=int(self.get_parameter("operator_imgsz").value),
            device=operator_device,
        )
        self._person_tracker = PersonTracker()

        robot_model = str(self.get_parameter("robot_model").value)
        if not Path(robot_model).is_file():
            raise FileNotFoundError(f"ABB IRB 140 YOLO model not found: {robot_model}")

        robot_device = str(self.get_parameter("robot_device").value).strip() or None
        self._robot_detector = RobotDetector(
            model_path=robot_model,
            confidence_threshold=float(self.get_parameter("robot_confidence").value),
            image_size=int(self.get_parameter("robot_imgsz").value),
            device=robot_device,
            robot_class_id=int(self.get_parameter("robot_class_id").value),
        )

        self._pointcloud_detector = PointCloudObjectDetector(
            sample_half_size=int(self.get_parameter("pointcloud_sample_half_size").value),
            minimum_valid_points=int(self.get_parameter("pointcloud_minimum_valid_points").value),
            maximum_range=float(self.get_parameter("pointcloud_maximum_range").value),
        )

        self._depth_person_detector = DepthPersonDetector(
            minimum_confidence=float(self.get_parameter("operator_depth_minimum_confidence").value),
            maximum_detection_distance=float(self.get_parameter("operator_depth_maximum_distance").value),
        )

        self._robot_state = RobotState(number_of_joints=6)
        self._collision_estimator = CollisionEstimator(
            warning_distance=float(self.get_parameter("warning_distance").value),
            critical_distance=float(self.get_parameter("critical_distance").value),
            warning_time_to_collision=float(self.get_parameter("warning_time_to_collision").value),
            critical_time_to_collision=float(self.get_parameter("critical_time_to_collision").value),
        )

        # ============================================================
        # ROS INTERFACES
        # ============================================================
        self._bridge = CvBridge()
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._interaction_pub = self.create_publisher(
            RobotOperatorInteraction,
            self._interaction_topic,
            10,
        )

        self.create_subscription(
            Image,
            self._rgb_topic,
            self._rgb_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            PointCloud2,
            self._pointcloud_topic,
            self._pointcloud_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            JointState,
            self._joint_states_topic,
            self._joint_state_callback,
            10,
        )

        # ============================================================
        # STATE
        # ============================================================
        self._latest_rgb: Optional[Image] = None
        self._latest_cloud: Optional[PointCloud2] = None
        self._rgb_receive_time: Optional[float] = None
        self._cloud_receive_time: Optional[float] = None

        self._last_robot_ref_position: Optional[np.ndarray] = None
        self._last_robot_ref_time: Optional[float] = None
        self._last_log_time = 0.0
        self._window_created = False

        self._timer = self.create_timer(
            1.0 / self._processing_rate,
            self._process_scene,
        )

        self.get_logger().info("====================================================")
        self.get_logger().info("Robot Operator Interaction")
        self.get_logger().info(f"RGB: {self._rgb_topic}")
        self.get_logger().info(f"PointCloud2: {self._pointcloud_topic}")
        self.get_logger().info(f"Robot model: {robot_model}")
        self.get_logger().info(f"Robot reference: {self._robot_reference_frame}")
        self.get_logger().info(f"Output: {self._interaction_topic}")
        self.get_logger().info(f"Visualization: {self._visualizar}")
        self.get_logger().info("====================================================")

    # ================================================================
    # CALLBACKS
    # ================================================================
    def _rgb_callback(self, msg: Image) -> None:
        self._latest_rgb = msg
        self._rgb_receive_time = self._now_seconds()

    def _pointcloud_callback(self, msg: PointCloud2) -> None:
        self._latest_cloud = msg
        self._cloud_receive_time = self._now_seconds()

    def _joint_state_callback(self, msg: JointState) -> None:
        if len(msg.position) != 6:
            self._debug_log(
                f"Se esperaban 6 articulaciones; se recibieron {len(msg.position)}"
            )
            return

        positions = np.asarray(msg.position, dtype=float)
        velocities = (
            np.asarray(msg.velocity, dtype=float)
            if len(msg.velocity) == 6
            else np.zeros(6, dtype=float)
        )

        try:
            self._robot_state.update_joint_state(positions, velocities)
        except Exception as exc:
            self._debug_log(f"RobotState: {exc}")

    # ================================================================
    # PROCESSING
    # ================================================================
    def _process_scene(self) -> None:
        if self._latest_rgb is None or self._latest_cloud is None:
            self._publish_invalid("WAITING FOR SENSORS")
            return

        image_msg = self._latest_rgb
        cloud = self._latest_cloud
        now = self._now_seconds()

        try:
            frame = self._bridge.imgmsg_to_cv2(
                image_msg,
                desired_encoding="bgr8",
            )
        except CvBridgeError as exc:
            self._publish_invalid("RGB CONVERSION ERROR")
            self._debug_log(f"RGB conversion: {exc}")
            return

        rgb_ok = (
            self._rgb_receive_time is not None
            and now - self._rgb_receive_time <= self._max_sensor_age
        )
        cloud_ok = (
            self._cloud_receive_time is not None
            and now - self._cloud_receive_time <= self._max_sensor_age
        )
        cloud_organized = cloud.height > 1
        sensor_dt = abs(
            self._stamp_seconds(image_msg)
            - self._stamp_seconds(cloud)
        )

        # ------------------------------------------------------------
        # Operator 2D
        # ------------------------------------------------------------
        primary_person = None
        try:
            persons = self._person_detector.update(frame)
            tracking_result = self._person_tracker.update(persons)
            if tracking_result.person_detected:
                primary_person = self._person_tracker.get_primary_person()
        except Exception as exc:
            self._debug_log(f"Operador YOLO: {exc}")

        # ------------------------------------------------------------
        # Operator 3D
        # ------------------------------------------------------------
        operator_position = None
        operator_depth_result = None

        if primary_person is not None and cloud_organized and sensor_dt <= self._max_rgb_pointcloud_dt:
            operator_bbox = (
                primary_person.x1,
                primary_person.y1,
                primary_person.x2,
                primary_person.y2,
            )
            operator_xyz_camera, _ = self._bbox_to_3d_aligned(
                cloud,
                operator_bbox,
                frame.shape[1],
                frame.shape[0],
            )
            if operator_xyz_camera is not None:
                operator_position = self._transform_position_to_base(
                    operator_xyz_camera,
                    cloud.header.frame_id,
                )

        operator_confidence = (
            float(primary_person.confidence)
            if primary_person is not None
            else 0.0
        )
        operator_depth_result = self._depth_person_detector.update(
            position=operator_position,
            confidence=(operator_confidence if operator_position is not None else 0.0),
            timestamp=now,
        )

        operator_valid = (
            operator_depth_result.detection_valid
            and operator_depth_result.person_detected
            and operator_depth_result.position is not None
        )

        # ------------------------------------------------------------
        # Robot 2D
        # ------------------------------------------------------------
        robot_detection = None
        try:
            detections = self._robot_detector.detect(frame)
            if detections:
                robot_detection = detections[0]
        except Exception as exc:
            self._debug_log(f"Robot YOLO: {exc}")

        # ------------------------------------------------------------
        # Robot visual 3D
        # ------------------------------------------------------------
        robot_visual_position = None
        robot_valid_points = 0
        if robot_detection is not None and cloud_organized and sensor_dt <= self._max_rgb_pointcloud_dt:
            robot_xyz_camera, robot_valid_points = self._bbox_to_3d_aligned(
                cloud,
                robot_detection.bbox,
                frame.shape[1],
                frame.shape[0],
            )
            if robot_xyz_camera is not None:
                robot_visual_position = self._transform_position_to_base(
                    robot_xyz_camera,
                    cloud.header.frame_id,
                )

        # ------------------------------------------------------------
        # Robot kinematic reference
        # ------------------------------------------------------------
        robot_reference_position = self._get_robot_reference_position()
        robot_reference_velocity = self._estimate_robot_reference_velocity(
            robot_reference_position,
            now,
        )

        # ------------------------------------------------------------
        # Interaction / collision
        # ------------------------------------------------------------
        collision_result = None
        if operator_valid and robot_reference_position is not None:
            operator_velocity = operator_depth_result.velocity
            if operator_velocity is None:
                operator_velocity = np.zeros(3, dtype=float)

            collision_result = self._collision_estimator.update(
                robot_position=robot_reference_position,
                robot_velocity=robot_reference_velocity,
                operator_position=operator_depth_result.position,
                operator_velocity=operator_velocity,
            )

        status = self._build_status(
            rgb_ok,
            cloud_ok,
            cloud_organized,
            sensor_dt,
            primary_person,
            operator_valid,
            robot_detection,
            robot_visual_position,
            robot_reference_position,
            collision_result,
        )

        self._publish_interaction(
            image_msg,
            primary_person,
            operator_depth_result,
            robot_detection,
            robot_visual_position,
            robot_reference_position,
            robot_reference_velocity,
            collision_result,
            status,
            robot_valid_points,
        )

        if self._visualizar:
            self._draw_scene(
                frame,
                rgb_ok,
                cloud_ok,
                cloud_organized,
                sensor_dt,
                primary_person,
                robot_detection,
                operator_depth_result.position if operator_valid else None,
                robot_visual_position,
                robot_reference_position,
                collision_result,
                status,
                robot_valid_points,
            )
            self._show(frame)

    # ================================================================
    # RGB -> POINTCLOUD2 -> XYZ
    # ================================================================
    def _bbox_to_3d_aligned(
        self,
        cloud: PointCloud2,
        bbox,
        rgb_width: int,
        rgb_height: int,
    ):
        """
        Convert an RGB bounding box to XYZ using the same
        implementation validated in the standalone RealSense tests.

        The RGB and PointCloud2 streams may have different image
        dimensions. The bbox center is therefore explicitly mapped
        from RGB coordinates to PointCloud2 coordinates before
        sampling the organized cloud.
        """

        if cloud.width <= 0 or cloud.height <= 0:
            return None, 0

        x1, y1, x2, y2 = bbox

        # ------------------------------------------------------------
        # RGB coordinates -> PointCloud2 coordinates
        # ------------------------------------------------------------
        cx = int(round(
            ((x1 + x2) / 2.0)
            * cloud.width
            / max(1, rgb_width)
        ))
        cy = int(round(
            ((y1 + y2) / 2.0)
            * cloud.height
            / max(1, rgb_height)
        ))

        cx = int(np.clip(cx, 0, cloud.width - 1))
        cy = int(np.clip(cy, 0, cloud.height - 1))

        half = int(
            self.get_parameter(
                "pointcloud_sample_half_size"
            ).value
        )

        u1 = max(0, cx - half)
        u2 = min(cloud.width - 1, cx + half)
        v1 = max(0, cy - half)
        v2 = min(cloud.height - 1, cy + half)

        # ------------------------------------------------------------
        # Read organized PointCloud2
        # ------------------------------------------------------------
        try:
            from sensor_msgs_py import point_cloud2

            points = point_cloud2.read_points_numpy(
                cloud,
                field_names=("x", "y", "z"),
                skip_nans=False,
            )
        except Exception as exc:
            self._debug_log(
                f"PointCloud2 read error: {exc}"
            )
            return None, 0

        points = np.asarray(points)

        if points.ndim == 3:
            organized = points[:, :, :3]

        elif (
            points.ndim == 2
            and points.shape[0] == cloud.width * cloud.height
        ):
            organized = points.reshape(
                cloud.height,
                cloud.width,
                3,
            )

        else:
            return None, 0

        # ------------------------------------------------------------
        # Region around the bbox center
        # ------------------------------------------------------------
        xyz = organized[
            v1:v2 + 1,
            u1:u2 + 1,
            :
        ].reshape(-1, 3).astype(np.float64)

        # ------------------------------------------------------------
        # Valid points
        # ------------------------------------------------------------
        xyz = xyz[
            np.isfinite(xyz).all(axis=1)
        ]

        if xyz.size == 0:
            return None, 0

        distances = np.linalg.norm(xyz, axis=1)

        maximum_range = float(
            self.get_parameter(
                "pointcloud_maximum_range"
            ).value
        )

        valid = (
            np.isfinite(distances)
            & (distances > 0.05)
            & (distances <= maximum_range)
        )

        xyz = xyz[valid]
        valid_points = int(xyz.shape[0])

        minimum_valid_points = int(
            self.get_parameter(
                "pointcloud_minimum_valid_points"
            ).value
        )

        if valid_points < minimum_valid_points:
            return None, valid_points

        # Exact same estimator as the validated standalone test.
        position = np.median(xyz, axis=0)

        return position, valid_points

    # ================================================================
    # TF / VELOCITY
    # ================================================================
    def _transform_position_to_base(
        self,
        position: np.ndarray,
        source_frame: str,
    ) -> Optional[np.ndarray]:
        point = PointStamped()
        point.header.frame_id = source_frame
        point.header.stamp = rclpy.time.Time().to_msg()
        point.point.x = float(position[0])
        point.point.y = float(position[1])
        point.point.z = float(position[2])

        try:
            transform = self._tf_buffer.lookup_transform(
                self._robot_base_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.10),
            )
            transformed = do_transform_point(point, transform)
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
            self._debug_log(
                f"TF {source_frame}->{self._robot_base_frame}: {exc}"
            )
            return None

    def _get_robot_reference_position(self) -> Optional[np.ndarray]:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._robot_base_frame,
                self._robot_reference_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
            t = transform.transform.translation
            return np.array([t.x, t.y, t.z], dtype=float)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return None

    def _estimate_robot_reference_velocity(
        self,
        position: Optional[np.ndarray],
        timestamp: float,
    ) -> np.ndarray:
        if position is None:
            self._last_robot_ref_position = None
            self._last_robot_ref_time = None
            return np.zeros(3, dtype=float)

        if self._last_robot_ref_position is None or self._last_robot_ref_time is None:
            self._last_robot_ref_position = position.copy()
            self._last_robot_ref_time = timestamp
            return np.zeros(3, dtype=float)

        dt = timestamp - self._last_robot_ref_time
        if dt <= 1e-4:
            return np.zeros(3, dtype=float)

        velocity = (position - self._last_robot_ref_position) / dt
        self._last_robot_ref_position = position.copy()
        self._last_robot_ref_time = timestamp
        return velocity

    # ================================================================
    # PUBLISH
    # ================================================================
    def _publish_interaction(
        self,
        image_msg,
        primary_person,
        operator_depth_result,
        robot_detection,
        robot_visual_position,
        robot_reference_position,
        robot_reference_velocity,
        collision_result,
        status,
        robot_valid_points,
    ) -> None:
        msg = RobotOperatorInteraction()
        msg.header = image_msg.header

        operator_valid = (
            operator_depth_result.detection_valid
            and operator_depth_result.person_detected
            and operator_depth_result.position is not None
        )

        msg.operator_detected = primary_person is not None
        msg.robot_detected = robot_detection is not None
        msg.operator_position_valid = operator_valid
        msg.robot_visual_position_valid = robot_visual_position is not None
        msg.robot_reference_valid = robot_reference_position is not None

        msg.operator_confidence = float(primary_person.confidence if primary_person else 0.0)
        msg.robot_confidence = float(robot_detection.confidence if robot_detection else 0.0)

        msg.operator_position = self._point_msg(
            operator_depth_result.position if operator_valid else None
        )
        msg.robot_visual_position = self._point_msg(robot_visual_position)
        msg.robot_reference_position = self._point_msg(robot_reference_position)

        operator_velocity = (
            operator_depth_result.velocity
            if operator_valid and operator_depth_result.velocity is not None
            else np.zeros(3, dtype=float)
        )
        msg.operator_velocity = self._vector_msg(operator_velocity)
        msg.robot_velocity = self._vector_msg(robot_reference_velocity)

        if collision_result is None:
            msg.interaction_valid = False
            msg.distance = -1.0
            msg.relative_velocity = 0.0
            msg.approach_velocity = 0.0
            msg.time_to_collision = -1.0
            msg.time_to_collision_valid = False
            msg.collision_risk_score = 0.0
            msg.collision_risk = "UNKNOWN"
            msg.collision_imminent = False
        else:
            msg.interaction_valid = True
            msg.distance = float(collision_result.distance)
            msg.relative_velocity = float(collision_result.relative_velocity)
            msg.approach_velocity = float(collision_result.approach_velocity)
            msg.collision_risk_score = float(collision_result.collision_risk_score)
            msg.collision_risk = str(collision_result.collision_risk)
            msg.collision_imminent = bool(collision_result.collision_imminent)

            if collision_result.time_to_collision is None:
                msg.time_to_collision = -1.0
                msg.time_to_collision_valid = False
            else:
                msg.time_to_collision = float(collision_result.time_to_collision)
                msg.time_to_collision_valid = True

        msg.status = str(status)
        self._interaction_pub.publish(msg)

    def _publish_invalid(self, status: str) -> None:
        msg = RobotOperatorInteraction()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.operator_detected = False
        msg.robot_detected = False
        msg.operator_position_valid = False
        msg.robot_visual_position_valid = False
        msg.robot_reference_valid = False
        msg.operator_confidence = 0.0
        msg.robot_confidence = 0.0
        msg.distance = -1.0
        msg.relative_velocity = 0.0
        msg.approach_velocity = 0.0
        msg.time_to_collision = -1.0
        msg.time_to_collision_valid = False
        msg.collision_risk_score = 0.0
        msg.collision_risk = "UNKNOWN"
        msg.collision_imminent = False
        msg.interaction_valid = False
        msg.status = status
        self._interaction_pub.publish(msg)

    @staticmethod
    def _point_msg(position: Optional[np.ndarray]) -> Point:
        msg = Point()
        if position is not None:
            msg.x = float(position[0])
            msg.y = float(position[1])
            msg.z = float(position[2])
        return msg

    @staticmethod
    def _vector_msg(vector: np.ndarray) -> Vector3:
        msg = Vector3()
        msg.x = float(vector[0])
        msg.y = float(vector[1])
        msg.z = float(vector[2])
        return msg

    # ================================================================
    # STATUS / VISUALIZATION
    # ================================================================
    @staticmethod
    def _build_status(
        rgb_ok,
        cloud_ok,
        cloud_organized,
        sensor_dt,
        primary_person,
        operator_valid,
        robot_detection,
        robot_visual_position,
        robot_reference_position,
        collision_result,
    ) -> str:
        if not rgb_ok:
            return "RGB NOT READY"
        if not cloud_ok:
            return "POINTCLOUD NOT READY"
        if not cloud_organized:
            return "POINTCLOUD NOT ORGANIZED"
        if sensor_dt > 0.15:
            return "RGB / POINTCLOUD NOT SYNC"
        if primary_person is None:
            return "OPERATOR NOT DETECTED"
        if not operator_valid:
            return "OPERATOR 3D INVALID"
        if robot_detection is None:
            return "ROBOT NOT DETECTED"
        if robot_visual_position is None:
            return "ROBOT 3D INVALID"
        if robot_reference_position is None:
            return "ROBOT TF INVALID"
        if collision_result is None:
            return "INTERACTION INVALID"
        return "INTERACTION VALID"

    def _draw_scene(
        self,
        frame,
        rgb_ok,
        cloud_ok,
        cloud_organized,
        sensor_dt,
        primary_person,
        robot_detection,
        operator_position,
        robot_visual_position,
        robot_reference_position,
        collision_result,
        status,
        robot_valid_points,
    ) -> None:
        if self._visualizar_detecciones:
            if primary_person is not None:
                cv2.rectangle(
                    frame,
                    (int(primary_person.x1), int(primary_person.y1)),
                    (int(primary_person.x2), int(primary_person.y2)),
                    (0, 255, 0),
                    2,
                )
                cv2.putText(
                    frame,
                    f"Operator {primary_person.confidence:.2f}",
                    (int(primary_person.x1), max(25, int(primary_person.y1) - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 0),
                    2,
                )

            if robot_detection is not None:
                x1, y1, x2, y2 = [int(v) for v in robot_detection.bbox]
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.putText(
                    frame,
                    f"ABB IRB 140 {robot_detection.confidence:.2f}",
                    (x1, max(25, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (255, 0, 0),
                    2,
                )
                cv2.circle(
                    frame,
                    (int(robot_detection.center_x), int(robot_detection.center_y)),
                    5,
                    (0, 255, 255),
                    -1,
                )

        if self._visualizar_posiciones_3d:
            y = 30
            entries = [
                ("Operator XYZ base:", operator_position, (0, 255, 0)),
                ("Robot visual XYZ base:", robot_visual_position, (255, 0, 0)),
                (f"Robot {self._robot_reference_frame} XYZ:", robot_reference_position, (255, 255, 0)),
            ]
            for label, position, text_color in entries:
                if position is not None:
                    cv2.putText(
                        frame,
                        f"{label} {position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f} m",
                        (15, y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.50,
                        text_color,
                        2,
                    )
                    y += 24

        if not self._visualizar_interaccion:
            return

        h, w = frame.shape[:2]
        panel_width = 430
        px = max(0, w - panel_width)
        py2 = min(h - 10, 690)

        overlay = frame.copy()
        cv2.rectangle(overlay, (px, 10), (w - 5, py2), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.78, frame, 0.22, 0.0, frame)

        x, y = px + 18, 35
        cv2.putText(frame, "ROBOT OPERATOR INTERACTION", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (255,255,255), 2)
        y += 28

        states = [
            ("RGB", rgb_ok),
            ("PointCloud2", cloud_ok),
            ("Organized cloud", cloud_organized),
            ("Operator", primary_person is not None),
            ("Robot YOLO", robot_detection is not None),
            ("Operator 3D", operator_position is not None),
            ("Robot visual 3D", robot_visual_position is not None),
            ("Robot TF", robot_reference_position is not None),
            ("Interaction", collision_result is not None),
        ]
        for label, ok in states:
            cv2.putText(frame, f"{label:<20}{'OK' if ok else 'WAIT / ERROR'}", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0,255,0) if ok else (0,0,255), 1)
            y += 22

        y += 4
        cv2.putText(frame, f"STATUS: {status}", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (0,255,255), 2)
        y += 27
        cv2.putText(frame, f"RGB-PC dt: {sensor_dt:.3f} s", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (220,220,220), 1)
        y += 23
        cv2.putText(frame, f"Robot valid 3D points: {robot_valid_points}", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (220,220,220), 1)
        y += 23

        if collision_result is not None:
            lines = [
                f"Distance: {collision_result.distance:.3f} m",
                f"Relative velocity: {collision_result.relative_velocity:.3f} m/s",
                f"Approach velocity: {collision_result.approach_velocity:.3f} m/s",
                (
                    "TTC: --"
                    if collision_result.time_to_collision is None
                    else f"TTC: {collision_result.time_to_collision:.3f} s"
                ),
                f"Risk: {collision_result.collision_risk}",
                f"Risk score: {collision_result.collision_risk_score:.3f}",
                f"Collision imminent: {'YES' if collision_result.collision_imminent else 'NO'}",
            ]
            for line in lines:
                color = (220,220,220)
                if line.startswith("Risk:"):
                    color = (
                        (0,0,255) if collision_result.collision_risk == "CRITICAL"
                        else (0,255,255) if collision_result.collision_risk == "WARNING"
                        else (0,255,0)
                    )
                cv2.putText(frame, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.44, color, 1 if not line.startswith("Risk:") else 2)
                y += 22

    def _show(self, frame) -> None:
        name = "Robot Operator Interaction"
        if not self._window_created:
            cv2.namedWindow(name, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
            cv2.resizeWindow(name, self._window_width, self._window_height)
            self._window_created = True
        cv2.imshow(name, frame)
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            rclpy.shutdown()

    # ================================================================
    # HELPERS
    # ================================================================
    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _stamp_seconds(msg) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9

    def _debug_log(self, message: str) -> None:
        now = self._now_seconds()
        if now - self._last_log_time >= 1.0:
            self.get_logger().warning(message)
            self._last_log_time = now

    def destroy_node(self) -> None:
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
            node.get_logger().error(f"Error en RobotOperatorInteractionNode: {exc}")
        else:
            print(f"Error inicializando RobotOperatorInteractionNode: {exc}")
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

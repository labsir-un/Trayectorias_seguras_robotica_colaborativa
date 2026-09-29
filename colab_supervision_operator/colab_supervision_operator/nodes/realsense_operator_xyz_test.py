#!/usr/bin/env python3

"""
ROS 2 visual test for RealSense D435i operator localization.

Validation chain:

    RealSense RGB
        -> YOLO person detection
        -> operator bounding box
        -> PointCloud2
        -> operator XYZ in camera/point-cloud frame
        -> TF2
        -> operator XYZ in base_link

The window intentionally shows the state of every stage so that the
RealSense perception chain can be diagnosed before adding robot,
zones and collision estimation.
"""

from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2

import tf2_ros
from tf2_geometry_msgs import do_transform_point

from colab_supervision_operator.perception.person_detector import PersonDetector
from colab_supervision_operator.perception.person_tracker import PersonTracker


class RealSenseOperatorXYZTest(Node):
    """Visual diagnostic node for RGB-D operator localization."""

    def __init__(self) -> None:
        super().__init__("realsense_operator_xyz_test")

        # ============================================================
        # TOPICS / TF
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
            "base_frame",
            "base_link",
        )

        # ============================================================
        # PROCESSING
        # ============================================================
        self.declare_parameter("processing_rate", 10.0)
        self.declare_parameter("max_sensor_age", 1.0)
        self.declare_parameter("max_rgb_pointcloud_dt", 0.15)

        # ============================================================
        # YOLO
        # ============================================================
        self.declare_parameter("yolo_model", "yolo26n.pt")
        self.declare_parameter("yolo_confidence", 0.50)
        self.declare_parameter("yolo_imgsz", 640)
        self.declare_parameter("yolo_device", "cpu")

        # ============================================================
        # POINT CLOUD
        # ============================================================
        self.declare_parameter("sample_half_size", 8)
        self.declare_parameter("minimum_valid_points", 10)
        self.declare_parameter("maximum_range", 5.0)

        # ============================================================
        # VISUALIZATION
        # ============================================================
        self.declare_parameter("visualizar", True)
        self.declare_parameter("window_width", 1280)
        self.declare_parameter("window_height", 720)

        # ============================================================
        # READ PARAMETERS
        # ============================================================
        self._rgb_topic = self.get_parameter("rgb_topic").value
        self._pointcloud_topic = self.get_parameter("pointcloud_topic").value
        self._base_frame = self.get_parameter("base_frame").value

        self._processing_rate = float(
            self.get_parameter("processing_rate").value
        )
        self._max_sensor_age = float(
            self.get_parameter("max_sensor_age").value
        )
        self._max_rgb_pointcloud_dt = float(
            self.get_parameter("max_rgb_pointcloud_dt").value
        )

        self._sample_half_size = int(
            self.get_parameter("sample_half_size").value
        )
        self._minimum_valid_points = int(
            self.get_parameter("minimum_valid_points").value
        )
        self._maximum_range = float(
            self.get_parameter("maximum_range").value
        )

        self._visualizar = bool(
            self.get_parameter("visualizar").value
        )
        self._window_width = int(
            self.get_parameter("window_width").value
        )
        self._window_height = int(
            self.get_parameter("window_height").value
        )

        # ============================================================
        # YOLO
        # ============================================================
        yolo_model = self.get_parameter("yolo_model").value
        yolo_confidence = float(
            self.get_parameter("yolo_confidence").value
        )
        yolo_imgsz = int(
            self.get_parameter("yolo_imgsz").value
        )
        yolo_device = self.get_parameter("yolo_device").value

        if yolo_device == "":
            yolo_device = None

        self._person_detector = PersonDetector(
            model_path=yolo_model,
            confidence_threshold=yolo_confidence,
            image_size=yolo_imgsz,
            device=yolo_device,
        )
        self._person_tracker = PersonTracker()

        self._yolo_ready = True
        self._yolo_error: Optional[str] = None

        # ============================================================
        # ROS INTERFACES
        # ============================================================
        self._bridge = CvBridge()

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer,
            self,
        )

        self._latest_rgb: Optional[Image] = None
        self._latest_cloud: Optional[PointCloud2] = None

        self._rgb_receive_time: Optional[float] = None
        self._cloud_receive_time: Optional[float] = None

        self._last_log_time = 0.0

        self._rgb_subscription = self.create_subscription(
            Image,
            self._rgb_topic,
            self._rgb_callback,
            qos_profile_sensor_data,
        )

        self._pointcloud_subscription = self.create_subscription(
            PointCloud2,
            self._pointcloud_topic,
            self._pointcloud_callback,
            qos_profile_sensor_data,
        )

        self._timer = self.create_timer(
            1.0 / self._processing_rate,
            self._process,
        )

        self.get_logger().info("====================================================")
        self.get_logger().info("RealSense Operator XYZ Test")
        self.get_logger().info(f"RGB topic: {self._rgb_topic}")
        self.get_logger().info(
            f"PointCloud2 topic: {self._pointcloud_topic}"
        )
        self.get_logger().info(f"Base frame: {self._base_frame}")
        self.get_logger().info(
            f"PointCloud sample: +/- {self._sample_half_size} px"
        )
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

    # ================================================================
    # MAIN PROCESSING
    # ================================================================

    def _process(self) -> None:
        now = self._now_seconds()

        if self._latest_rgb is None:
            self._show_status_only("WAITING FOR RGB")
            return

        if self._latest_cloud is None:
            try:
                frame = self._bridge.imgmsg_to_cv2(
                    self._latest_rgb,
                    desired_encoding="bgr8",
                )
            except CvBridgeError:
                self._show_status_only("RGB CONVERSION ERROR")
                return

            frame = self._fit_window(frame)
            self._draw_status_panel(
                frame,
                rgb_ok=True,
                cloud_ok=False,
                cloud_organized=False,
                yolo_ok=self._yolo_ready,
                operator_ok=False,
                xyz_ok=False,
                tf_ok=False,
                message="WAITING FOR POINTCLOUD2",
            )
            self._show(frame)
            return

        # ------------------------------------------------------------
        # Convert RGB
        # ------------------------------------------------------------
        try:
            frame = self._bridge.imgmsg_to_cv2(
                self._latest_rgb,
                desired_encoding="bgr8",
            )
        except CvBridgeError as exc:
            self.get_logger().error(
                f"Error convirtiendo RGB: {exc}"
            )
            return

        rgb_age = now - self._rgb_receive_time if self._rgb_receive_time else 999.0
        cloud_age = now - self._cloud_receive_time if self._cloud_receive_time else 999.0

        rgb_ok = rgb_age <= self._max_sensor_age
        cloud_ok = cloud_age <= self._max_sensor_age
        cloud_organized = self._latest_cloud.height > 1

        # ------------------------------------------------------------
        # RGB / PointCloud synchronization diagnostic
        # ------------------------------------------------------------
        rgb_stamp = self._stamp_seconds(self._latest_rgb)
        cloud_stamp = self._stamp_seconds(self._latest_cloud)
        sensor_dt = abs(rgb_stamp - cloud_stamp)

        # ------------------------------------------------------------
        # Default state
        # ------------------------------------------------------------
        operator_ok = False
        xyz_camera = None
        xyz_base = None
        tf_ok = False
        message = ""
        operator = None
        point_count = 0

        # ============================================================
        # OPERATOR DETECTION
        # ============================================================
        try:
            persons = self._person_detector.update(frame)
            tracking_result = self._person_tracker.update(persons)

            if tracking_result.person_detected:
                operator = self._person_tracker.get_primary_person()
                operator_ok = operator is not None
        except Exception as exc:
            self._yolo_ready = False
            self._yolo_error = str(exc)
            message = "YOLO ERROR"

        # ============================================================
        # OPERATOR RGB -> POINT CLOUD
        # ============================================================
        if operator is not None and cloud_organized and sensor_dt <= self._max_rgb_pointcloud_dt:
            xyz_camera, point_count = self._bbox_to_3d(
                self._latest_cloud,
                (
                    operator.x1,
                    operator.y1,
                    operator.x2,
                    operator.y2,
                ),
                rgb_width=frame.shape[1],
                rgb_height=frame.shape[0],
            )

            if xyz_camera is None:
                message = "OPERATOR DEPTH INVALID"
            else:
                # ====================================================
                # TF2 -> base_link
                # ====================================================
                xyz_base = self._transform_to_base(
                    xyz_camera,
                    self._latest_cloud.header.frame_id,
                )
                tf_ok = xyz_base is not None

                if xyz_base is None:
                    message = "TF WAITING / INVALID"
                else:
                    message = "OPERATOR XYZ OK"

        elif operator is None:
            message = "OPERATOR NOT DETECTED"
        elif not cloud_organized:
            message = "POINTCLOUD NOT ORGANIZED"
        elif sensor_dt > self._max_rgb_pointcloud_dt:
            message = "RGB / POINTCLOUD NOT SYNC"

        # ============================================================
        # VISUALIZATION
        # ============================================================
        frame = self._fit_window(frame)

        if operator is not None:
            scale_x = frame.shape[1] / max(1, self._latest_rgb.width)
            scale_y = frame.shape[0] / max(1, self._latest_rgb.height)

            x1 = int(operator.x1 * scale_x)
            y1 = int(operator.y1 * scale_y)
            x2 = int(operator.x2 * scale_x)
            y2 = int(operator.y2 * scale_y)

            cv2.rectangle(
                frame,
                (x1, y1),
                (x2, y2),
                (0, 255, 0),
                2,
            )

            cv2.putText(
                frame,
                f"Operator {operator.confidence:.2f}",
                (x1, max(25, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 0),
                2,
            )

        self._draw_status_panel(
            frame,
            rgb_ok=rgb_ok,
            cloud_ok=cloud_ok,
            cloud_organized=cloud_organized,
            yolo_ok=self._yolo_ready,
            operator_ok=operator_ok,
            xyz_ok=xyz_camera is not None,
            tf_ok=tf_ok,
            message=message,
            xyz_camera=xyz_camera,
            xyz_base=xyz_base,
            cloud_frame=self._latest_cloud.header.frame_id,
            cloud_size=(
                self._latest_cloud.width,
                self._latest_cloud.height,
            ),
            rgb_size=(
                self._latest_rgb.width,
                self._latest_rgb.height,
            ),
            sensor_dt=sensor_dt,
            point_count=point_count,
        )

        if self._visualizar:
            self._show(frame)

        # ------------------------------------------------------------
        # Useful terminal output only when XYZ is available.
        # ------------------------------------------------------------
        if xyz_base is not None:
            self.get_logger().info(
                "Operador | "
                f"Camera XYZ=[{xyz_camera[0]:.3f}, "
                f"{xyz_camera[1]:.3f}, {xyz_camera[2]:.3f}] m | "
                f"{self._base_frame} XYZ=[{xyz_base[0]:.3f}, "
                f"{xyz_base[1]:.3f}, {xyz_base[2]:.3f}] m"
            )

    # ================================================================
    # POINT CLOUD PROCESSING
    # ================================================================

    def _bbox_to_3d(
        self,
        cloud: PointCloud2,
        bbox: tuple[float, float, float, float],
        rgb_width: int,
        rgb_height: int,
    ) -> tuple[Optional[np.ndarray], int]:
        """
        Extract a robust XYZ from the bbox center.

        When RGB and PointCloud2 dimensions differ, the RGB pixel is
        scaled into the organized PointCloud2 image coordinates. This
        is valid for the configured RealSense point cloud when the
        point cloud has the RGB stream as its texture/alignment source.
        """

        if cloud.width <= 0 or cloud.height <= 0:
            return None, 0

        x1, y1, x2, y2 = bbox

        rgb_center_x = (x1 + x2) / 2.0
        rgb_center_y = (y1 + y2) / 2.0

        cloud_x = int(
            round(rgb_center_x * cloud.width / max(1, rgb_width))
        )
        cloud_y = int(
            round(rgb_center_y * cloud.height / max(1, rgb_height))
        )

        cloud_x = int(np.clip(cloud_x, 0, cloud.width - 1))
        cloud_y = int(np.clip(cloud_y, 0, cloud.height - 1))

        half = self._sample_half_size

        u1 = max(0, cloud_x - half)
        u2 = min(cloud.width - 1, cloud_x + half)
        v1 = max(0, cloud_y - half)
        v2 = min(cloud.height - 1, cloud_y + half)

        try:
            points = point_cloud2.read_points_numpy(
                cloud,
                field_names=("x", "y", "z"),
                skip_nans=False,
            )
        except Exception as exc:
            self._debug_log(
                f"Error leyendo PointCloud2: {exc}"
            )
            return None, 0

        points = np.asarray(points)

        if points.ndim == 3:
            organized = points[:, :, :3]
        elif points.ndim == 2:
            expected = cloud.width * cloud.height
            if points.shape[0] != expected:
                return None, 0
            organized = points.reshape(
                cloud.height,
                cloud.width,
                3,
            )
        else:
            return None, 0

        region = organized[v1:v2 + 1, u1:u2 + 1, :]
        xyz = region.reshape(-1, 3).astype(np.float64)

        valid = np.isfinite(xyz).all(axis=1)
        xyz = xyz[valid]

        if xyz.size == 0:
            return None, 0

        distance = np.linalg.norm(xyz, axis=1)
        valid_distance = (
            np.isfinite(distance)
            & (distance > 0.05)
            & (distance <= self._maximum_range)
        )

        xyz = xyz[valid_distance]
        count = int(xyz.shape[0])

        if count < self._minimum_valid_points:
            return None, count

        return np.median(xyz, axis=0), count

    # ================================================================
    # TF2
    # ================================================================

    def _transform_to_base(
        self,
        position: np.ndarray,
        source_frame: str,
    ) -> Optional[np.ndarray]:
        """Transform a point from the point-cloud frame to base_link."""

        point = PointStamped()
        point.header.frame_id = source_frame
        point.header.stamp = self.get_clock().now().to_msg()
        point.point.x = float(position[0])
        point.point.y = float(position[1])
        point.point.z = float(position[2])

        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.10),
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
            self._debug_log(
                f"TF2 {source_frame} -> {self._base_frame}: {exc}"
            )
            return None

    # ================================================================
    # VISUALIZATION
    # ================================================================

    def _show_status_only(self, message: str) -> None:
        frame = np.zeros(
            (self._window_height, self._window_width, 3),
            dtype=np.uint8,
        )

        self._draw_status_panel(
            frame,
            rgb_ok=self._latest_rgb is not None,
            cloud_ok=self._latest_cloud is not None,
            cloud_organized=(
                self._latest_cloud is not None
                and self._latest_cloud.height > 1
            ),
            yolo_ok=self._yolo_ready,
            operator_ok=False,
            xyz_ok=False,
            tf_ok=False,
            message=message,
        )

        if self._visualizar:
            self._show(frame)

    def _draw_status_panel(
        self,
        frame: np.ndarray,
        *,
        rgb_ok: bool,
        cloud_ok: bool,
        cloud_organized: bool,
        yolo_ok: bool,
        operator_ok: bool,
        xyz_ok: bool,
        tf_ok: bool,
        message: str,
        xyz_camera: Optional[np.ndarray] = None,
        xyz_base: Optional[np.ndarray] = None,
        cloud_frame: str = "-",
        cloud_size: tuple[int, int] = (0, 0),
        rgb_size: tuple[int, int] = (0, 0),
        sensor_dt: float = 0.0,
        point_count: int = 0,
    ) -> None:
        """Draw the diagnostic state panel on the right side."""

        panel_width = 410
        panel_x1 = max(0, frame.shape[1] - panel_width)
        panel_x2 = frame.shape[1] - 5
        panel_y1 = 10
        panel_y2 = min(frame.shape[0] - 10, 575)

        overlay = frame.copy()
        cv2.rectangle(
            overlay,
            (panel_x1, panel_y1),
            (panel_x2, panel_y2),
            (20, 20, 20),
            -1,
        )
        cv2.addWeighted(
            overlay,
            0.75,
            frame,
            0.25,
            0.0,
            frame,
        )

        cv2.rectangle(
            frame,
            (panel_x1, panel_y1),
            (panel_x2, panel_y2),
            (180, 180, 180),
            1,
        )

        x = panel_x1 + 18
        y = panel_y1 + 28

        cv2.putText(
            frame,
            "REALSENSE D435i - OPERATOR XYZ",
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            2,
        )
        y += 30

        def state_line(label: str, state: bool) -> None:
            nonlocal y
            text = f"{label:<20} {'OK' if state else 'WAIT / ERROR'}"
            cv2.putText(
                frame,
                text,
                (x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                (0, 255, 0) if state else (0, 0, 255),
                1,
            )
            y += 25

        state_line("RGB stream", rgb_ok)
        state_line("PointCloud2", cloud_ok)
        state_line("Organized cloud", cloud_organized)
        state_line("YOLO", yolo_ok)
        state_line("Operator", operator_ok)
        state_line("3D XYZ", xyz_ok)
        state_line("TF -> base_link", tf_ok)

        y += 5
        cv2.putText(
            frame,
            f"STATUS: {message}",
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (0, 255, 255),
            2,
        )
        y += 28

        if rgb_size != (0, 0):
            cv2.putText(
                frame,
                f"RGB: {rgb_size[0]} x {rgb_size[1]}",
                (x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.44,
                (220, 220, 220),
                1,
            )
            y += 22

        if cloud_size != (0, 0):
            cv2.putText(
                frame,
                f"Cloud: {cloud_size[0]} x {cloud_size[1]}",
                (x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.44,
                (220, 220, 220),
                1,
            )
            y += 22

        cv2.putText(
            frame,
            f"RGB-PC dt: {sensor_dt:.3f} s",
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.44,
            (220, 220, 220),
            1,
        )
        y += 22

        cv2.putText(
            frame,
            f"Valid 3D points: {point_count}",
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.44,
            (220, 220, 220),
            1,
        )
        y += 22

        cv2.putText(
            frame,
            f"Cloud frame: {cloud_frame}",
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.40,
            (220, 220, 220),
            1,
        )
        y += 28

        if xyz_camera is not None:
            cv2.putText(
                frame,
                (
                    "Camera XYZ [m]: "
                    f"{xyz_camera[0]:.3f}, "
                    f"{xyz_camera[1]:.3f}, "
                    f"{xyz_camera[2]:.3f}"
                ),
                (x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.43,
                (255, 255, 0),
                1,
            )
            y += 24

        if xyz_base is not None:
            cv2.putText(
                frame,
                (
                    f"{self._base_frame} XYZ [m]: "
                    f"{xyz_base[0]:.3f}, "
                    f"{xyz_base[1]:.3f}, "
                    f"{xyz_base[2]:.3f}"
                ),
                (x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.43,
                (255, 255, 0),
                1,
            )

    def _fit_window(self, frame: np.ndarray) -> np.ndarray:
        """Return the original frame; do not artificially limit its size."""
        return frame

    def _show(self, frame: np.ndarray) -> None:
        """Display the diagnostic window as a freely resizable window."""

        window_name = "RealSense D435i - Operator XYZ"

        if not hasattr(self, "_window_created"):
            cv2.namedWindow(
                window_name,
                cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO,
            )
            cv2.resizeWindow(
                window_name,
                self._window_width,
                self._window_height,
            )
            self._window_created = True

        cv2.imshow(window_name, frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q") or key == 27:
            rclpy.shutdown()

    # ================================================================
    # HELPERS
    # ================================================================

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _stamp_seconds(msg) -> float:
        return float(msg.header.stamp.sec) + (
            float(msg.header.stamp.nanosec) / 1e9
        )

    def _debug_log(self, message: str) -> None:
        now = self._now_seconds()
        if now - self._last_log_time >= 1.0:
            self.get_logger().warning(message)
            self._last_log_time = now

    # ================================================================
    # SHUTDOWN
    # ================================================================

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
        node = RealSenseOperatorXYZTest()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as exc:
        if node is not None:
            node.get_logger().error(
                f"Error en RealSense Operator XYZ Test: {exc}"
            )
        else:
            print(
                "Error inicializando RealSense Operator XYZ Test: "
                f"{exc}"
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

#!/usr/bin/env python3

"""ROS 2 visual test for ABB IRB 140 RGB-D localization."""

from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PointStamped

import tf2_ros
from tf2_geometry_msgs import do_transform_point

from colab_supervision_operator.perception.robot_detector import RobotDetector
from colab_supervision_operator.perception.pointcloud_object_detector import PointCloudObjectDetector


class RealSenseRobotXYZTest(Node):
    """Validate RealSense RGB -> YOLO -> PointCloud2 -> TF2 for the robot."""

    def __init__(self) -> None:
        super().__init__("realsense_robot_xyz_test")

        self.declare_parameter("rgb_topic", "/IRB140_Cain/D435i/color/image_raw")
        self.declare_parameter("pointcloud_topic", "/IRB140_Cain/D435i/depth/color/points")
        self.declare_parameter("base_frame", "base_link")

        self.declare_parameter("processing_rate", 10.0)
        self.declare_parameter("max_sensor_age", 1.0)
        self.declare_parameter("max_rgb_pointcloud_dt", 0.15)

        package_share = get_package_share_directory("colab_supervision_operator")
        default_model = str(Path(package_share) / "models" / "abb_irb140_yolo.pt")

        self.declare_parameter("robot_model", default_model)
        self.declare_parameter("robot_confidence", 0.50)
        self.declare_parameter("robot_imgsz", 640)
        self.declare_parameter("robot_device", "cpu")
        self.declare_parameter("robot_class_id", 0)

        self.declare_parameter("pointcloud_sample_half_size", 8)
        self.declare_parameter("pointcloud_minimum_valid_points", 10)
        self.declare_parameter("pointcloud_maximum_range", 5.0)

        self.declare_parameter("visualizar", True)
        self.declare_parameter("window_width", 1280)
        self.declare_parameter("window_height", 720)

        self._rgb_topic = str(self.get_parameter("rgb_topic").value)
        self._pointcloud_topic = str(self.get_parameter("pointcloud_topic").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._processing_rate = float(self.get_parameter("processing_rate").value)
        self._max_sensor_age = float(self.get_parameter("max_sensor_age").value)
        self._max_rgb_pointcloud_dt = float(self.get_parameter("max_rgb_pointcloud_dt").value)
        self._robot_model = str(self.get_parameter("robot_model").value)
        self._robot_confidence = float(self.get_parameter("robot_confidence").value)
        self._robot_imgsz = int(self.get_parameter("robot_imgsz").value)
        self._robot_device = str(self.get_parameter("robot_device").value).strip() or None
        self._robot_class_id = int(self.get_parameter("robot_class_id").value)
        self._visualizar = bool(self.get_parameter("visualizar").value)
        self._window_width = int(self.get_parameter("window_width").value)
        self._window_height = int(self.get_parameter("window_height").value)

        if self._processing_rate <= 0.0:
            raise ValueError("processing_rate must be greater than zero")
        if not 0.0 <= self._robot_confidence <= 1.0:
            raise ValueError("robot_confidence must be between 0 and 1")
        if not Path(self._robot_model).is_file():
            raise FileNotFoundError(f"Robot YOLO model not found: {self._robot_model}")

        self._robot_detector = RobotDetector(
            model_path=self._robot_model,
            confidence_threshold=self._robot_confidence,
            image_size=self._robot_imgsz,
            device=self._robot_device,
            robot_class_id=self._robot_class_id,
        )
        self._pointcloud_detector = PointCloudObjectDetector(
            sample_half_size=int(self.get_parameter("pointcloud_sample_half_size").value),
            minimum_valid_points=int(self.get_parameter("pointcloud_minimum_valid_points").value),
            maximum_range=float(self.get_parameter("pointcloud_maximum_range").value),
        )
        self._bridge = CvBridge()
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._latest_rgb: Optional[Image] = None
        self._latest_cloud: Optional[PointCloud2] = None
        self._rgb_receive_time: Optional[float] = None
        self._cloud_receive_time: Optional[float] = None
        self._last_log_time = 0.0

        self.create_subscription(Image, self._rgb_topic, self._rgb_callback, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self._pointcloud_topic, self._pointcloud_callback, qos_profile_sensor_data)
        self._timer = self.create_timer(1.0 / self._processing_rate, self._process)

        self.get_logger().info("====================================================")
        self.get_logger().info("RealSense D435i - ABB IRB 140 XYZ Test")
        self.get_logger().info(f"RGB: {self._rgb_topic}")
        self.get_logger().info(f"PointCloud2: {self._pointcloud_topic}")
        self.get_logger().info(f"Model: {self._robot_model}")
        self.get_logger().info(f"Base frame: {self._base_frame}")
        self.get_logger().info(f"Visualization: {self._visualizar}")
        self.get_logger().info("====================================================")

    def _rgb_callback(self, msg: Image) -> None:
        self._latest_rgb = msg
        self._rgb_receive_time = self._now_seconds()

    def _pointcloud_callback(self, msg: PointCloud2) -> None:
        self._latest_cloud = msg
        self._cloud_receive_time = self._now_seconds()

    def _process(self) -> None:
        now = self._now_seconds()
        if self._latest_rgb is None:
            self._show_status_only("WAITING FOR RGB")
            return
        if self._latest_cloud is None:
            self._show_status_only("WAITING FOR POINTCLOUD2")
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(self._latest_rgb, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self._debug_log(f"RGB conversion error: {exc}")
            self._show_status_only("RGB CONVERSION ERROR")
            return

        cloud = self._latest_cloud
        rgb_age = now - self._rgb_receive_time if self._rgb_receive_time is not None else 999.0
        cloud_age = now - self._cloud_receive_time if self._cloud_receive_time is not None else 999.0
        rgb_ok = rgb_age <= self._max_sensor_age
        cloud_ok = cloud_age <= self._max_sensor_age
        cloud_organized = cloud.height > 1
        sensor_dt = abs(self._stamp_seconds(self._latest_rgb) - self._stamp_seconds(cloud))

        robot_detection = None
        robot_ok = False
        xyz_camera = None
        xyz_base = None
        tf_ok = False
        point_count = 0
        status = "WAITING"

        try:
            detections = self._robot_detector.detect(frame)
            if detections:
                robot_detection = detections[0]
                robot_ok = True
                status = "ROBOT DETECTED"
            else:
                status = "ROBOT NOT DETECTED"
        except Exception as exc:
            status = "YOLO ERROR"
            self._debug_log(f"YOLO robot error: {exc}")

        if robot_detection is not None and cloud_organized and sensor_dt <= self._max_rgb_pointcloud_dt:
            try:
                bbox = robot_detection.bbox
                xyz_camera, point_count = self._bbox_to_3d_aligned(
                    cloud, bbox, frame.shape[1], frame.shape[0]
                ) or (None, 0)
                if xyz_camera is not None:
                    xyz_base = self._transform_to_base(xyz_camera, cloud.header.frame_id)
                    tf_ok = xyz_base is not None
                    status = "ROBOT XYZ OK" if tf_ok else "TF WAITING / INVALID"
                else:
                    status = "ROBOT DEPTH INVALID"
            except Exception as exc:
                status = "3D ERROR"
                self._debug_log(f"XYZ robot error: {exc}")
        elif robot_detection is not None:
            if not cloud_organized:
                status = "POINTCLOUD NOT ORGANIZED"
            elif sensor_dt > self._max_rgb_pointcloud_dt:
                status = "RGB / POINTCLOUD NOT SYNC"

        if self._visualizar:
            visualization = self._fit_window(frame.copy())
            sx = visualization.shape[1] / max(1, frame.shape[1])
            sy = visualization.shape[0] / max(1, frame.shape[0])
            if robot_detection is not None:
                x1, y1, x2, y2 = [int(v) for v in robot_detection.bbox]
                x1, x2 = int(x1 * sx), int(x2 * sx)
                y1, y2 = int(y1 * sy), int(y2 * sy)
                cv2.rectangle(visualization, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.putText(visualization, f"ABB IRB 140 {robot_detection.confidence:.2f}",
                            (x1, max(25, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 0, 0), 2)
                cx = int(robot_detection.center_x * sx)
                cy = int(robot_detection.center_y * sy)
                cv2.circle(visualization, (cx, cy), 5, (0, 255, 255), -1)

            self._draw_status_panel(
                visualization, rgb_ok, cloud_ok, cloud_organized,
                True, robot_ok, xyz_camera is not None, tf_ok,
                status, robot_detection, xyz_camera, xyz_base,
                cloud.header.frame_id, (cloud.width, cloud.height),
                (self._latest_rgb.width, self._latest_rgb.height),
                sensor_dt, point_count,
            )
            self._show(visualization)

        if xyz_base is not None:
            self._debug_log(
                "Robot | Camera XYZ=[%.3f, %.3f, %.3f] m | %s XYZ=[%.3f, %.3f, %.3f] m"
                % (*xyz_camera, self._base_frame, *xyz_base)
            )

    def _bbox_to_3d_aligned(self, cloud, bbox, rgb_width, rgb_height):
        if cloud.width <= 0 or cloud.height <= 0:
            return None
        x1, y1, x2, y2 = bbox
        cx = int(round(((x1 + x2) / 2.0) * cloud.width / max(1, rgb_width)))
        cy = int(round(((y1 + y2) / 2.0) * cloud.height / max(1, rgb_height)))
        cx = int(np.clip(cx, 0, cloud.width - 1))
        cy = int(np.clip(cy, 0, cloud.height - 1))
        half = int(self.get_parameter("pointcloud_sample_half_size").value)
        u1, u2 = max(0, cx - half), min(cloud.width - 1, cx + half)
        v1, v2 = max(0, cy - half), min(cloud.height - 1, cy + half)
        try:
            from sensor_msgs_py import point_cloud2
            points = point_cloud2.read_points_numpy(cloud, field_names=("x", "y", "z"), skip_nans=False)
        except Exception as exc:
            self._debug_log(f"PointCloud2 read error: {exc}")
            return None
        points = np.asarray(points)
        if points.ndim == 3:
            organized = points[:, :, :3]
        elif points.ndim == 2 and points.shape[0] == cloud.width * cloud.height:
            organized = points.reshape(cloud.height, cloud.width, 3)
        else:
            return None
        xyz = organized[v1:v2 + 1, u1:u2 + 1, :].reshape(-1, 3).astype(np.float64)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if xyz.size == 0:
            return None
        d = np.linalg.norm(xyz, axis=1)
        maximum_range = float(self.get_parameter("pointcloud_maximum_range").value)
        xyz = xyz[np.isfinite(d) & (d > 0.05) & (d <= maximum_range)]
        minimum_valid_points = int(self.get_parameter("pointcloud_minimum_valid_points").value)
        if xyz.shape[0] < minimum_valid_points:
            return None
        return np.median(xyz, axis=0), int(xyz.shape[0])

    def _transform_to_base(self, position, source_frame):
        point = PointStamped()
        point.header.frame_id = source_frame
        point.header.stamp = rclpy.time.Time().to_msg()
        point.point.x, point.point.y, point.point.z = map(float, position)
        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame, source_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.10),
            )
            transformed = do_transform_point(point, transform)
            return np.array([transformed.point.x, transformed.point.y, transformed.point.z], dtype=float)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            self._debug_log(f"TF2 {source_frame} -> {self._base_frame}: {exc}")
            return None

    def _draw_status_panel(self, frame, rgb_ok, cloud_ok, cloud_organized, yolo_ok,
                           robot_ok, xyz_ok, tf_ok, status, robot_detection,
                           xyz_camera, xyz_base, cloud_frame, cloud_size,
                           rgb_size, sensor_dt, point_count):
        panel_width = 430
        px = max(0, frame.shape[1] - panel_width)
        overlay = frame.copy()
        cv2.rectangle(overlay, (px, 10), (frame.shape[1] - 5, min(frame.shape[0] - 10, 690)), (20,20,20), -1)
        cv2.addWeighted(overlay, 0.78, frame, 0.22, 0, frame)
        x, y = px + 18, 38
        cv2.putText(frame, "REALSENSE D435i - ROBOT XYZ", (x, y), cv2.FONT_HERSHEY_SIMPLEX, .56, (255,255,255), 2)
        y += 30
        for label, ok in [("RGB stream", rgb_ok), ("PointCloud2", cloud_ok), ("Organized cloud", cloud_organized),
                          ("YOLO robot", yolo_ok), ("Robot", robot_ok), ("3D XYZ", xyz_ok), ("TF -> base_link", tf_ok)]:
            cv2.putText(frame, f"{label:<22}{'OK' if ok else 'WAIT / ERROR'}", (x, y), cv2.FONT_HERSHEY_SIMPLEX, .46,
                        (0,255,0) if ok else (0,0,255), 1)
            y += 23
        y += 5
        cv2.putText(frame, f"STATUS: {status}", (x, y), cv2.FONT_HERSHEY_SIMPLEX, .48, (0,255,255), 2)
        y += 28
        if robot_detection is not None:
            cv2.putText(frame, f"Confidence: {robot_detection.confidence:.3f}", (x,y), cv2.FONT_HERSHEY_SIMPLEX,.44,(220,220,220),1); y += 21
            b = robot_detection.bbox
            cv2.putText(frame, f"BBox: {b[0]:.0f}, {b[1]:.0f}, {b[2]:.0f}, {b[3]:.0f}", (x,y), cv2.FONT_HERSHEY_SIMPLEX,.44,(220,220,220),1); y += 23
        cv2.putText(frame, f"RGB: {rgb_size[0]} x {rgb_size[1]}", (x,y), cv2.FONT_HERSHEY_SIMPLEX,.43,(220,220,220),1); y += 21
        cv2.putText(frame, f"Cloud: {cloud_size[0]} x {cloud_size[1]}", (x,y), cv2.FONT_HERSHEY_SIMPLEX,.43,(220,220,220),1); y += 21
        cv2.putText(frame, f"RGB-PC dt: {sensor_dt:.3f} s", (x,y), cv2.FONT_HERSHEY_SIMPLEX,.43,(220,220,220),1); y += 21
        cv2.putText(frame, f"Valid 3D points: {point_count}", (x,y), cv2.FONT_HERSHEY_SIMPLEX,.43,(220,220,220),1); y += 21
        cv2.putText(frame, f"Cloud frame: {cloud_frame}", (x,y), cv2.FONT_HERSHEY_SIMPLEX,.39,(220,220,220),1); y += 28
        if xyz_camera is not None:
            cv2.putText(frame, f"Camera XYZ [m]: {xyz_camera[0]:.3f}, {xyz_camera[1]:.3f}, {xyz_camera[2]:.3f}", (x,y), cv2.FONT_HERSHEY_SIMPLEX,.43,(255,255,0),1); y += 24
        if xyz_base is not None:
            cv2.putText(frame, f"{self._base_frame} XYZ [m]: {xyz_base[0]:.3f}, {xyz_base[1]:.3f}, {xyz_base[2]:.3f}", (x,y), cv2.FONT_HERSHEY_SIMPLEX,.43,(255,255,0),1)

    def _show_status_only(self, status):
        if not self._visualizar:
            return
        frame = np.zeros((self._window_height, self._window_width, 3), dtype=np.uint8)
        cloud = self._latest_cloud
        self._draw_status_panel(
            frame,
            rgb_ok=self._latest_rgb is not None,
            cloud_ok=cloud is not None,
            cloud_organized=cloud is not None and cloud.height > 1,
            yolo_ok=True,
            robot_ok=False,
            xyz_ok=False,
            tf_ok=False,
            status=status,
            robot_detection=None,
            xyz_camera=None,
            xyz_base=None,
            cloud_frame=cloud.header.frame_id if cloud is not None else "-",
            cloud_size=(cloud.width, cloud.height) if cloud is not None else (0, 0),
            rgb_size=(self._latest_rgb.width, self._latest_rgb.height) if self._latest_rgb is not None else (0, 0),
            sensor_dt=0.0,
            point_count=0,
        )
        self._show(frame)

    def _fit_window(self, frame):
        """Return the original frame without artificial size limits."""
        return frame

    def _show(self, frame):
        """Display the diagnostic window as a freely resizable window."""

        name = "RealSense D435i - ABB IRB 140 XYZ"

        if not hasattr(self, "_window_created"):
            cv2.namedWindow(
                name,
                cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO,
            )
            cv2.resizeWindow(
                name,
                self._window_width,
                self._window_height,
            )
            self._window_created = True

        cv2.imshow(name, frame)

        if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
            rclpy.shutdown()

    def _now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _stamp_seconds(msg):
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9

    def _debug_log(self, message):
        now = self._now_seconds()
        if now - self._last_log_time >= 1.0:
            self.get_logger().warning(message)
            self._last_log_time = now

    def destroy_node(self):
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RealSenseRobotXYZTest()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().error(f"Error en RealSenseRobotXYZTest: {exc}")
        else:
            print(f"Error inicializando RealSenseRobotXYZTest: {exc}")
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

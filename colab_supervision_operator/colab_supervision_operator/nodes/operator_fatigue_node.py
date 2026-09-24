#!/usr/bin/env python3

"""ROS 2 node for RGB-based operator perception."""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from colab_supervision_operator.perception.fatigue_analyzer import FatigueAnalyzer
from colab_supervision_operator.perception.blink_analyzer import BlinkAnalyzer
from colab_supervision_operator.perception.expression_analyzer import ExpressionAnalyzer
from colab_supervision_operator.perception.head_pose_estimator import HeadPoseEstimator
from colab_supervision_operator.perception.attention_analyzer import AttentionAnalyzer
from colab_supervision_operator.perception.person_tracker import PersonTracker
from colab_supervision_operator.perception.face_landmark_detector import FaceLandmarkDetector
from colab_supervision_operator.perception.person_detector import PersonDetector


class OperatorFatigueNode(Node):

    def __init__(self):
        super().__init__("operator_fatigue_node")

        package_share = get_package_share_directory("colab_supervision_operator")
        default_face_model = f"{package_share}/models/face_landmarker.task"

        # ====================================================
        # CÁMARA / VISUALIZACIÓN
        # ====================================================
        self.declare_parameter("camera_device", "/dev/video0")
        self.declare_parameter("camera_width", 1280)
        self.declare_parameter("camera_height", 720)
        self.declare_parameter("camera_fps", 30.0)

        self.declare_parameter("visualizar", False)
        self.declare_parameter("points_face", False)
        self.declare_parameter("visualizar_face_roi", False)
        self.declare_parameter("visualizar_blink", False)
        self.declare_parameter("visualizar_head_pose", False)
        self.declare_parameter("visualizar_pose_axes", False)
        self.declare_parameter("visualizar_attention", False)
        self.declare_parameter("visualizar_expression", False)
        self.declare_parameter("visualizar_fatigue", False)

        # ====================================================
        # ROI FACIAL
        # ====================================================
        self.declare_parameter("face_roi_enabled", True)
        self.declare_parameter("face_roi_upper_ratio", 0.65)
        self.declare_parameter("face_roi_padding_x", 0.15)
        self.declare_parameter("face_roi_padding_y", 0.10)
        self.declare_parameter("face_roi_target_size", 640)

        # ====================================================
        # YOLO
        # ====================================================
        self.declare_parameter("yolo_model", "yolo26n.pt")
        self.declare_parameter("yolo_confidence", 0.5)
        self.declare_parameter("yolo_imgsz", 640)
        self.declare_parameter("yolo_device", "")

        # ====================================================
        # FACE LANDMARKER
        # ====================================================
        self.declare_parameter("face_landmarker_model", default_face_model)
        self.declare_parameter("face_detection_confidence", 0.5)
        self.declare_parameter("face_tracking_confidence", 0.5)

        # ====================================================
        # EXPRESSION
        # ====================================================
        self.declare_parameter("expression_threshold", 0.28)
        self.declare_parameter("expression_sleepy_threshold", 0.35)
        self.declare_parameter("expression_tired_threshold", 0.35)
        self.declare_parameter("expression_fatigued_threshold", 0.45)
        self.declare_parameter("expression_yawn_threshold", 0.38)
        self.declare_parameter("expression_smoothing_alpha", 0.35)
        self.declare_parameter("expression_sleepy_frames", 6)
        self.declare_parameter("expression_tired_frames", 10)
        self.declare_parameter("expression_fatigued_frames", 20)
        self.declare_parameter("expression_yawn_frames", 8)

        # ====================================================
        # FATIGA / CABECEO
        # ====================================================
        self.declare_parameter("fatigue_eye_openness_threshold", 0.30)
        self.declare_parameter("fatigue_minimum_blink_frequency", 5.0)
        self.declare_parameter("head_nod_pitch_change", 0.8)
        self.declare_parameter("head_nod_min_angle", 8.0)
        self.declare_parameter("head_nod_window_seconds", 5.0)
        self.declare_parameter("head_nod_threshold", 0.40)
        self.declare_parameter("head_nod_max_reversals", 4)

        # ====================================================
        # LEER PARÁMETROS
        # ====================================================
        self._camera_device = self.get_parameter("camera_device").value
        self._camera_width = int(self.get_parameter("camera_width").value)
        self._camera_height = int(self.get_parameter("camera_height").value)
        self._camera_fps = float(self.get_parameter("camera_fps").value)

        self._visualizar = bool(self.get_parameter("visualizar").value)
        self._points_face = bool(self.get_parameter("points_face").value)
        self._visualizar_face_roi = bool(self.get_parameter("visualizar_face_roi").value)
        self._visualizar_blink = bool(self.get_parameter("visualizar_blink").value)
        self._visualizar_head_pose = bool(self.get_parameter("visualizar_head_pose").value)
        self._visualizar_pose_axes = bool(self.get_parameter("visualizar_pose_axes").value)
        self._visualizar_attention = bool(self.get_parameter("visualizar_attention").value)
        self._visualizar_expression = bool(self.get_parameter("visualizar_expression").value)
        self._visualizar_fatigue = bool(self.get_parameter("visualizar_fatigue").value)

        self._face_roi_enabled = bool(self.get_parameter("face_roi_enabled").value)
        self._face_roi_upper_ratio = float(self.get_parameter("face_roi_upper_ratio").value)
        self._face_roi_padding_x = float(self.get_parameter("face_roi_padding_x").value)
        self._face_roi_padding_y = float(self.get_parameter("face_roi_padding_y").value)
        self._face_roi_target_size = int(self.get_parameter("face_roi_target_size").value)

        yolo_model = self.get_parameter("yolo_model").value
        yolo_confidence = float(self.get_parameter("yolo_confidence").value)
        yolo_imgsz = int(self.get_parameter("yolo_imgsz").value)
        yolo_device = self.get_parameter("yolo_device").value
        if yolo_device == "":
            yolo_device = None

        face_model = self.get_parameter("face_landmarker_model").value
        face_detection_confidence = float(self.get_parameter("face_detection_confidence").value)
        face_tracking_confidence = float(self.get_parameter("face_tracking_confidence").value)

        expression_threshold = float(self.get_parameter("expression_threshold").value)
        expression_sleepy_threshold = float(self.get_parameter("expression_sleepy_threshold").value)
        expression_tired_threshold = float(self.get_parameter("expression_tired_threshold").value)
        expression_fatigued_threshold = float(self.get_parameter("expression_fatigued_threshold").value)
        expression_yawn_threshold = float(self.get_parameter("expression_yawn_threshold").value)
        expression_smoothing_alpha = float(self.get_parameter("expression_smoothing_alpha").value)
        expression_sleepy_frames = int(self.get_parameter("expression_sleepy_frames").value)
        expression_tired_frames = int(self.get_parameter("expression_tired_frames").value)
        expression_fatigued_frames = int(self.get_parameter("expression_fatigued_frames").value)
        expression_yawn_frames = int(self.get_parameter("expression_yawn_frames").value)

        # ====================================================
        # CÁMARA
        # ====================================================
        self._camera = None
        self._initialize_camera()

        # ====================================================
        # DETECTORES
        # ====================================================
        self._person_detector = PersonDetector(
            model_path=yolo_model,
            confidence_threshold=yolo_confidence,
            image_size=yolo_imgsz,
            device=yolo_device,
        )
        self._person_tracker = PersonTracker()

        self._face_landmark_detector = FaceLandmarkDetector(
            model_path=face_model,
            min_detection_confidence=face_detection_confidence,
            min_tracking_confidence=face_tracking_confidence,
        )

        # ====================================================
        # ANALIZADORES
        # ====================================================
        self._blink_analyzer = BlinkAnalyzer(
            eye_closed_threshold=0.23,
            min_closed_frames=1,
            max_closed_frames=20,
            fps=self._camera_fps,
            frequency_window_seconds=60.0,
            openness_min_ear=0.10,
            openness_max_ear=0.35,
        )

        self._head_pose_estimator = HeadPoseEstimator()
        self._attention_analyzer = AttentionAnalyzer()

        self._expression_analyzer = ExpressionAnalyzer(
            expression_threshold=expression_threshold,
            sleepy_threshold=expression_sleepy_threshold,
            tired_threshold=expression_tired_threshold,
            fatigued_threshold=expression_fatigued_threshold,
            yawn_threshold=expression_yawn_threshold,
            smoothing_alpha=expression_smoothing_alpha,
            sleepy_frames=expression_sleepy_frames,
            tired_frames=expression_tired_frames,
            fatigued_frames=expression_fatigued_frames,
            yawn_frames=expression_yawn_frames,
        )

        self._fatigue_analyzer = FatigueAnalyzer(
            eye_openness_threshold=float(self.get_parameter("fatigue_eye_openness_threshold").value),
            minimum_blink_frequency=float(self.get_parameter("fatigue_minimum_blink_frequency").value),
            fatigue_expressions={"sleepy", "tired", "fatigued", "yawn"},
            head_nod_pitch_change=float(self.get_parameter("head_nod_pitch_change").value),
            head_nod_min_angle=float(self.get_parameter("head_nod_min_angle").value),
            head_nod_window_seconds=float(self.get_parameter("head_nod_window_seconds").value),
            head_nod_threshold=float(self.get_parameter("head_nod_threshold").value),
            head_nod_max_reversals=int(self.get_parameter("head_nod_max_reversals").value),
        )

        timer_period = 1.0 / self._camera_fps
        self._timer = self.create_timer(timer_period, self._process_frame)

        self.get_logger().info("==============================================")
        self.get_logger().info("Operator Fatigue Node iniciado")
        self.get_logger().info(f"Cámara: {self._camera_device}")
        self.get_logger().info(f"Resolución: {self._camera_width}x{self._camera_height}")
        self.get_logger().info(f"FPS: {self._camera_fps}")
        self.get_logger().info(f"YOLO device: {yolo_device if yolo_device is not None else 'auto'}")
        self.get_logger().info("BlinkAnalyzer inicializado correctamente")
        self.get_logger().info("ExpressionAnalyzer inicializado correctamente")
        self.get_logger().info("FatigueAnalyzer inicializado correctamente")
        self.get_logger().info("==============================================")

    def _initialize_camera(self):
        self._camera = cv2.VideoCapture(self._camera_device, cv2.CAP_V4L2)
        if not self._camera.isOpened():
            raise RuntimeError(f"No fue posible abrir la cámara {self._camera_device}")

        self._camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self._camera.set(cv2.CAP_PROP_FRAME_WIDTH, self._camera_width)
        self._camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self._camera_height)
        self._camera.set(cv2.CAP_PROP_FPS, self._camera_fps)

        actual_width = int(self._camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self._camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self._camera.get(cv2.CAP_PROP_FPS)

        fourcc = int(self._camera.get(cv2.CAP_PROP_FOURCC))
        fourcc_string = "".join(chr((fourcc >> (8 * i)) & 0xFF) for i in range(4))

        self.get_logger().info(f"Cámara inicializada: {actual_width}x{actual_height} @ {actual_fps:.2f} FPS ({fourcc_string})")

        ret, frame = self._camera.read()
        if not ret or frame is None:
            self._camera.release()
            self._camera = None
            raise RuntimeError("La cámara fue abierta, pero no fue posible capturar el primer frame.")

    def _process_frame(self):
        if self._camera is None:
            return

        ret, frame = self._camera.read()
        if not ret or frame is None:
            self.get_logger().warning("No fue posible obtener un frame.")
            return

        display_frame = frame.copy()

        try:
            persons = self._detect_persons(frame)
            tracking_result = self._person_tracker.update(persons)
        except Exception as e:
            self.get_logger().error(f"Error en detección/tracking: {e}")
            self._show_frame(display_frame)
            return

        if not tracking_result.person_detected:
            self._reset_temporal_analyzers()
            self._draw_no_operator(display_frame)
            self._show_frame(display_frame)
            return

        primary_person = self._person_tracker.get_primary_person()
        if primary_person is None:
            self._reset_temporal_analyzers()
            self._draw_no_operator(display_frame)
            self._show_frame(display_frame)
            return

        self._draw_person(display_frame, primary_person)

        face_input = frame
        roi_transform = None
        roi_data = None

        if self._face_roi_enabled:
            roi_data = self._extract_face_roi(frame, primary_person)
            if roi_data is not None:
                face_input = roi_data["image"]
                roi_transform = roi_data["transform"]
                if self._visualizar_face_roi:
                    self._draw_face_roi(display_frame, roi_data)

        try:
            face_result = self._face_landmark_detector.update(face_input)
            if face_result.face_detected and roi_transform is not None:
                self._restore_face_coordinates(face_result, roi_transform)
        except Exception as e:
            self.get_logger().error(f"Error en FaceLandmarkDetector: {e}")
            self._show_frame(display_frame)
            return

        if not face_result.face_detected:
            self._reset_temporal_analyzers()
            self._draw_no_face(display_frame, primary_person)
            self._show_frame(display_frame)
            return

        display_frame = self._draw_face_information(display_frame, face_result)

        # ====================================================
        # BLINK
        # ====================================================
        blink_result = None
        if face_result.left_eye_points is not None and face_result.right_eye_points is not None:
            try:
                blink_result = self._blink_analyzer.update(
                    face_result.left_eye_points,
                    face_result.right_eye_points,
                )
            except Exception as e:
                self.get_logger().error(f"Error en BlinkAnalyzer: {e}")

        # ====================================================
        # HEAD POSE
        # ====================================================
        head_pose_result = None
        if face_result.head_pose_points is not None:
            try:
                head_pose_result = self._head_pose_estimator.update(
                    face_result.head_pose_points,
                    (frame.shape[1], frame.shape[0]),
                )
            except Exception as e:
                self.get_logger().error(f"Error en HeadPoseEstimator: {e}")

        # ====================================================
        # ATTENTION
        # ====================================================
        attention_result = None
        if head_pose_result is not None and head_pose_result.pose_valid:
            try:
                attention_result = self._attention_analyzer.update(
                    head_pose_result.yaw,
                    head_pose_result.pitch,
                    head_pose_result.roll,
                )
            except Exception as e:
                self.get_logger().error(f"Error en AttentionAnalyzer: {e}")

        # ====================================================
        # EXPRESSION
        # ====================================================
        expression_result = None
        if face_result.face_blendshapes:
            try:
                expression_result = self._expression_analyzer.update(
                    face_result.face_blendshapes
                )
            except Exception as e:
                self.get_logger().error(f"Error en ExpressionAnalyzer: {e}")

        # ====================================================
        # FATIGUE
        # ====================================================
        fatigue_result = None
        if (
            blink_result is not None
            and head_pose_result is not None
            and head_pose_result.pose_valid
            and expression_result is not None
        ):
            try:
                timestamp = self.get_clock().now().nanoseconds / 1e9
                fatigue_result = self._fatigue_analyzer.update(
                    eye_openness=blink_result.eye_openness,
                    blink_frequency=blink_result.blink_frequency,
                    facial_expression=expression_result.expression,
                    head_pitch=head_pose_result.pitch,
                    timestamp=timestamp,
                )
            except Exception as e:
                self.get_logger().error(f"Error en FatigueAnalyzer: {e}")

        # ====================================================
        # VISUALIZACIONES
        # ====================================================
        if self._visualizar_blink and blink_result is not None:
            display_frame = self._draw_blink_information(display_frame, blink_result)

        if self._visualizar_head_pose and head_pose_result is not None:
            display_frame = self._draw_head_pose_information(display_frame, head_pose_result)

        if self._visualizar_pose_axes and head_pose_result is not None and head_pose_result.pose_valid:
            try:
                display_frame = self._head_pose_estimator.draw_pose_axes(
                    display_frame,
                    head_pose_result,
                )
            except AttributeError:
                pass
            except Exception as e:
                self.get_logger().warning(f"Error dibujando ejes de pose: {e}")

        if self._visualizar_attention and attention_result is not None:
            display_frame = self._draw_attention_information(display_frame, attention_result)

        if self._visualizar_expression and expression_result is not None:
            display_frame = self._draw_expression_information(display_frame, expression_result)

        if self._visualizar_fatigue and fatigue_result is not None:
            display_frame = self._draw_fatigue_information(display_frame, fatigue_result)

        self._show_frame(display_frame)

    def _reset_temporal_analyzers(self):
        try:
            self._blink_analyzer.reset()
        except Exception:
            pass

        try:
            self._expression_analyzer.reset()
        except Exception:
            pass

        try:
            self._fatigue_analyzer.reset()
        except Exception:
            pass

    def _detect_persons(self, frame):
        return self._person_detector.update(frame)

    def _extract_face_roi(self, frame, person):
        height, width = frame.shape[:2]

        x1 = max(0, int(person.x1))
        y1 = max(0, int(person.y1))
        x2 = min(width, int(person.x2))
        y2 = min(height, int(person.y2))

        if x2 <= x1 or y2 <= y1:
            return None

        person_width = x2 - x1
        person_height = y2 - y1

        roi_height = max(1, int(person_height * self._face_roi_upper_ratio))
        face_y2 = min(y2, y1 + roi_height)

        pad_x = int(person_width * self._face_roi_padding_x)
        pad_y = int(person_height * self._face_roi_padding_y)

        roi_x1 = max(0, x1 - pad_x)
        roi_y1 = max(0, y1 - pad_y)
        roi_x2 = min(width, x2 + pad_x)
        roi_y2 = min(height, face_y2 + pad_y)

        if roi_x2 <= roi_x1 or roi_y2 <= roi_y1:
            return None

        roi = frame[roi_y1:roi_y2, roi_x1:roi_x2]
        if roi.size == 0:
            return None

        roi_h, roi_w = roi.shape[:2]
        short_side = min(roi_w, roi_h)
        scale = 1.0 if short_side >= self._face_roi_target_size else self._face_roi_target_size / float(short_side)

        new_width = max(1, int(round(roi_w * scale)))
        new_height = max(1, int(round(roi_h * scale)))

        if scale != 1.0:
            roi_resized = cv2.resize(roi, (new_width, new_height), interpolation=cv2.INTER_CUBIC)
        else:
            roi_resized = roi

        transform = {
            "offset_x": float(roi_x1),
            "offset_y": float(roi_y1),
            "scale_x": float(new_width / roi_w),
            "scale_y": float(new_height / roi_h),
        }

        return {
            "image": roi_resized,
            "transform": transform,
            "bbox": (roi_x1, roi_y1, roi_x2, roi_y2),
        }

    def _restore_face_coordinates(self, face_result, transform):
        offset_x = float(transform["offset_x"])
        offset_y = float(transform["offset_y"])
        scale_x = float(transform["scale_x"])
        scale_y = float(transform["scale_y"])

        def restore_points(points):
            if points is None:
                return None
            points = np.asarray(points, dtype=np.float32).copy()
            points[:, 0] = points[:, 0] / scale_x + offset_x
            points[:, 1] = points[:, 1] / scale_y + offset_y
            return points

        if face_result.landmarks is not None:
            face_result.landmarks = restore_points(face_result.landmarks)
        if face_result.head_pose_points is not None:
            face_result.head_pose_points = restore_points(face_result.head_pose_points)
        if face_result.left_eye_points is not None:
            face_result.left_eye_points = restore_points(face_result.left_eye_points)
        if face_result.right_eye_points is not None:
            face_result.right_eye_points = restore_points(face_result.right_eye_points)
        if face_result.mouth_points is not None:
            face_result.mouth_points = restore_points(face_result.mouth_points)

        if face_result.bbox is not None:
            x1, y1, x2, y2 = face_result.bbox
            face_result.bbox = (
                int(round(float(x1) / scale_x + offset_x)),
                int(round(float(y1) / scale_y + offset_y)),
                int(round(float(x2) / scale_x + offset_x)),
                int(round(float(y2) / scale_y + offset_y)),
            )

        if face_result.nose_point is not None:
            x, y = face_result.nose_point
            face_result.nose_point = (
                int(round(float(x) / scale_x + offset_x)),
                int(round(float(y) / scale_y + offset_y)),
            )

    def _draw_face_roi(self, image, roi_data):
        x1, y1, x2, y2 = roi_data["bbox"]
        cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 255), 2)
        cv2.putText(image, "Face ROI", (int(x1), max(20, int(y1) - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 255), 2)
        return image

    def _draw_person(self, image, person):
        x1, y1, x2, y2 = map(int, (person.x1, person.y1, person.x2, person.y2))
        cv2.rectangle(image, (x1, y1), (x2, y2), (255, 0, 0), 2)
        label = f"Operator ID:{person.track_id} {person.confidence:.2f}"
        cv2.putText(image, label, (x1, max(20, y1 - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        return image

    def _draw_face_information(self, image, face_result):
        if face_result.bbox is not None:
            x1, y1, x2, y2 = face_result.bbox
            cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(image, "Face detected", (int(x1), max(20, int(y1) - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if self._points_face:
            image = self._face_landmark_detector.draw_landmarks(image, face_result)

        return image

    def _draw_blink_information(self, image, result):
        x, y, line = 700, 40, 25
        lines = [
            f"Left EAR: {result.left_ear:.2f}",
            f"Right EAR: {result.right_ear:.2f}",
            f"Eye openness: {result.eye_openness:.2f}",
            f"Eyes closed: {result.eyes_closed}",
            f"Blink detected: {result.blink_detected}",
            f"Blink duration: {result.blink_duration:.2f}s",
            f"Blink frequency: {result.blink_frequency:.2f}",
            f"Total blinks: {result.total_blinks}",
        ]
        for i, text in enumerate(lines):
            cv2.putText(image, text, (x, y + i * line), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)
        return image

    def _draw_head_pose_information(self, image, result):
        x, y, line = 20, 320, 25
        lines = [
            f"Yaw: {result.yaw:.1f}",
            f"Pitch: {result.pitch:.1f}",
            f"Roll: {result.roll:.1f}",
            f"Valid: {result.pose_valid}",
        ]
        for i, text in enumerate(lines):
            cv2.putText(image, text, (x, y + i * line), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        return image

    def _draw_attention_information(self, image, result):
        x, y, line = 20, 440, 25
        lines = [
            f"Attention: {result.attention_target}",
            f"Score: {result.attention_score:.2f}",
            f"Task: {result.task_attention}",
            f"HMI: {result.hmi_attention}",
            f"Distracted: {result.distracted}",
        ]
        for i, text in enumerate(lines):
            cv2.putText(image, text, (x, y + i * line), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        return image

    def _draw_expression_information(self, image, result):
        x, y, line = 20, 40, 24
        lines = [
            f"Expression: {result.expression}",
            f"Confidence: {result.confidence:.2f}",
            f"Smile: {result.smile_score:.2f}",
            f"Frown: {result.frown_score:.2f}",
            f"Surprise: {result.surprise_score:.2f}",
            f"Neutral: {result.neutral_score:.2f}",
            f"Sleepy: {result.sleepy_score:.2f}",
            f"Tired: {result.tired_score:.2f}",
            f"Fatigued: {result.fatigued_score:.2f}",
            f"Yawn: {result.yawn_score:.2f}",
        ]
        for i, text in enumerate(lines):
            cv2.putText(image, text, (x, y + i * line), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 255), 2)
        return image

    def _draw_fatigue_information(self, image, result):
        x, y, line = 700, 280, 25
        lines = [
            f"Fatigue: {result.fatigue_level}",
            f"Fatigue score: {result.fatigue_score:.2f}",
            f"Eye openness: {result.eye_openness:.2f}",
            f"Blink frequency: {result.blink_frequency:.2f}",
            f"Expression: {result.facial_expression}",
            f"Head pitch: {result.head_pitch:.1f}",
            f"Nodding score: {result.head_nodding_score:.2f}",
            f"Eye fatigue: {result.eye_openness_fatigued}",
            f"Blink fatigue: {result.blink_frequency_fatigued}",
            f"Expression fatigue: {result.facial_expression_fatigued}",
            f"Nodding fatigue: {result.head_nodding_fatigued}",
        ]
        for i, text in enumerate(lines):
            cv2.putText(image, text, (x, y + i * line), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 0), 2)
        return image

    def _draw_no_operator(self, image):
        cv2.putText(image, "No operator detected", (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        return image

    def _draw_no_face(self, image, primary_person):
        x1 = int(primary_person.x1)
        y1 = int(primary_person.y1)
        cv2.putText(image, "Face not detected", (x1, max(20, y1 - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        return image

    def _show_frame(self, image):
        if not self._visualizar:
            return
        cv2.imshow("Operator RGB Camera", image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            rclpy.shutdown()

    def destroy_node(self):
        if self._camera is not None:
            self._camera.release()
            self._camera = None
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        try:
            if getattr(self, "_face_landmark_detector", None) is not None:
                self._face_landmark_detector.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OperatorFatigueNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node is not None:
            node.get_logger().error(f"Error en OperatorFatigueNode: {e}")
        else:
            print(f"Error inicializando OperatorFatigueNode: {e}")
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

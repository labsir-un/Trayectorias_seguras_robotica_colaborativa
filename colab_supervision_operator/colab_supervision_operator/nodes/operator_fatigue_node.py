#!/usr/bin/env python3

"""
Nodo ROS 2 para percepción del operador.

Este nodo:
- Captura imágenes desde una cámara RGB.
- Permite visualizar la imagen mediante un parámetro.
- Ejecuta los módulos de percepción del operador.
- Detecta personas mediante YOLO.
- Realiza seguimiento mediante PersonTracker.
- Detecta landmarks faciales mediante MediaPipe Face Landmarker.
- Analiza parpadeos mediante BlinkAnalyzer.
- Estima orientación de cabeza mediante HeadPoseEstimator.
- Permite visualizar datos de parpadeo y orientación de cabeza.
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from colab_supervision_operator.perception.fatigue_analyzer import FatigueAnalyzer
from colab_supervision_operator.perception.blink_analyzer import BlinkAnalyzer
from colab_supervision_operator.perception.expression_analyzer import ExpressionAnalyzer
from colab_supervision_operator.perception.head_pose_estimator import HeadPoseEstimator
from colab_supervision_operator.perception.attention_analyzer import AttentionAnalyzer
from colab_supervision_operator.perception.person_tracker import PersonTracker
from colab_supervision_operator.perception.face_landmark_detector import FaceLandmarkDetector
from colab_supervision_operator.perception.person_detector import PersonDetector

from ament_index_python.packages import get_package_share_directory


class OperatorFatigueNode(Node):

    # ========================================================
    # Constructor
    # ========================================================

    def __init__(self):

        super().__init__(
            "operator_fatigue_node"
        )

        package_share = get_package_share_directory(
            "colab_supervision_operator"
        )

        default_face_model = (
            f"{package_share}/models/face_landmarker.task"
        )

        # ====================================================
        # PARÁMETROS
        # ====================================================

        # ----------------------------------------------------
        # Cámara
        # ----------------------------------------------------

        self.declare_parameter(
            "camera_device",
            "/dev/video0"
        )

        self.declare_parameter(
            "camera_width",
            1280
        )

        self.declare_parameter(
            "camera_height",
            720
        )

        self.declare_parameter(
            "camera_fps",
            30.0
        )

        # ----------------------------------------------------
        # Visualización general
        # ----------------------------------------------------

        self.declare_parameter(
            "visualizar",
            False
        )

        self.declare_parameter(
            "points_face",
            False
        )

        # ----------------------------------------------------
        # Visualización Blink
        # ----------------------------------------------------

        self.declare_parameter(
            "visualizar_blink",
            False
        )

        # ----------------------------------------------------
        # Visualización Head Pose
        # ----------------------------------------------------

        self.declare_parameter(
            "visualizar_head_pose",
            False
        )

        # ----------------------------------------------------
        # Visualización ejes de pose
        # ----------------------------------------------------

        self.declare_parameter(
            "visualizar_pose_axes",
            False
        )

        # ----------------------------------------------------
        # Visualización Attention
        # ----------------------------------------------------

        self.declare_parameter(
            "visualizar_attention",
            False
        )

        # ----------------------------------------------------
        # Visualización ROI facial
        # ----------------------------------------------------

        self.declare_parameter(
            "visualizar_face_roi",
            False
        )

        # ----------------------------------------------------
        # ROI facial
        # ----------------------------------------------------

        self.declare_parameter(
            "face_roi_enabled",
            True
        )

        self.declare_parameter(
            "face_roi_upper_ratio",
            0.65
        )

        self.declare_parameter(
            "face_roi_padding_x",
            0.15
        )

        self.declare_parameter(
            "face_roi_padding_y",
            0.10
        )

        self.declare_parameter(
            "face_roi_target_size",
            640
        )

        # ----------------------------------------------------
        # YOLO
        # ----------------------------------------------------

        self.declare_parameter(
            "yolo_model",
            "yolo26n.pt"
        )

        self.declare_parameter(
            "yolo_confidence",
            0.5
        )

        self.declare_parameter(
            "yolo_imgsz",
            640
        )

        self.declare_parameter(
            "yolo_device",
            ""
        )

        # ----------------------------------------------------
        # MediaPipe Face Landmarker
        # ----------------------------------------------------

        self.declare_parameter(
            "face_landmarker_model",
            default_face_model
        )

        self.declare_parameter(
            "face_detection_confidence",
            0.5
        )

        self.declare_parameter(
            "face_tracking_confidence",
            0.5
        )

        # ====================================================
        # OBTENER PARÁMETROS
        # ====================================================

        self._camera_device = self.get_parameter(
            "camera_device"
        ).value

        self._camera_width = self.get_parameter(
            "camera_width"
        ).value

        self._camera_height = self.get_parameter(
            "camera_height"
        ).value

        self._camera_fps = self.get_parameter(
            "camera_fps"
        ).value

        self._visualizar = self.get_parameter(
            "visualizar"
        ).value

        self._points_face = self.get_parameter(
            "points_face"
        ).value

        self._visualizar_blink = self.get_parameter(
            "visualizar_blink"
        ).value

        self._visualizar_head_pose = self.get_parameter(
            "visualizar_head_pose"
        ).value

        self._visualizar_pose_axes = self.get_parameter(
            "visualizar_pose_axes"
        ).value

        self._visualizar_attention = self.get_parameter(
            "visualizar_attention"
        ).value

        self._visualizar_face_roi = self.get_parameter(
            "visualizar_face_roi"
        ).value

        # ----------------------------------------------------
        # ROI
        # ----------------------------------------------------

        self._face_roi_enabled = self.get_parameter(
            "face_roi_enabled"
        ).value

        self._face_roi_upper_ratio = self.get_parameter(
            "face_roi_upper_ratio"
        ).value

        self._face_roi_padding_x = self.get_parameter(
            "face_roi_padding_x"
        ).value

        self._face_roi_padding_y = self.get_parameter(
            "face_roi_padding_y"
        ).value

        self._face_roi_target_size = self.get_parameter(
            "face_roi_target_size"
        ).value

        # ----------------------------------------------------
        # YOLO
        # ----------------------------------------------------

        yolo_model = self.get_parameter(
            "yolo_model"
        ).value

        yolo_confidence = self.get_parameter(
            "yolo_confidence"
        ).value

        yolo_imgsz = self.get_parameter(
            "yolo_imgsz"
        ).value

        yolo_device = self.get_parameter(
            "yolo_device"
        ).value

        if yolo_device == "":
            yolo_device = None

        # ----------------------------------------------------
        # MediaPipe
        # ----------------------------------------------------

        face_model = self.get_parameter(
            "face_landmarker_model"
        ).value

        face_detection_confidence = (
            self.get_parameter(
                "face_detection_confidence"
            ).value
        )

        face_tracking_confidence = (
            self.get_parameter(
                "face_tracking_confidence"
            ).value
        )

        # ====================================================
        # INFORMACIÓN DE INICIALIZACIÓN
        # ====================================================

        self.get_logger().info(
            "=============================================="
        )

        self.get_logger().info(
            "Inicializando Operator Fatigue Node"
        )

        self.get_logger().info(
            f"Cámara: {self._camera_device}"
        )

        self.get_logger().info(
            "Resolución solicitada: "
            f"{self._camera_width}x"
            f"{self._camera_height}"
        )

        self.get_logger().info(
            f"FPS solicitado: {self._camera_fps}"
        )

        self.get_logger().info(
            f"Visualización: {self._visualizar}"
        )

        self.get_logger().info(
            f"Landmarks faciales: {self._points_face}"
        )

        self.get_logger().info(
            f"Visualización Blink: "
            f"{self._visualizar_blink}"
        )

        self.get_logger().info(
            f"Visualización Head Pose: "
            f"{self._visualizar_head_pose}"
        )

        self.get_logger().info(
            f"Visualización Pose Axes: "
            f"{self._visualizar_pose_axes}"
        )

        self.get_logger().info(
            f"Visualización Attention: "
            f"{self._visualizar_attention}"
        )

        self.get_logger().info(
            f"Visualización Face ROI: "
            f"{self._visualizar_face_roi}"
        )

        self.get_logger().info(
            f"Face ROI habilitada: "
            f"{self._face_roi_enabled}"
        )

        self.get_logger().info(
            f"Face ROI upper ratio: "
            f"{self._face_roi_upper_ratio}"
        )

        self.get_logger().info(
            f"Face ROI target size: "
            f"{self._face_roi_target_size}"
        )

        self.get_logger().info(
            f"Modelo YOLO: {yolo_model}"
        )

        self.get_logger().info(
            f"Modelo Face Landmarker: {face_model}"
        )

        self.get_logger().info(
            "=============================================="
        )

        # ====================================================
        # CÁMARA
        # ====================================================

        self._camera = None

        self._initialize_camera()

        # ====================================================
        # DETECTOR DE PERSONAS
        # ====================================================

        self.get_logger().info(
            "Inicializando PersonDetector..."
        )

        try:

            self._person_detector = PersonDetector(
                model_path=yolo_model,
                confidence_threshold=yolo_confidence,
                image_size=yolo_imgsz,
                device=yolo_device,
            )

        except Exception as e:

            self.get_logger().error(
                "Error inicializando PersonDetector: "
                f"{e}"
            )

            raise

        self.get_logger().info(
            "PersonDetector inicializado correctamente."
        )

        # ====================================================
        # TRACKER
        # ====================================================

        self.get_logger().info(
            "Inicializando PersonTracker..."
        )

        self._person_tracker = PersonTracker()

        self.get_logger().info(
            "PersonTracker inicializado correctamente."
        )

        # ====================================================
        # FACE LANDMARK DETECTOR
        # ====================================================

        self.get_logger().info(
            "Inicializando FaceLandmarkDetector..."
        )

        try:

            self._face_landmark_detector = (
                FaceLandmarkDetector(
                    model_path=face_model,
                    min_detection_confidence=(
                        face_detection_confidence
                    ),
                    min_tracking_confidence=(
                        face_tracking_confidence
                    ),
                )
            )

        except Exception as e:

            self.get_logger().error(
                "Error inicializando "
                "FaceLandmarkDetector: "
                f"{e}"
            )

            raise

        self.get_logger().info(
            "FaceLandmarkDetector "
            "inicializado correctamente."
        )

        # ====================================================
        # BLINK ANALYZER
        # ====================================================

        self.get_logger().info(
            "Inicializando BlinkAnalyzer..."
        )

        self._blink_analyzer = BlinkAnalyzer(
            eye_closed_threshold=0.23,
            min_closed_frames=1,
            max_closed_frames=20,
            fps=float(self._camera_fps),
            frequency_window_seconds=60.0,
            openness_min_ear=0.10,
            openness_max_ear=0.35,
        )

        self.get_logger().info(
            "BlinkAnalyzer inicializado correctamente."
        )

        # ====================================================
        # HEAD POSE ESTIMATOR
        # ====================================================

        self.get_logger().info(
            "Inicializando HeadPoseEstimator..."
        )

        self._head_pose_estimator = HeadPoseEstimator()

        self.get_logger().info(
            "HeadPoseEstimator inicializado correctamente."
        )

        # ====================================================
        # ATTENTION ANALYZER
        # ====================================================

        self.get_logger().info(
            "Inicializando AttentionAnalyzer..."
        )

        self._attention_analyzer = AttentionAnalyzer()

        self.get_logger().info(
            "AttentionAnalyzer inicializado correctamente."
        )

        # ====================================================
        # TIMER
        # ====================================================

        timer_period = 1.0 / float(
            self._camera_fps
        )

        self._timer = self.create_timer(
            timer_period,
            self._process_frame
        )

        self.get_logger().info(
            "Operator Fatigue Node iniciado."
        )

        self.get_logger().info(
            "=============================================="
        )

    # ========================================================
    # INICIALIZAR CÁMARA
    # ========================================================

    def _initialize_camera(self):

        self.get_logger().info(
            "Inicializando cámara mediante V4L2..."
        )

        self._camera = cv2.VideoCapture(
            self._camera_device,
            cv2.CAP_V4L2
        )

        if not self._camera.isOpened():

            raise RuntimeError(
                f"No fue posible abrir la cámara "
                f"{self._camera_device}"
            )

        self._camera.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(
                *"MJPG"
            )
        )

        self._camera.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            self._camera_width
        )

        self._camera.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            self._camera_height
        )

        self._camera.set(
            cv2.CAP_PROP_FPS,
            self._camera_fps
        )

        actual_width = int(
            self._camera.get(
                cv2.CAP_PROP_FRAME_WIDTH
            )
        )

        actual_height = int(
            self._camera.get(
                cv2.CAP_PROP_FRAME_HEIGHT
            )
        )

        actual_fps = self._camera.get(
            cv2.CAP_PROP_FPS
        )

        fourcc = int(
            self._camera.get(
                cv2.CAP_PROP_FOURCC
            )
        )

        fourcc_string = "".join(
            [
                chr(
                    (fourcc >> (8 * i)) & 0xFF
                )
                for i in range(4)
            ]
        )

        self.get_logger().info(
            "Cámara inicializada:"
        )

        self.get_logger().info(
            f"  Resolución: "
            f"{actual_width}x{actual_height}"
        )

        self.get_logger().info(
            f"  FPS: {actual_fps:.2f}"
        )

        self.get_logger().info(
            f"  Formato: {fourcc_string}"
        )

        ret, frame = self._camera.read()

        if not ret or frame is None:

            self._camera.release()

            self._camera = None

            raise RuntimeError(
                "La cámara fue abierta, "
                "pero no fue posible capturar "
                "el primer frame."
            )

        self.get_logger().info(
            "Primer frame capturado correctamente."
        )

    # ========================================================
    # PROCESAR FRAME
    # ========================================================

    def _process_frame(self):

        if self._camera is None:
            return

        ret, frame = self._camera.read()

        if not ret or frame is None:

            self.get_logger().warning(
                "No fue posible obtener un frame."
            )

            return

        display_frame = frame.copy()

        # ====================================================
        # DETECCIÓN DE PERSONAS
        # ====================================================

        try:

            persons = self._detect_persons(
                frame
            )

        except Exception as e:

            self.get_logger().error(
                f"Error en PersonDetector: {e}"
            )

            self._show_frame(
                display_frame
            )

            return

        # ====================================================
        # TRACKER
        # ====================================================

        try:

            tracking_result = (
                self._person_tracker.update(
                    persons
                )
            )

        except Exception as e:

            self.get_logger().error(
                f"Error en PersonTracker: {e}"
            )

            self._show_frame(
                display_frame
            )

            return

        # ====================================================
        # VERIFICAR OPERADOR
        # ====================================================

        if not tracking_result.person_detected:

            self._blink_analyzer.reset()

            self._draw_no_operator(
                display_frame
            )

            self._show_frame(
                display_frame
            )

            return

        # ====================================================
        # OBTENER OPERADOR PRINCIPAL
        # ====================================================

        primary_person = (
            self._person_tracker.get_primary_person()
        )

        if primary_person is None:

            self._blink_analyzer.reset()

            self._draw_no_operator(
                display_frame
            )

            self._show_frame(
                display_frame
            )

            return

        # ====================================================
        # DIBUJAR OPERADOR
        # ====================================================

        self._draw_person(
            display_frame,
            primary_person
        )

        # ====================================================
        # EXTRAER ROI FACIAL
        # ====================================================

        if self._face_roi_enabled:

            face_input, roi_transform = (
                self._extract_face_roi(
                    frame,
                    primary_person
                )
            )

        else:

            face_input = frame

            roi_transform = {
                "offset_x": 0.0,
                "offset_y": 0.0,
                "scale_x": 1.0,
                "scale_y": 1.0,
            }

        if face_input is None:

            self._blink_analyzer.reset()

            self._draw_no_face(
                display_frame,
                primary_person
            )

            self._show_frame(
                display_frame
            )

            return

        # ====================================================
        # VISUALIZAR ROI
        # ====================================================

        if self._visualizar_face_roi:

            self._draw_face_roi(
                display_frame,
                roi_transform,
                face_input
            )

        # ====================================================
        # FACE LANDMARKS
        # ====================================================

        try:

            face_result = (
                self._face_landmark_detector.update(
                    face_input
                )
            )

            face_result = (
                self._restore_face_coordinates(
                    face_result,
                    roi_transform
                )
            )

        except Exception as e:

            self.get_logger().error(
                "Error en FaceLandmarkDetector: "
                f"{e}"
            )

            self._blink_analyzer.reset()

            self._draw_no_face(
                display_frame,
                primary_person
            )

            self._show_frame(
                display_frame
            )

            return

        # ====================================================
        # ROSTRO DETECTADO
        # ====================================================

        if face_result.face_detected:

            # =================================================
            # BLINK ANALYZER
            # =================================================

            try:

                blink_result = (
                    self._blink_analyzer.update(
                        face_result.left_eye_points,
                        face_result.right_eye_points,
                    )
                )

            except Exception as e:

                self.get_logger().error(
                    "Error en BlinkAnalyzer: "
                    f"{e}"
                )

                blink_result = None

            # =================================================
            # HEAD POSE ESTIMATOR
            # =================================================

            try:

                head_pose_result = (
                    self._head_pose_estimator.update(
                        face_result.head_pose_points,
                        (
                            frame.shape[1],
                            frame.shape[0]
                        )
                    )
                )

            except Exception as e:

                self.get_logger().error(
                    "Error en HeadPoseEstimator: "
                    f"{e}"
                )

                head_pose_result = None

            # =================================================
            # ATTENTION ANALYZER
            # =================================================

            attention_result = None

            if (
                head_pose_result is not None
                and head_pose_result.pose_valid
            ):

                try:

                    attention_result = (
                        self._attention_analyzer.update(
                            head_pose_result.yaw,
                            head_pose_result.pitch,
                            head_pose_result.roll,
                        )
                    )

                except Exception as e:

                    self.get_logger().error(
                        "Error en AttentionAnalyzer: "
                        f"{e}"
                    )

            # =================================================
            # INFORMACIÓN FACIAL
            # =================================================

            self._draw_face_information(
                display_frame,
                face_result,
                primary_person
            )

            # =================================================
            # INFORMACIÓN BLINK
            # =================================================

            if (
                self._visualizar_blink
                and blink_result is not None
                and blink_result.valid
            ):

                self._draw_blink_information(
                    display_frame,
                    blink_result
                )

            # =================================================
            # INFORMACIÓN HEAD POSE
            # =================================================

            if (
                self._visualizar_head_pose
                and head_pose_result is not None
                and head_pose_result.pose_valid
            ):

                self._draw_head_pose_information(
                    display_frame,
                    head_pose_result
                )

            # =================================================
            # EJES 3D DE POSE
            # =================================================

            if (
                self._visualizar_pose_axes
                and head_pose_result is not None
                and head_pose_result.pose_valid
            ):

                self._head_pose_estimator.draw_pose_axes(
                    display_frame,
                    head_pose_result
                )

            # =================================================
            # INFORMACIÓN ATTENTION
            # =================================================

            if (
                self._visualizar_attention
                and attention_result is not None
            ):

                self._draw_attention_information(
                    display_frame,
                    attention_result
                )

        else:

            self._blink_analyzer.reset()

            self._draw_no_face(
                display_frame,
                primary_person
            )

        # ====================================================
        # MOSTRAR
        # ====================================================

        self._show_frame(
            display_frame
        )

    # ========================================================
    # DETECTAR PERSONAS
    # ========================================================

    def _detect_persons(
        self,
        frame
    ):

        return self._person_detector.update(
            frame
        )

    # ========================================================
    # EXTRAER ROI FACIAL
    # ========================================================

    def _extract_face_roi(
        self,
        frame,
        person
    ):
        """
        Extrae la región superior del operador y la reescala
        para mejorar la detección facial a distancia.
        """

        frame_height, frame_width = frame.shape[:2]

        # ----------------------------------------------------
        # Bounding box del operador
        # ----------------------------------------------------

        x1 = int(person.x1)
        y1 = int(person.y1)

        x2 = int(person.x2)
        y2 = int(person.y2)

        x1 = max(
            0,
            min(x1, frame_width - 1)
        )

        y1 = max(
            0,
            min(y1, frame_height - 1)
        )

        x2 = max(
            0,
            min(x2, frame_width)
        )

        y2 = max(
            0,
            min(y2, frame_height)
        )

        if x2 <= x1 or y2 <= y1:

            return None, None

        person_width = x2 - x1
        person_height = y2 - y1

        # ----------------------------------------------------
        # Región superior
        # ----------------------------------------------------

        upper_ratio = np.clip(
            float(self._face_roi_upper_ratio),
            0.20,
            1.00
        )

        roi_y2 = (
            y1
            + int(
                person_height
                * upper_ratio
            )
        )

        # ----------------------------------------------------
        # Padding
        # ----------------------------------------------------

        padding_x = int(
            person_width
            * max(
                0.0,
                float(
                    self._face_roi_padding_x
                )
            )
        )

        padding_y = int(
            person_height
            * max(
                0.0,
                float(
                    self._face_roi_padding_y
                )
            )
        )

        roi_x1 = max(
            0,
            x1 - padding_x
        )

        roi_y1 = max(
            0,
            y1 - padding_y
        )

        roi_x2 = min(
            frame_width,
            x2 + padding_x
        )

        roi_y2 = min(
            frame_height,
            roi_y2 + padding_y
        )

        if roi_x2 <= roi_x1 or roi_y2 <= roi_y1:

            return None, None

        # ----------------------------------------------------
        # Crop
        # ----------------------------------------------------

        crop = frame[
            roi_y1:roi_y2,
            roi_x1:roi_x2
        ]

        if crop.size == 0:

            return None, None

        crop_height, crop_width = (
            crop.shape[:2]
        )

        # ----------------------------------------------------
        # Reescalado
        # ----------------------------------------------------

        target_size = max(
            int(
                self._face_roi_target_size
            ),
            128
        )

        short_side = min(
            crop_width,
            crop_height
        )

        if short_side <= 0:

            return None, None

        scale = (
            target_size
            / float(short_side)
        )

        # No reducir una ROI que ya tenga
        # suficiente resolución.
        scale = max(
            1.0,
            scale
        )

        new_width = max(
            1,
            int(
                crop_width * scale
            )
        )

        new_height = max(
            1,
            int(
                crop_height * scale
            )
        )

        resized_crop = cv2.resize(
            crop,
            (
                new_width,
                new_height
            ),
            interpolation=cv2.INTER_CUBIC
        )

        transform = {
            "offset_x": float(roi_x1),
            "offset_y": float(roi_y1),
            "scale_x": float(scale),
            "scale_y": float(scale),
            "roi_x1": int(roi_x1),
            "roi_y1": int(roi_y1),
            "roi_x2": int(roi_x2),
            "roi_y2": int(roi_y2),
        }

        return (
            resized_crop,
            transform
        )

    # ========================================================
    # RESTAURAR COORDENADAS
    # ========================================================

    def _restore_face_coordinates(
        self,
        face_result,
        transform,
    ):
        """
        Convierte todos los puntos detectados en la ROI
        a coordenadas de la imagen original.
        """

        if face_result is None:
            return None

        if not face_result.face_detected:
            return face_result

        offset_x = transform["offset_x"]
        offset_y = transform["offset_y"]

        scale_x = transform["scale_x"]
        scale_y = transform["scale_y"]

        # ----------------------------------------------------
        # Punto individual
        # ----------------------------------------------------

        def restore_point(point):

            if point is None:
                return None

            x = (
                float(point[0])
                / scale_x
                + offset_x
            )

            y = (
                float(point[1])
                / scale_y
                + offset_y
            )

            return (
                x,
                y
            )

        # ----------------------------------------------------
        # Landmarks generales
        # ----------------------------------------------------

        if face_result.landmarks is not None:

            restored_landmarks = []

            for point in face_result.landmarks:

                restored_landmarks.append(
                    restore_point(point)
                )

            face_result.landmarks = (
                restored_landmarks
            )

        # ----------------------------------------------------
        # Bounding box
        # ----------------------------------------------------

        if face_result.bbox is not None:

            x1, y1, x2, y2 = (
                face_result.bbox
            )

            face_result.bbox = (
                int(
                    x1 / scale_x
                    + offset_x
                ),
                int(
                    y1 / scale_y
                    + offset_y
                ),
                int(
                    x2 / scale_x
                    + offset_x
                ),
                int(
                    y2 / scale_y
                    + offset_y
                ),
            )

        # ----------------------------------------------------
        # Head pose
        # ----------------------------------------------------

        face_result.head_pose_points = (
            self._restore_points(
                face_result.head_pose_points,
                scale_x,
                scale_y,
                offset_x,
                offset_y,
            )
        )

        # ----------------------------------------------------
        # Ojo izquierdo
        # ----------------------------------------------------

        face_result.left_eye_points = (
            self._restore_points(
                face_result.left_eye_points,
                scale_x,
                scale_y,
                offset_x,
                offset_y,
            )
        )

        # ----------------------------------------------------
        # Ojo derecho
        # ----------------------------------------------------

        face_result.right_eye_points = (
            self._restore_points(
                face_result.right_eye_points,
                scale_x,
                scale_y,
                offset_x,
                offset_y,
            )
        )

        # ----------------------------------------------------
        # Boca
        # ----------------------------------------------------

        face_result.mouth_points = (
            self._restore_points(
                face_result.mouth_points,
                scale_x,
                scale_y,
                offset_x,
                offset_y,
            )
        )

        # ----------------------------------------------------
        # Nariz
        # ----------------------------------------------------

        if face_result.nose_point is not None:

            face_result.nose_point = (
                restore_point(
                    face_result.nose_point
                )
            )

        return face_result

    # ========================================================
    # RESTAURAR CONJUNTO DE PUNTOS
    # ========================================================

    @staticmethod
    def _restore_points(
        points,
        scale_x,
        scale_y,
        offset_x,
        offset_y,
    ):
        """
        Restaura un conjunto de puntos 2D.
        """

        if points is None:
            return None

        points = np.asarray(
            points,
            dtype=np.float32
        )

        if points.size == 0:
            return points

        restored = points.copy()

        restored[:, 0] = (
            restored[:, 0]
            / scale_x
            + offset_x
        )

        restored[:, 1] = (
            restored[:, 1]
            / scale_y
            + offset_y
        )

        return restored

    # ========================================================
    # DIBUJAR PERSONA
    # ========================================================

    def _draw_person(
        self,
        image,
        person
    ):

        x1 = int(person.x1)
        y1 = int(person.y1)

        x2 = int(person.x2)
        y2 = int(person.y2)

        cv2.rectangle(
            image,
            (x1, y1),
            (x2, y2),
            (255, 0, 0),
            2
        )

        label = (
            f"Operator "
            f"ID:{person.track_id} "
            f"{person.confidence:.2f}"
        )

        text_y = max(
            20,
            y1 - 10
        )

        cv2.putText(
            image,
            label,
            (x1, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 0, 0),
            2
        )

    # ========================================================
    # DIBUJAR ROI
    # ========================================================

    def _draw_face_roi(
        self,
        image,
        transform,
        face_input,
    ):
        """
        Dibuja sobre la imagen original la ROI utilizada
        para la detección facial.
        """

        if transform is None:
            return

        x1 = transform.get(
            "roi_x1",
            0
        )

        y1 = transform.get(
            "roi_y1",
            0
        )

        x2 = transform.get(
            "roi_x2",
            0
        )

        y2 = transform.get(
            "roi_y2",
            0
        )

        cv2.rectangle(
            image,
            (x1, y1),
            (x2, y2),
            (255, 0, 255),
            2
        )

        cv2.putText(
            image,
            "Face ROI",
            (
                x1,
                max(
                    20,
                    y1 - 8
                )
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 0, 255),
            2
        )

    # ========================================================
    # DIBUJAR INFORMACIÓN FACIAL
    # ========================================================

    def _draw_face_information(
        self,
        image,
        face_result,
        primary_person
    ):

        if face_result.bbox is not None:

            x1, y1, x2, y2 = (
                face_result.bbox
            )

            cv2.rectangle(
                image,
                (x1, y1),
                (x2, y2),
                (0, 255, 0),
                2
            )

        if self._points_face:

            image_with_landmarks = (
                self._face_landmark_detector
                .draw_landmarks(
                    image,
                    face_result
                )
            )

            image[:] = (
                image_with_landmarks
            )

        if face_result.bbox is not None:

            x1, y1, _, _ = (
                face_result.bbox
            )

            cv2.putText(
                image,
                "Face detected",
                (
                    x1,
                    max(
                        20,
                        y1 - 10
                    )
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

    # ========================================================
    # DIBUJAR INFORMACIÓN DE PARPADEO
    # ========================================================

    def _draw_blink_information(
        self,
        image,
        blink_result,
    ):

        x = 20
        y = 100
        line_spacing = 30

        cv2.putText(
            image,
            (
                f"Left EAR: "
                f"{blink_result.left_ear:.3f}"
            ),
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )

        cv2.putText(
            image,
            (
                f"Right EAR: "
                f"{blink_result.right_ear:.3f}"
            ),
            (
                x,
                y + line_spacing
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )

        cv2.putText(
            image,
            (
                f"Eye openness: "
                f"{blink_result.eye_openness:.2f}"
            ),
            (
                x,
                y + 2 * line_spacing
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )

        cv2.putText(
            image,
            (
                f"Blink frequency: "
                f"{blink_result.blink_frequency:.1f} "
                f"blinks/min"
            ),
            (
                x,
                y + 3 * line_spacing
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )

        cv2.putText(
            image,
            (
                f"Total blinks: "
                f"{blink_result.total_blinks}"
            ),
            (
                x,
                y + 4 * line_spacing
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )

    # ========================================================
    # DIBUJAR INFORMACIÓN HEAD POSE
    # ========================================================

    def _draw_head_pose_information(
        self,
        image,
        head_pose_result,
    ):

        x = 20
        y = 250
        line_spacing = 30

        cv2.putText(
            image,
            (
                f"Yaw: "
                f"{head_pose_result.yaw:.1f} deg"
            ),
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 0),
            2
        )

        cv2.putText(
            image,
            (
                f"Pitch: "
                f"{head_pose_result.pitch:.1f} deg"
            ),
            (
                x,
                y + line_spacing
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 0),
            2
        )

        cv2.putText(
            image,
            (
                f"Roll: "
                f"{head_pose_result.roll:.1f} deg"
            ),
            (
                x,
                y + 2 * line_spacing
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 0),
            2
        )

    # ========================================================
    # DIBUJAR INFORMACIÓN ATTENTION
    # ========================================================

    def _draw_attention_information(
        self,
        image,
        attention_result,
    ):

        x = 20
        y = 350
        line_spacing = 30

        cv2.putText(
            image,
            (
                f"Attention: "
                f"{attention_result.attention_target}"
            ),
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 0),
            2
        )

        cv2.putText(
            image,
            (
                f"Attention score: "
                f"{attention_result.attention_score:.2f}"
            ),
            (
                x,
                y + line_spacing
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 0),
            2
        )

        cv2.putText(
            image,
            (
                f"Task attention: "
                f"{attention_result.task_attention}"
            ),
            (
                x,
                y + 2 * line_spacing
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 0),
            2
        )

        cv2.putText(
            image,
            (
                f"HMI attention: "
                f"{attention_result.hmi_attention}"
            ),
            (
                x,
                y + 3 * line_spacing
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 0),
            2
        )

        cv2.putText(
            image,
            (
                f"Distracted: "
                f"{attention_result.distracted}"
            ),
            (
                x,
                y + 4 * line_spacing
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 0),
            2
        )

    # ========================================================
    # DIBUJAR CUANDO NO HAY OPERADOR
    # ========================================================

    def _draw_no_operator(
        self,
        image
    ):

        cv2.putText(
            image,
            "No operator detected",
            (30, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 0, 255),
            2
        )

    # ========================================================
    # DIBUJAR CUANDO NO HAY ROSTRO
    # ========================================================

    def _draw_no_face(
        self,
        image,
        primary_person
    ):

        x1 = int(
            primary_person.x1
        )

        y1 = int(
            primary_person.y1
        )

        cv2.putText(
            image,
            "Face not detected",
            (
                x1,
                max(
                    20,
                    y1 - 10
                )
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 255),
            2
        )

    # ========================================================
    # MOSTRAR FRAME
    # ========================================================

    def _show_frame(
        self,
        image
    ):

        if not self._visualizar:
            return

        cv2.imshow(
            "Operator RGB Camera",
            image
        )

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q") or key == 27:

            self.get_logger().info(
                "Cierre solicitado por usuario."
            )

            rclpy.shutdown()

    # ========================================================
    # DESTRUCTOR
    # ========================================================

    def destroy_node(self):

        self.get_logger().info(
            "Cerrando Operator Fatigue Node..."
        )

        if self._camera is not None:

            self._camera.release()

            self._camera = None

        try:

            cv2.destroyAllWindows()

        except Exception:

            pass

        try:

            if (
                hasattr(
                    self,
                    "_face_landmark_detector"
                )
                and
                self._face_landmark_detector is not None
            ):

                self._face_landmark_detector.close()

        except Exception as e:

            self.get_logger().warning(
                "Error cerrando FaceLandmarkDetector: "
                f"{e}"
            )

        super().destroy_node()


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = None

    try:

        node = OperatorFatigueNode()

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    except Exception as e:

        if node is not None:

            node.get_logger().error(
                "Error en OperatorFatigueNode: "
                f"{e}"
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
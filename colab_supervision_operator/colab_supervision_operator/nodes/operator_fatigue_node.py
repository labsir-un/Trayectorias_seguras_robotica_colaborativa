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


# ============================================================
# OPERATOR FATIGUE NODE
# ============================================================

class OperatorFatigueNode(Node):

    # ========================================================
    # CONSTRUCTOR
    # ========================================================

    def __init__(self):

        super().__init__(
            "operator_fatigue_node"
        )

        # ====================================================
        # PACKAGE
        # ====================================================

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
        # ROI facial
        # ----------------------------------------------------

        self.declare_parameter(
            "visualizar_face_roi",
            False
        )

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
        # Blink
        # ----------------------------------------------------

        self.declare_parameter(
            "visualizar_blink",
            False
        )

        # ----------------------------------------------------
        # Head Pose
        # ----------------------------------------------------

        self.declare_parameter(
            "visualizar_head_pose",
            False
        )

        self.declare_parameter(
            "visualizar_pose_axes",
            False
        )

        # ----------------------------------------------------
        # Attention
        # ----------------------------------------------------

        self.declare_parameter(
            "visualizar_attention",
            False
        )

        # ----------------------------------------------------
        # Expression
        # ----------------------------------------------------

        self.declare_parameter(
            "visualizar_expression",
            False
        )

        # ====================================================
        # YOLO
        # ====================================================

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

        # ====================================================
        # FACE LANDMARKER
        # ====================================================

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
        # EXPRESSION ANALYZER
        # ====================================================

        self.declare_parameter(
            "expression_threshold",
            0.30
        )

        self.declare_parameter(
            "expression_sleepy_threshold",
            0.35
        )

        self.declare_parameter(
            "expression_tired_threshold",
            0.35
        )

        self.declare_parameter(
            "expression_fatigued_threshold",
            0.45
        )

        self.declare_parameter(
            "expression_yawn_threshold",
            0.38
        )

        self.declare_parameter(
            "expression_smoothing_alpha",
            0.35
        )

        self.declare_parameter(
            "expression_sleepy_frames",
            6
        )

        self.declare_parameter(
            "expression_tired_frames",
            10
        )

        self.declare_parameter(
            "expression_fatigued_frames",
            20
        )

        self.declare_parameter(
            "expression_yawn_frames",
            8
        )

        # ====================================================
        # OBTENER PARÁMETROS
        # ====================================================

        # ----------------------------------------------------
        # Cámara
        # ----------------------------------------------------

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

        # ----------------------------------------------------
        # Visualización
        # ----------------------------------------------------

        self._visualizar = self.get_parameter(
            "visualizar"
        ).value

        self._points_face = self.get_parameter(
            "points_face"
        ).value

        self._visualizar_face_roi = self.get_parameter(
            "visualizar_face_roi"
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

        self._visualizar_expression = self.get_parameter(
            "visualizar_expression"
        ).value

        # ----------------------------------------------------
        # ROI facial
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
        # Face Landmarker
        # ----------------------------------------------------

        face_model = self.get_parameter(
            "face_landmarker_model"
        ).value

        face_detection_confidence = self.get_parameter(
            "face_detection_confidence"
        ).value

        face_tracking_confidence = self.get_parameter(
            "face_tracking_confidence"
        ).value

        # ----------------------------------------------------
        # Expression Analyzer
        # ----------------------------------------------------

        expression_threshold = self.get_parameter(
            "expression_threshold"
        ).value

        expression_sleepy_threshold = self.get_parameter(
            "expression_sleepy_threshold"
        ).value

        expression_tired_threshold = self.get_parameter(
            "expression_tired_threshold"
        ).value

        expression_fatigued_threshold = self.get_parameter(
            "expression_fatigued_threshold"
        ).value

        expression_yawn_threshold = self.get_parameter(
            "expression_yawn_threshold"
        ).value

        expression_smoothing_alpha = self.get_parameter(
            "expression_smoothing_alpha"
        ).value

        expression_sleepy_frames = self.get_parameter(
            "expression_sleepy_frames"
        ).value

        expression_tired_frames = self.get_parameter(
            "expression_tired_frames"
        ).value

        expression_fatigued_frames = self.get_parameter(
            "expression_fatigued_frames"
        ).value

        expression_yawn_frames = self.get_parameter(
            "expression_yawn_frames"
        ).value

        # ====================================================
        # INFORMACIÓN DE INICIO
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
            f"Resolución: "
            f"{self._camera_width}x"
            f"{self._camera_height}"
        )

        self.get_logger().info(
            f"FPS: {self._camera_fps}"
        )

        self.get_logger().info(
            f"YOLO: {yolo_model}"
        )

        self.get_logger().info(
            f"Face Landmarker: {face_model}"
        )

        self.get_logger().info(
            f"Expression threshold: "
            f"{expression_threshold:.2f}"
        )

        self.get_logger().info(
            f"Sleepy threshold: "
            f"{expression_sleepy_threshold:.2f}"
        )

        self.get_logger().info(
            f"Tired threshold: "
            f"{expression_tired_threshold:.2f}"
        )

        self.get_logger().info(
            f"Fatigued threshold: "
            f"{expression_fatigued_threshold:.2f}"
        )

        self.get_logger().info(
            f"Yawn threshold: "
            f"{expression_yawn_threshold:.2f}"
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
        # PERSON DETECTOR
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
        # PERSON TRACKER
        # ====================================================

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

        self._head_pose_estimator = (
            HeadPoseEstimator()
        )

        self.get_logger().info(
            "HeadPoseEstimator inicializado correctamente."
        )

        # ====================================================
        # ATTENTION ANALYZER
        # ====================================================

        self._attention_analyzer = (
            AttentionAnalyzer()
        )

        self.get_logger().info(
            "AttentionAnalyzer inicializado correctamente."
        )

        # ====================================================
        # EXPRESSION ANALYZER
        # ====================================================

        self._expression_analyzer = (
            ExpressionAnalyzer(

                expression_threshold=(
                    expression_threshold
                ),

                sleepy_threshold=(
                    expression_sleepy_threshold
                ),

                tired_threshold=(
                    expression_tired_threshold
                ),

                fatigued_threshold=(
                    expression_fatigued_threshold
                ),

                yawn_threshold=(
                    expression_yawn_threshold
                ),

                smoothing_alpha=(
                    expression_smoothing_alpha
                ),

                sleepy_frames=(
                    expression_sleepy_frames
                ),

                tired_frames=(
                    expression_tired_frames
                ),

                fatigued_frames=(
                    expression_fatigued_frames
                ),

                yawn_frames=(
                    expression_yawn_frames
                ),
            )
        )

        self.get_logger().info(
            "ExpressionAnalyzer "
            "inicializado correctamente."
        )

        # ====================================================
        # TIMER
        # ====================================================

        timer_period = (
            1.0 /
            float(self._camera_fps)
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

        # ----------------------------------------------------
        # MJPG
        # ----------------------------------------------------

        self._camera.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(
                *"MJPG"
            )
        )

        # ----------------------------------------------------
        # Resolución
        # ----------------------------------------------------

        self._camera.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            self._camera_width
        )

        self._camera.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            self._camera_height
        )

        # ----------------------------------------------------
        # FPS
        # ----------------------------------------------------

        self._camera.set(
            cv2.CAP_PROP_FPS,
            self._camera_fps
        )

        # ----------------------------------------------------
        # Configuración real
        # ----------------------------------------------------

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
            f"{actual_width}x"
            f"{actual_height}"
        )

        self.get_logger().info(
            f"  FPS: {actual_fps:.2f}"
        )

        self.get_logger().info(
            f"  Formato: "
            f"{fourcc_string}"
        )

        # ----------------------------------------------------
        # Primer frame
        # ----------------------------------------------------

        ret, frame = (
            self._camera.read()
        )

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

        # ----------------------------------------------------
        # Capturar frame
        # ----------------------------------------------------

        ret, frame = (
            self._camera.read()
        )

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
                "Error en PersonDetector: "
                f"{e}"
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
                "Error en PersonTracker: "
                f"{e}"
            )

            self._show_frame(
                display_frame
            )

            return

        # ====================================================
        # NO HAY OPERADOR
        # ====================================================

        if not tracking_result.person_detected:

            self._draw_no_operator(
                display_frame
            )

            self._show_frame(
                display_frame
            )

            return

        # ====================================================
        # OPERADOR PRINCIPAL
        # ====================================================

        primary_person = (
            self._person_tracker.get_primary_person()
        )

        if primary_person is None:

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

        face_input = frame

        roi_transform = None

        if self._face_roi_enabled:

            roi_data = self._extract_face_roi(
                frame,
                primary_person
            )

            if roi_data is not None:

                face_input = roi_data["image"]

                roi_transform = (
                    roi_data["transform"]
                )

                if self._visualizar_face_roi:

                    self._draw_face_roi(
                        display_frame,
                        roi_data
                    )

        # ====================================================
        # FACE LANDMARK DETECTOR
        # ====================================================

        try:

            face_result = (
                self._face_landmark_detector.update(
                    face_input
                )
            )

        except Exception as e:

            self.get_logger().error(
                "Error en FaceLandmarkDetector: "
                f"{e}"
            )

            self._show_frame(
                display_frame
            )

            return

        # ====================================================
        # RESTAURAR COORDENADAS
        # ====================================================

        if (
            face_result.face_detected
            and
            roi_transform is not None
        ):

            self._restore_face_coordinates(
                face_result,
                roi_transform
            )

        # ====================================================
        # NO HAY ROSTRO
        # ====================================================

        if not face_result.face_detected:

            self._draw_no_face(
                display_frame,
                primary_person
            )

            self._show_frame(
                display_frame
            )

            return

        # ====================================================
        # DIBUJAR INFORMACIÓN FACIAL
        # ====================================================

        self._draw_face_information(
            display_frame,
            face_result
        )

        # ====================================================
        # BLINK ANALYZER
        # ====================================================

        blink_result = None

        if (
            face_result.left_eye_points is not None
            and
            face_result.right_eye_points is not None
        ):

            try:

                timestamp = (
                    self.get_clock()
                    .now()
                    .nanoseconds
                    /
                    1e9
                )

                # El BlinkAnalyzer utiliza el valor
                # de apertura ocular calculado a partir
                # de los landmarks de los ojos.
                #
                # En esta integración calculamos EAR
                # directamente para mantener compatibilidad
                # con el analyzer validado anteriormente.

                eye_openness = (
                    self._calculate_eye_openness(
                        face_result.left_eye_points,
                        face_result.right_eye_points
                    )
                )

                blink_result = (
                    self._blink_analyzer.update(
                        eye_openness,
                        timestamp
                    )
                )

            except Exception as e:

                self.get_logger().error(
                    "Error en BlinkAnalyzer: "
                    f"{e}"
                )

        # ====================================================
        # HEAD POSE
        # ====================================================

        head_pose_result = None

        if (
            face_result.head_pose_points
            is not None
        ):

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

        # ====================================================
        # ATTENTION
        # ====================================================

        attention_result = None

        if (
            head_pose_result is not None
            and
            head_pose_result.pose_valid
        ):

            try:

                attention_result = (
                    self._attention_analyzer.update(
                        head_pose_result.yaw,
                        head_pose_result.pitch,
                        head_pose_result.roll
                    )
                )

            except Exception as e:

                self.get_logger().error(
                    "Error en AttentionAnalyzer: "
                    f"{e}"
                )

        # ====================================================
        # EXPRESSION ANALYZER
        # ====================================================

        expression_result = None

        if (
            face_result.face_blendshapes is not None
            and
            face_result.face_blendshapes
        ):

            try:

                expression_result = (
                    self._expression_analyzer.update(
                        face_result.face_blendshapes
                    )
                )

            except Exception as e:

                self.get_logger().error(
                    "Error en ExpressionAnalyzer: "
                    f"{e}"
                )

        # ====================================================
        # VISUALIZACIÓN BLINK
        # ====================================================

        if (
            self._visualizar_blink
            and
            blink_result is not None
        ):

            display_frame = (
                self._draw_blink_information(
                    display_frame,
                    blink_result
                )
            )

        # ====================================================
        # VISUALIZACIÓN HEAD POSE
        # ====================================================

        if (
            self._visualizar_head_pose
            and
            head_pose_result is not None
        ):

            display_frame = (
                self._draw_head_pose_information(
                    display_frame,
                    head_pose_result
                )
            )

        # ====================================================
        # EJES HEAD POSE
        # ====================================================

        if (
            self._visualizar_pose_axes
            and
            head_pose_result is not None
            and
            head_pose_result.pose_valid
        ):

            try:

                display_frame = (
                    self._head_pose_estimator.draw_pose_axes(
                        display_frame,
                        head_pose_result
                    )
                )

            except AttributeError:

                pass

            except Exception as e:

                self.get_logger().warning(
                    "Error dibujando ejes de pose: "
                    f"{e}"
                )

        # ====================================================
        # VISUALIZACIÓN ATTENTION
        # ====================================================

        if (
            self._visualizar_attention
            and
            attention_result is not None
        ):

            display_frame = (
                self._draw_attention_information(
                    display_frame,
                    attention_result
                )
            )

        # ====================================================
        # VISUALIZACIÓN EXPRESSION
        # ====================================================

        if (
            self._visualizar_expression
            and
            expression_result is not None
        ):

            display_frame = (
                self._draw_expression_information(
                    display_frame,
                    expression_result
                )
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

        height, width = (
            frame.shape[:2]
        )

        # ----------------------------------------------------
        # Bounding box del operador
        # ----------------------------------------------------

        x1 = int(
            max(
                0,
                person.x1
            )
        )

        y1 = int(
            max(
                0,
                person.y1
            )
        )

        x2 = int(
            min(
                width - 1,
                person.x2
            )
        )

        y2 = int(
            min(
                height - 1,
                person.y2
            )
        )

        if (
            x2 <= x1
            or
            y2 <= y1
        ):

            return None

        person_width = (
            x2 - x1
        )

        person_height = (
            y2 - y1
        )

        # ----------------------------------------------------
        # Región superior
        # ----------------------------------------------------

        roi_height = int(
            person_height
            *
            float(
                self._face_roi_upper_ratio
            )
        )

        roi_height = max(
            1,
            roi_height
        )

        face_y1 = y1

        face_y2 = min(
            y2,
            y1 + roi_height
        )

        # ----------------------------------------------------
        # Padding
        # ----------------------------------------------------

        pad_x = int(
            person_width
            *
            float(
                self._face_roi_padding_x
            )
        )

        pad_y = int(
            person_height
            *
            float(
                self._face_roi_padding_y
            )
        )

        roi_x1 = max(
            0,
            x1 - pad_x
        )

        roi_y1 = max(
            0,
            face_y1 - pad_y
        )

        roi_x2 = min(
            width,
            x2 + pad_x
        )

        roi_y2 = min(
            height,
            face_y2 + pad_y
        )

        if (
            roi_x2 <= roi_x1
            or
            roi_y2 <= roi_y1
        ):

            return None

        # ----------------------------------------------------
        # Recortar
        # ----------------------------------------------------

        roi = frame[
            roi_y1:roi_y2,
            roi_x1:roi_x2
        ]

        if roi.size == 0:

            return None

        roi_height_pixels, roi_width_pixels = (
            roi.shape[:2]
        )

        # ----------------------------------------------------
        # Escalado
        # ----------------------------------------------------

        target_size = int(
            self._face_roi_target_size
        )

        short_side = min(
            roi_width_pixels,
            roi_height_pixels
        )

        scale = 1.0

        if short_side < target_size:

            scale = (
                float(target_size)
                /
                float(short_side)
            )

        new_width = max(
            1,
            int(
                round(
                    roi_width_pixels
                    *
                    scale
                )
            )
        )

        new_height = max(
            1,
            int(
                round(
                    roi_height_pixels
                    *
                    scale
                )
            )
        )

        if (
            new_width != roi_width_pixels
            or
            new_height != roi_height_pixels
        ):

            roi_resized = cv2.resize(
                roi,
                (
                    new_width,
                    new_height
                ),
                interpolation=cv2.INTER_CUBIC
            )

        else:

            roi_resized = roi

        # ----------------------------------------------------
        # Transformación
        # ----------------------------------------------------

        scale_x = (
            float(new_width)
            /
            float(roi_width_pixels)
        )

        scale_y = (
            float(new_height)
            /
            float(roi_height_pixels)
        )

        transform = {
            "offset_x": float(roi_x1),
            "offset_y": float(roi_y1),
            "scale_x": scale_x,
            "scale_y": scale_y,
        }

        return {
            "image": roi_resized,
            "transform": transform,
            "bbox": (
                roi_x1,
                roi_y1,
                roi_x2,
                roi_y2
            ),
        }

    # ========================================================
    # RESTAURAR COORDENADAS
    # ========================================================

    def _restore_face_coordinates(
        self,
        face_result,
        transform
    ):

        offset_x = transform[
            "offset_x"
        ]

        offset_y = transform[
            "offset_y"
        ]

        scale_x = transform[
            "scale_x"
        ]

        scale_y = transform[
            "scale_y"
        ]

        # ----------------------------------------------------
        # Landmarks
        # ----------------------------------------------------

        if (
            face_result.landmarks
            is not None
        ):

            face_result.landmarks = (
                self._restore_points(
                    face_result.landmarks,
                    offset_x,
                    offset_y,
                    scale_x,
                    scale_y
                )
            )

        # ----------------------------------------------------
        # BBox
        # ----------------------------------------------------

        if face_result.bbox is not None:

            x1, y1, x2, y2 = (
                face_result.bbox
            )

            face_result.bbox = (

                int(
                    round(
                        float(x1)
                        /
                        scale_x
                        +
                        offset_x
                    )
                ),

                int(
                    round(
                        float(y1)
                        /
                        scale_y
                        +
                        offset_y
                    )
                ),

                int(
                    round(
                        float(x2)
                        /
                        scale_x
                        +
                        offset_x
                    )
                ),

                int(
                    round(
                        float(y2)
                        /
                        scale_y
                        +
                        offset_y
                    )
                ),
            )

        # ----------------------------------------------------
        # Head Pose
        # ----------------------------------------------------

        if (
            face_result.head_pose_points
            is not None
        ):

            face_result.head_pose_points = (
                self._restore_points(
                    face_result.head_pose_points,
                    offset_x,
                    offset_y,
                    scale_x,
                    scale_y
                )
            )

        # ----------------------------------------------------
        # Ojo izquierdo
        # ----------------------------------------------------

        if (
            face_result.left_eye_points
            is not None
        ):

            face_result.left_eye_points = (
                self._restore_points(
                    face_result.left_eye_points,
                    offset_x,
                    offset_y,
                    scale_x,
                    scale_y
                )
            )

        # ----------------------------------------------------
        # Ojo derecho
        # ----------------------------------------------------

        if (
            face_result.right_eye_points
            is not None
        ):

            face_result.right_eye_points = (
                self._restore_points(
                    face_result.right_eye_points,
                    offset_x,
                    offset_y,
                    scale_x,
                    scale_y
                )
            )

        # ----------------------------------------------------
        # Boca
        # ----------------------------------------------------

        if (
            face_result.mouth_points
            is not None
        ):

            face_result.mouth_points = (
                self._restore_points(
                    face_result.mouth_points,
                    offset_x,
                    offset_y,
                    scale_x,
                    scale_y
                )
            )

        # ----------------------------------------------------
        # Nariz
        # ----------------------------------------------------

        if (
            face_result.nose_point
            is not None
        ):

            x = face_result.nose_point[0]

            y = face_result.nose_point[1]

            face_result.nose_point = (
                int(
                    round(
                        float(x)
                        /
                        scale_x
                        +
                        offset_x
                    )
                ),

                int(
                    round(
                        float(y)
                        /
                        scale_y
                        +
                        offset_y
                    )
                ),
            )

    # ========================================================
    # RESTAURAR PUNTOS
    # ========================================================

    @staticmethod
    def _restore_points(
        points,
        offset_x,
        offset_y,
        scale_x,
        scale_y
    ):

        points = np.asarray(
            points,
            dtype=np.float32
        )

        restored = (
            points.copy()
        )

        restored[:, 0] = (
            points[:, 0]
            /
            float(scale_x)
            +
            float(offset_x)
        )

        restored[:, 1] = (
            points[:, 1]
            /
            float(scale_y)
            +
            float(offset_y)
        )

        return restored

    # ========================================================
    # CALCULAR EYE OPENNESS
    # ========================================================

    @staticmethod
    def _calculate_eye_openness(
        left_eye_points,
        right_eye_points
    ):

        """
        Calcula una medida normalizada de apertura ocular
        utilizando los seis puntos de cada ojo.

        La estructura esperada es:

            [p1, p2, p3, p4, p5, p6]

        siguiendo los landmarks utilizados por BlinkAnalyzer.
        """

        def calculate_ear(points):

            points = np.asarray(
                points,
                dtype=np.float32
            )

            if points.shape[0] != 6:

                raise ValueError(
                    "Eye landmarks must contain "
                    "exactly 6 points"
                )

            p1, p2, p3, p4, p5, p6 = points

            vertical_1 = np.linalg.norm(
                p2 - p6
            )

            vertical_2 = np.linalg.norm(
                p3 - p5
            )

            horizontal = np.linalg.norm(
                p1 - p4
            )

            if horizontal <= 1e-6:

                return 0.0

            return (
                vertical_1
                +
                vertical_2
            ) / (
                2.0 * horizontal
            )

        left_ear = calculate_ear(
            left_eye_points
        )

        right_ear = calculate_ear(
            right_eye_points
        )

        mean_ear = (
            left_ear
            +
            right_ear
        ) / 2.0

        # ----------------------------------------------------
        # Normalización aproximada
        # ----------------------------------------------------

        minimum = 0.10

        maximum = 0.35

        openness = (
            mean_ear - minimum
        ) / (
            maximum - minimum
        )

        return max(
            0.0,
            min(
                1.0,
                openness
            )
        )

    # ========================================================
    # DIBUJAR ROI
    # ========================================================

    def _draw_face_roi(
        self,
        image,
        roi_data
    ):

        x1, y1, x2, y2 = (
            roi_data["bbox"]
        )

        cv2.rectangle(
            image,
            (
                int(x1),
                int(y1)
            ),
            (
                int(x2),
                int(y2)
            ),
            (255, 0, 255),
            2
        )

        cv2.putText(
            image,
            "Face ROI",
            (
                int(x1),
                max(
                    20,
                    int(y1) - 8
                )
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 0, 255),
            2
        )

    # ========================================================
    # DIBUJAR PERSONA
    # ========================================================

    def _draw_person(
        self,
        image,
        person
    ):

        x1 = int(
            person.x1
        )

        y1 = int(
            person.y1
        )

        x2 = int(
            person.x2
        )

        y2 = int(
            person.y2
        )

        cv2.rectangle(
            image,
            (
                x1,
                y1
            ),
            (
                x2,
                y2
            ),
            (255, 0, 0),
            2
        )

        label = (
            f"Operator "
            f"ID:{person.track_id} "
            f"{person.confidence:.2f}"
        )

        cv2.putText(
            image,
            label,
            (
                x1,
                max(
                    20,
                    y1 - 10
                )
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 0, 0),
            2
        )

    # ========================================================
    # DIBUJAR INFORMACIÓN FACIAL
    # ========================================================

    def _draw_face_information(
        self,
        image,
        face_result
    ):

        # ----------------------------------------------------
        # Bounding box
        # ----------------------------------------------------

        if face_result.bbox is not None:

            x1, y1, x2, y2 = (
                face_result.bbox
            )

            cv2.rectangle(
                image,
                (
                    int(x1),
                    int(y1)
                ),
                (
                    int(x2),
                    int(y2)
                ),
                (0, 255, 0),
                2
            )

            cv2.putText(
                image,
                "Face detected",
                (
                    int(x1),
                    max(
                        20,
                        int(y1) - 10
                    )
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

        # ----------------------------------------------------
        # Landmarks
        # ----------------------------------------------------

        if self._points_face:

            try:

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

            except Exception as e:

                self.get_logger().warning(
                    "Error dibujando landmarks: "
                    f"{e}"
                )

    # ========================================================
    # DIBUJAR EXPRESSION
    # ========================================================

    def _draw_expression_information(
        self,
        image,
        expression_result
    ):

        x = 20
        y = 40
        line_height = 25

        lines = [

            (
                f"Expression: "
                f"{expression_result.expression}"
            ),

            (
                f"Confidence: "
                f"{expression_result.confidence:.2f}"
            ),

            (
                f"Smile: "
                f"{expression_result.smile_score:.2f}"
            ),

            (
                f"Frown: "
                f"{expression_result.frown_score:.2f}"
            ),

            (
                f"Surprise: "
                f"{expression_result.surprise_score:.2f}"
            ),

            (
                f"Neutral: "
                f"{expression_result.neutral_score:.2f}"
            ),

            (
                f"Sleepy: "
                f"{expression_result.sleepy_score:.2f}"
            ),

            (
                f"Tired: "
                f"{expression_result.tired_score:.2f}"
            ),

            (
                f"Fatigued: "
                f"{expression_result.fatigued_score:.2f}"
            ),

            (
                f"Yawn: "
                f"{expression_result.yawn_score:.2f}"
            ),
        ]

        for index, line in enumerate(
            lines
        ):

            cv2.putText(
                image,
                line,
                (
                    x,
                    y +
                    index *
                    line_height
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2
            )

        return image

    # ========================================================
    # DIBUJAR BLINK
    # ========================================================

    def _draw_blink_information(
        self,
        image,
        blink_result
    ):

        x = 700
        y = 40
        line_height = 25

        lines = [

            (
                f"Left EAR: "
                f"{blink_result.left_ear:.2f}"
            ),

            (
                f"Right EAR: "
                f"{blink_result.right_ear:.2f}"
            ),

            (
                f"Eye openness: "
                f"{blink_result.eye_openness:.2f}"
            ),

            (
                f"Blink frequency: "
                f"{blink_result.blink_frequency:.2f}"
            ),

            (
                f"Total blinks: "
                f"{blink_result.total_blinks}"
            ),
        ]

        for index, line in enumerate(
            lines
        ):

            cv2.putText(
                image,
                line,
                (
                    x,
                    y +
                    index *
                    line_height
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 0),
                2
            )

        return image

    # ========================================================
    # DIBUJAR HEAD POSE
    # ========================================================

    def _draw_head_pose_information(
        self,
        image,
        head_pose_result
    ):

        x = 20
        y = 320
        line_height = 25

        lines = [

            (
                f"Yaw: "
                f"{head_pose_result.yaw:.1f}"
            ),

            (
                f"Pitch: "
                f"{head_pose_result.pitch:.1f}"
            ),

            (
                f"Roll: "
                f"{head_pose_result.roll:.1f}"
            ),

            (
                f"Valid: "
                f"{head_pose_result.pose_valid}"
            ),
        ]

        for index, line in enumerate(
            lines
        ):

            cv2.putText(
                image,
                line,
                (
                    x,
                    y +
                    index *
                    line_height
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2
            )

        return image

    # ========================================================
    # DIBUJAR ATTENTION
    # ========================================================

    def _draw_attention_information(
        self,
        image,
        attention_result
    ):

        x = 20
        y = 440
        line_height = 25

        lines = [

            (
                f"Attention: "
                f"{attention_result.attention_target}"
            ),

            (
                f"Score: "
                f"{attention_result.attention_score:.2f}"
            ),

            (
                f"Task: "
                f"{attention_result.task_attention}"
            ),

            (
                f"HMI: "
                f"{attention_result.hmi_attention}"
            ),

            (
                f"Distracted: "
                f"{attention_result.distracted}"
            ),
        ]

        for index, line in enumerate(
            lines
        ):

            cv2.putText(
                image,
                line,
                (
                    x,
                    y +
                    index *
                    line_height
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2
            )

        return image

    # ========================================================
    # NO HAY OPERADOR
    # ========================================================

    def _draw_no_operator(
        self,
        image
    ):

        cv2.putText(
            image,
            "No operator detected",
            (
                30,
                40
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 0, 255),
            2
        )

    # ========================================================
    # NO HAY ROSTRO
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

        key = (
            cv2.waitKey(1)
            &
            0xFF
        )

        if (
            key == ord("q")
            or
            key == 27
        ):

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

        # ----------------------------------------------------
        # Cámara
        # ----------------------------------------------------

        if self._camera is not None:

            self._camera.release()

            self._camera = None

        # ----------------------------------------------------
        # Ventanas
        # ----------------------------------------------------

        try:

            cv2.destroyAllWindows()

        except Exception:

            pass

        # ----------------------------------------------------
        # MediaPipe
        # ----------------------------------------------------

        try:

            if (
                hasattr(
                    self,
                    "_face_landmark_detector"
                )
                and
                self._face_landmark_detector
                is not None
            ):

                self._face_landmark_detector.close()

        except Exception as e:

            self.get_logger().warning(
                "Error cerrando "
                "FaceLandmarkDetector: "
                f"{e}"
            )

        # ----------------------------------------------------
        # ROS
        # ----------------------------------------------------

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

        else:

            print(
                "Error inicializando "
                f"OperatorFatigueNode: {e}"
            )

    finally:

        if node is not None:

            node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":

    main()
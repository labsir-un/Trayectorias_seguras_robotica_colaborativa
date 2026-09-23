#!/usr/bin/env python3

"""
Nodo ROS 2 para percepción del operador.

Este nodo:
- Captura imágenes desde una cámara RGB.
- Permite visualizar la imagen mediante un parámetro.
- Ejecuta los módulos de percepción del operador.
- Analiza fatiga, parpadeo, expresión, postura de cabeza y atención.
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

class OperatorFatigueNode(Node):
    """
    Nodo ROS 2 para percepción del operador mediante cámara RGB.
    """

    def __init__(self):

        super().__init__("operator_fatigue_node")

        # ====================================================
        # PARÁMETROS
        # ====================================================

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

        # ----------------------------------------------------
        # Mostrar landmarks faciales
        # ----------------------------------------------------

        self.declare_parameter(
            "points_face",
            False
        )

        # ------------------------------------------------------------
        # YOLO
        # ------------------------------------------------------------

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

        # ====================================================
        # OBTENER PARÁMETROS de Modelo
        # ====================================================
        
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

        self._person_detector = PersonDetector(
            model_path=yolo_model,
            confidence_threshold=yolo_confidence,
            image_size=yolo_imgsz,
            device=yolo_device,
        )

        # ====================================================
        # INFORMACIÓN
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
            f"Resolución solicitada: "
            f"{self._camera_width}x{self._camera_height}"
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
            "=============================================="
        )

        # ====================================================
        # CÁMARA
        # ====================================================

        self._camera = self._initialize_camera()

        if self._camera is None:
            raise RuntimeError(
                "No fue posible inicializar la cámara RGB."
            )

        # ====================================================
        # PERCEPCIÓN
        # ====================================================

        self._person_detector = PersonDetector(
            model_path="yolo26n.pt",
            confidence_threshold=0.5,
            image_size=640,
        )

        self._person_tracker = PersonTracker()

        self._face_landmark_detector = FaceLandmarkDetector(
            min_detection_confidence=0.5,
            max_num_faces=1,
        )

        # ====================================================
        # TIMER
        # ====================================================

        timer_period = 1.0 / self._camera_fps

        self._timer = self.create_timer(
            timer_period,
            self._process_frame
        )

        self.get_logger().info(
            "Operator Fatigue Node iniciado correctamente."
        )

    # ========================================================
    # CÁMARA
    # ========================================================

    def _initialize_camera(self):

        self.get_logger().info(
            "Inicializando cámara mediante V4L2..."
        )

        camera = cv2.VideoCapture(
            self._camera_device,
            cv2.CAP_V4L2
        )

        if not camera.isOpened():

            self.get_logger().error(
                f"No se pudo abrir la cámara: "
                f"{self._camera_device}"
            )

            return None

        # ----------------------------------------------------
        # Formato MJPG
        # ----------------------------------------------------

        camera.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*"MJPG")
        )

        # ----------------------------------------------------
        # Resolución
        # ----------------------------------------------------

        camera.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            self._camera_width
        )

        camera.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            self._camera_height
        )

        # ----------------------------------------------------
        # FPS
        # ----------------------------------------------------

        camera.set(
            cv2.CAP_PROP_FPS,
            self._camera_fps
        )

        # ----------------------------------------------------
        # Leer primer frame
        # ----------------------------------------------------

        ret, frame = camera.read()

        if not ret or frame is None:

            self.get_logger().error(
                "La cámara abrió correctamente, "
                "pero no fue posible obtener el primer frame."
            )

            camera.release()

            return None

        # ----------------------------------------------------
        # Información real de la cámara
        # ----------------------------------------------------

        width = int(
            camera.get(cv2.CAP_PROP_FRAME_WIDTH)
        )

        height = int(
            camera.get(cv2.CAP_PROP_FRAME_HEIGHT)
        )

        fps = camera.get(
            cv2.CAP_PROP_FPS
        )

        fourcc = int(
            camera.get(cv2.CAP_PROP_FOURCC)
        )

        fourcc_text = "".join(
            [
                chr((fourcc >> 0) & 0xFF),
                chr((fourcc >> 8) & 0xFF),
                chr((fourcc >> 16) & 0xFF),
                chr((fourcc >> 24) & 0xFF),
            ]
        )

        self.get_logger().info(
            "Cámara inicializada:"
        )

        self.get_logger().info(
            f"  Resolución: {width}x{height}"
        )

        self.get_logger().info(
            f"  FPS: {fps:.2f}"
        )

        self.get_logger().info(
            f"  Formato: {fourcc_text}"
        )

        self.get_logger().info(
            "Primer frame capturado correctamente."
        )

        return camera

    # ========================================================
    # PROCESAMIENTO PRINCIPAL
    # ========================================================

    def _process_frame(self):

        # ----------------------------------------------------
        # Leer frame
        # ----------------------------------------------------

        ret, frame = self._camera.read()

        if not ret or frame is None:

            self.get_logger().warning(
                "No fue posible obtener un frame."
            )

            return

        # ----------------------------------------------------
        # Imagen de visualización
        # ----------------------------------------------------

        display_frame = frame.copy()

        # ====================================================
        # DETECCIÓN DE PERSONAS
        # ====================================================

        persons = self._detect_persons(frame)

        # ----------------------------------------------------
        # Actualizar tracker
        # ----------------------------------------------------

        tracking_result = self._person_tracker.update(
            persons
        )

        # ----------------------------------------------------
        # Verificar si existe operador
        # ----------------------------------------------------

        if not tracking_result.person_detected:

            self._draw_no_operator(
                display_frame
            )

            self._show_frame(
                display_frame
            )

            return

        # ----------------------------------------------------
        # Obtener operador principal
        # ----------------------------------------------------

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
        # RECORTE DEL OPERADOR
        # ====================================================

        person_image = self._crop_person(
            frame,
            primary_person
        )

        if person_image is None:

            self._show_frame(
                display_frame
            )

            return

        # ====================================================
        # DETECCIÓN DE LANDMARKS FACIALES
        # ====================================================

        face_result = (
            self._face_landmark_detector.update(
                person_image
            )
        )

        # ----------------------------------------------------
        # Si se encontró rostro
        # ----------------------------------------------------

        if face_result.face_detected:

            self._draw_face_information(
                display_frame,
                face_result,
                primary_person
            )

        else:

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
    # DETECCIÓN DE PERSONAS
    # ========================================================

    def _detect_persons(
            self,
            frame: np.ndarray
        ) -> list[dict]:

        return self._person_detector.update(frame)

    # ========================================================
    # RECORTE DEL OPERADOR
    # ========================================================

    def _crop_person(
        self,
        frame: np.ndarray,
        person
    ) -> np.ndarray | None:
        """
        Obtiene el recorte correspondiente al operador.

        Se agrega un margen alrededor del bounding box.
        """

        height, width = frame.shape[:2]

        # ----------------------------------------------------
        # Coordenadas
        # ----------------------------------------------------

        x1 = int(person.x1)
        y1 = int(person.y1)

        x2 = int(person.x2)
        y2 = int(person.y2)

        # ----------------------------------------------------
        # Margen
        # ----------------------------------------------------

        margin_x = int(
            0.10 * (x2 - x1)
        )

        margin_y = int(
            0.10 * (y2 - y1)
        )

        x1 = max(
            0,
            x1 - margin_x
        )

        y1 = max(
            0,
            y1 - margin_y
        )

        x2 = min(
            width,
            x2 + margin_x
        )

        y2 = min(
            height,
            y2 + margin_y
        )

        if x2 <= x1 or y2 <= y1:

            return None

        return frame[
            y1:y2,
            x1:x2
        ]

    # ========================================================
    # INFORMACIÓN DEL OPERADOR
    # ========================================================

    def _draw_person(
        self,
        frame: np.ndarray,
        person
    ):
        """
        Dibuja el bounding box y la información del operador.
        """

        x1 = int(person.x1)
        y1 = int(person.y1)

        x2 = int(person.x2)
        y2 = int(person.y2)

        # ----------------------------------------------------
        # Bounding box
        # ----------------------------------------------------

        cv2.rectangle(
            frame,
            (x1, y1),
            (x2, y2),
            (0, 255, 0),
            2
        )

        # ----------------------------------------------------
        # Texto
        # ----------------------------------------------------

        label = (
            f"Operator "
            f"ID:{person.track_id} "
            f"{person.confidence:.2f}"
        )

        cv2.putText(
            frame,
            label,
            (x1, max(y1 - 10, 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2
        )

    # ========================================================
    # INFORMACIÓN FACIAL
    # ========================================================

    def _draw_face_information(
        self,
        frame: np.ndarray,
        face_result,
        person
    ):
        """
        Dibuja la información facial.

        Los landmarks solamente se dibujan cuando:

            visualizar == True
            points_face == True
        """

        if not self._points_face:

            return

        if face_result.landmarks is None:

            return

        # ----------------------------------------------------
        # Actualmente face_result.landmarks están expresados
        # respecto al recorte del operador.
        #
        # Debemos trasladarlos nuevamente a las coordenadas
        # originales de la imagen.
        # ----------------------------------------------------

        x_offset = int(person.x1)
        y_offset = int(person.y1)

        for point in face_result.landmarks:

            x = int(point[0]) + x_offset
            y = int(point[1]) + y_offset

            cv2.circle(
                frame,
                (x, y),
                2,
                (0, 255, 255),
                -1
            )

    # ========================================================
    # SIN OPERADOR
    # ========================================================

    def _draw_no_operator(
        self,
        frame: np.ndarray
    ):

        cv2.putText(
            frame,
            "No operator detected",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2
        )

    # ========================================================
    # SIN ROSTRO
    # ========================================================

    def _draw_no_face(
        self,
        frame: np.ndarray,
        person
    ):

        x = int(person.x1)

        y = max(
            int(person.y1) - 35,
            20
        )

        cv2.putText(
            frame,
            "Face not detected",
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 165, 255),
            2
        )

    # ========================================================
    # VISUALIZACIÓN
    # ========================================================

    def _show_frame(
        self,
        frame: np.ndarray
    ):

        if not self._visualizar:

            return

        cv2.imshow(
            "Operator RGB Camera",
            frame
        )

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q") or key == 27:

            self.get_logger().info(
                "Cerrando visualización..."
            )

            self._visualizar = False

            cv2.destroyWindow(
                "Operator RGB Camera"
            )

    # ========================================================
    # DESTRUCTOR
    # ========================================================

    def destroy_node(self):

        self.get_logger().info(
            "Liberando cámara..."
        )

        if self._camera is not None:

            self._camera.release()

        cv2.destroyAllWindows()

        super().destroy_node()


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = None

    try:

        node = OperatorFatigueNode()

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    except Exception as e:

        print(
            f"Error en OperatorFatigueNode: {e}"
        )

    finally:

        if node is not None:

            node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":

    main()
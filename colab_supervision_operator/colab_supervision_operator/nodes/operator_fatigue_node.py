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

import rclpy
from rclpy.node import Node

from colab_supervision_operator.perception.fatigue_analyzer import FatigueAnalyzer
from colab_supervision_operator.perception.blink_analyzer import BlinkAnalyzer
from colab_supervision_operator.perception.expression_analyzer import ExpressionAnalyzer
from colab_supervision_operator.perception.head_pose_estimator import HeadPoseEstimator
from colab_supervision_operator.perception.attention_analyzer import AttentionAnalyzer
from colab_supervision_operator.perception.person_tracker import PersonTracker


class OperatorFatigueNode(Node):

    def __init__(self):
        super().__init__("operator_fatigue_node")

        # ==============================================================
        # PARÁMETROS
        # ==============================================================

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

        # --------------------------------------------------------------
        # Visualización de la cámara
        #
        # False -> no se crea ninguna ventana de OpenCV
        # True  -> se muestra la imagen capturada
        # --------------------------------------------------------------

        self.declare_parameter(
            "visualizar",
            False
        )

        # ==============================================================
        # OBTENER PARÁMETROS
        # ==============================================================

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

        # ==============================================================
        # MÓDULOS DE PERCEPCIÓN
        # ==============================================================

        self._fatigue_analyzer = FatigueAnalyzer()
        self._blink_analyzer = BlinkAnalyzer()
        self._expression_analyzer = ExpressionAnalyzer()
        self._head_pose_estimator = HeadPoseEstimator()
        self._attention_analyzer = AttentionAnalyzer()
        self._person_tracker = PersonTracker()

        # ==============================================================
        # CÁMARA
        # ==============================================================

        self._camera = None

        self._initialize_camera()

        # ==============================================================
        # TEMPORIZADOR
        # ==============================================================

        self._timer_period = 1.0 / self._camera_fps

        self._timer = self.create_timer(
            self._timer_period,
            self._process_frame
        )

        self.get_logger().info(
            "Operator fatigue node initialized"
        )

        self.get_logger().info(
            f"Camera device: {self._camera_device}"
        )

        self.get_logger().info(
            f"Visualization: {'enabled' if self._visualizar else 'disabled'}"
        )

    # ==================================================================
    # INICIALIZACIÓN DE LA CÁMARA
    # ==================================================================

    def _initialize_camera(self):

        self.get_logger().info(
            "Initializing RGB camera..."
        )

        # --------------------------------------------------------------
        # Abrir cámara utilizando V4L2
        # --------------------------------------------------------------

        self._camera = cv2.VideoCapture(
            self._camera_device,
            cv2.CAP_V4L2
        )

        if not self._camera.isOpened():

            self.get_logger().error(
                f"Could not open camera: {self._camera_device}"
            )

            self._camera = None

            return

        # --------------------------------------------------------------
        # Utilizar MJPG
        # --------------------------------------------------------------

        self._camera.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*"MJPG")
        )

        # --------------------------------------------------------------
        # Resolución
        # --------------------------------------------------------------

        self._camera.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            self._camera_width
        )

        self._camera.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            self._camera_height
        )

        # --------------------------------------------------------------
        # FPS
        # --------------------------------------------------------------

        self._camera.set(
            cv2.CAP_PROP_FPS,
            self._camera_fps
        )

        # ==============================================================
        # VERIFICAR CONFIGURACIÓN NEGOCIADA
        # ==============================================================

        actual_width = self._camera.get(
            cv2.CAP_PROP_FRAME_WIDTH
        )

        actual_height = self._camera.get(
            cv2.CAP_PROP_FRAME_HEIGHT
        )

        actual_fps = self._camera.get(
            cv2.CAP_PROP_FPS
        )

        actual_fourcc = int(
            self._camera.get(cv2.CAP_PROP_FOURCC)
        )

        fourcc_string = "".join(
            [
                chr(actual_fourcc & 0xFF),
                chr((actual_fourcc >> 8) & 0xFF),
                chr((actual_fourcc >> 16) & 0xFF),
                chr((actual_fourcc >> 24) & 0xFF),
            ]
        )

        self.get_logger().info(
            "RGB camera opened successfully"
        )

        self.get_logger().info(
            f"Resolution: {int(actual_width)}x{int(actual_height)}"
        )

        self.get_logger().info(
            f"FPS: {actual_fps:.2f}"
        )

        self.get_logger().info(
            f"Format: {fourcc_string}"
        )

        # ==============================================================
        # PROBAR PRIMER FRAME
        # ==============================================================

        ret, frame = self._camera.read()

        if not ret or frame is None:

            self.get_logger().error(
                "Camera opened but failed to capture first frame"
            )

            self._camera.release()
            self._camera = None

            return

        self.get_logger().info(
            f"First frame captured successfully: {frame.shape}"
        )

    # ==================================================================
    # PROCESAMIENTO DE FRAME
    # ==================================================================

    def _process_frame(self):

        if self._camera is None:
            return

        # --------------------------------------------------------------
        # Capturar frame
        # --------------------------------------------------------------

        ret, frame = self._camera.read()

        if not ret or frame is None:

            self.get_logger().warn(
                "Failed to read RGB frame"
            )

            return

        # ==============================================================
        # VISUALIZACIÓN
        # ==============================================================

        if self._visualizar:

            cv2.imshow(
                "Operator RGB Camera",
                frame
            )

            # ----------------------------------------------------------
            # Tecla Q o ESC para cerrar la ventana
            # ----------------------------------------------------------

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q") or key == 27:

                self.get_logger().info(
                    "Visualization window closed by user"
                )

                self._visualizar = False

                cv2.destroyWindow(
                    "Operator RGB Camera"
                )

        # ==============================================================
        # DETECCIÓN DE PERSONAS
        # ==============================================================

        persons = self._detect_persons(frame)

        tracked_persons = self._person_tracker.update(persons)

        # --------------------------------------------------------------
        # No hay personas detectadas
        # --------------------------------------------------------------

        if not tracked_persons:

            self.get_logger().debug(
                "No operator detected"
            )

            return

        # ==============================================================
        # SELECCIONAR OPERADOR PRINCIPAL
        # ==============================================================

        primary_person = self._person_tracker.get_primary_person()

        if primary_person is None:
            self.get_logger().debug(
                "No primary operator detected"
            )
            return

        # ==============================================================
        # EXTRAER INFORMACIÓN FACIAL
        # ==============================================================

        face_data = self._extract_face_data(
            frame,
            primary_person
        )

        if face_data is None:
            return

        # ==============================================================
        # ANÁLISIS DE PARPADEO
        # ==============================================================

        blink_result = self._blink_analyzer.analyze(
            face_data
        )

        # ==============================================================
        # ANÁLISIS DE EXPRESIÓN
        # ==============================================================

        expression_result = self._expression_analyzer.analyze(
            face_data
        )

        # ==============================================================
        # ESTIMACIÓN DE POSTURA DE CABEZA
        # ==============================================================

        head_pose_result = self._head_pose_estimator.estimate(
            face_data
        )

        # ==============================================================
        # ANÁLISIS DE ATENCIÓN
        # ==============================================================

        attention_result = self._attention_analyzer.analyze(
            head_pose_result
        )

        # ==============================================================
        # ANÁLISIS DE FATIGA
        # ==============================================================

        fatigue_result = self._fatigue_analyzer.analyze(
            blink_result=blink_result,
            expression_result=expression_result
        )

        # ==============================================================
        # PUBLICAR / PROCESAR RESULTADOS
        # ==============================================================

        self._publish_results(
            fatigue_result,
            attention_result
        )

    # ==================================================================
    # DETECCIÓN DE PERSONAS
    # ==================================================================

    def _detect_persons(self, frame):

        # TODO:
        # Implementar detector de personas.

        return []

    # ==================================================================
    # EXTRACCIÓN DE INFORMACIÓN FACIAL
    # ==================================================================

    def _extract_face_data(
        self,
        frame,
        person
    ):

        # TODO:
        # Implementar extracción de landmarks
        # e información facial.

        return None

    # ==================================================================
    # PUBLICACIÓN DE RESULTADOS
    # ==================================================================

    def _publish_results(
        self,
        fatigue_result,
        attention_result
    ):

        self.get_logger().debug(
            f"Fatigue result: {fatigue_result}"
        )

        self.get_logger().debug(
            f"Attention result: {attention_result}"
        )

    # ==================================================================
    # DESTRUCCIÓN DEL NODO
    # ==================================================================

    def destroy_node(self):

        self.get_logger().info(
            "Shutting down operator fatigue node..."
        )

        # --------------------------------------------------------------
        # Liberar cámara
        # --------------------------------------------------------------

        if self._camera is not None:

            self._camera.release()
            self._camera = None

        # --------------------------------------------------------------
        # Cerrar ventana de visualización
        # --------------------------------------------------------------

        cv2.destroyAllWindows()

        super().destroy_node()


# ======================================================================
# MAIN
# ======================================================================

def main(args=None):

    rclpy.init(args=args)

    node = OperatorFatigueNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()
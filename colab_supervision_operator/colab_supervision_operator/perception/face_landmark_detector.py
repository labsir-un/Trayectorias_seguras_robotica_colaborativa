# ============================================================
# face_landmark_detector.py
# ============================================================
#
# Detector de landmarks faciales utilizando:
#
#   MediaPipe Tasks - Face Landmarker
#
# Compatible con:
#
#   MediaPipe 1.0.1
#
# Entrada:
#
#   Imagen BGR proveniente de OpenCV.
#
# Salida:
#
#   - Landmarks faciales
#   - Bounding box facial
#   - Puntos para HeadPoseEstimator
#   - Puntos de ojos
#   - Puntos de boca
#   - Punto de nariz
#
# ============================================================

from dataclasses import dataclass
from pathlib import Path

import cv2
import mediapipe as mp
import numpy as np

from mediapipe.tasks import python
from mediapipe.tasks.python import vision


# ============================================================
# Resultado del detector
# ============================================================

@dataclass
class FaceLandmarkResult:

    # --------------------------------------------------------
    # Estado
    # --------------------------------------------------------

    face_detected: bool

    # --------------------------------------------------------
    # Todos los landmarks
    #
    # Shape:
    #
    #   (N, 2)
    #
    # Coordenadas en píxeles.
    # --------------------------------------------------------

    landmarks: np.ndarray | None

    # --------------------------------------------------------
    # Bounding box facial
    #
    # (x1, y1, x2, y2)
    # --------------------------------------------------------

    bbox: tuple[int, int, int, int] | None

    # --------------------------------------------------------
    # Puntos para HeadPoseEstimator
    #
    # Orden:
    #
    #   0 -> nariz
    #   1 -> mentón
    #   2 -> ojo izquierdo
    #   3 -> ojo derecho
    #   4 -> boca izquierda
    #   5 -> boca derecha
    #
    # Shape:
    #
    #   (6, 2)
    # --------------------------------------------------------

    head_pose_points: np.ndarray | None

    # --------------------------------------------------------
    # Ojos
    # --------------------------------------------------------

    left_eye_points: np.ndarray | None

    right_eye_points: np.ndarray | None

    # --------------------------------------------------------
    # Boca
    # --------------------------------------------------------

    mouth_points: np.ndarray | None

    # --------------------------------------------------------
    # Nariz
    # --------------------------------------------------------

    nose_point: tuple[int, int] | None


# ============================================================
# Face Landmark Detector
# ============================================================

class FaceLandmarkDetector:

    # ========================================================
    # ÍNDICES DE LANDMARKS
    # ========================================================

    # --------------------------------------------------------
    # Nariz
    # --------------------------------------------------------

    NOSE_INDEX = 1

    # --------------------------------------------------------
    # Mentón
    # --------------------------------------------------------

    CHIN_INDEX = 152

    # --------------------------------------------------------
    # Centros de ojos
    # --------------------------------------------------------

    LEFT_EYE_CENTER_INDEX = 468

    RIGHT_EYE_CENTER_INDEX = 473

    # --------------------------------------------------------
    # Boca
    # --------------------------------------------------------

    LEFT_MOUTH_INDEX = 61

    RIGHT_MOUTH_INDEX = 291

    # ========================================================
    # LANDMARKS DE OJOS
    # ========================================================

    LEFT_EYE = [
        33,
        160,
        158,
        133,
        153,
        144,
    ]

    RIGHT_EYE = [
        362,
        385,
        387,
        263,
        373,
        380,
    ]

    # ========================================================
    # LANDMARKS DE BOCA
    # ========================================================

    MOUTH = [
        61,
        146,
        91,
        181,
        84,
        17,
        314,
        405,
        321,
        375,
        291,
    ]

    # ========================================================
    # CONSTRUCTOR
    # ========================================================

    def __init__(
        self,
        model_path: str,
        min_detection_confidence: float = 0.5,
        min_tracking_confidence: float = 0.5,
    ):

        # ----------------------------------------------------
        # Ruta del modelo
        # ----------------------------------------------------

        self._model_path = Path(
            model_path
        )

        # ----------------------------------------------------
        # Verificar existencia
        # ----------------------------------------------------

        if not self._model_path.exists():

            raise FileNotFoundError(
                "No se encontró el modelo "
                f"Face Landmarker: "
                f"{self._model_path}"
            )

        # ====================================================
        # CONFIGURACIÓN MEDIAPIPE
        # ====================================================

        base_options = python.BaseOptions(
            model_asset_path=str(
                self._model_path
            )
        )

        options = vision.FaceLandmarkerOptions(

            base_options=base_options,

            running_mode=(
                vision.RunningMode.VIDEO
            ),

            num_faces=1,

            min_face_detection_confidence=(
                min_detection_confidence
            ),

            min_face_presence_confidence=(
                min_detection_confidence
            ),

            min_tracking_confidence=(
                min_tracking_confidence
            ),

            # No necesitamos estas salidas todavía.
            output_face_blendshapes=False,

            output_facial_transformation_matrixes=False,
        )

        # ====================================================
        # CREAR FACE LANDMARKER
        # ====================================================

        self._detector = (
            vision.FaceLandmarker.create_from_options(
                options
            )
        )

        # ====================================================
        # TIMESTAMP
        # ====================================================

        self._timestamp_ms = 0

    # ========================================================
    # PROCESAR IMAGEN
    # ========================================================

    def update(
        self,
        image: np.ndarray,
    ) -> FaceLandmarkResult:

        # ----------------------------------------------------
        # Validar imagen
        # ----------------------------------------------------

        if image is None:

            return self._empty_result()

        if image.size == 0:

            return self._empty_result()

        # ----------------------------------------------------
        # Dimensiones
        # ----------------------------------------------------

        height, width = image.shape[:2]

        # ====================================================
        # BGR -> RGB
        # ====================================================

        rgb_image = cv2.cvtColor(
            image,
            cv2.COLOR_BGR2RGB
        )

        # ====================================================
        # CREAR MP IMAGE
        # ====================================================
        #
        # IMPORTANTE:
        #
        # MediaPipe 1.0.1 utiliza:
        #
        #   mp.Image
        #
        # NO:
        #
        #   vision.MPImage
        #
        # ====================================================

        mp_image = mp.Image(
            image_format=mp.ImageFormat.SRGB,
            data=rgb_image,
        )

        # ====================================================
        # TIMESTAMP
        # ====================================================

        self._timestamp_ms += 33

        # ====================================================
        # DETECTAR LANDMARKS
        # ====================================================

        result = (
            self._detector.detect_for_video(
                mp_image,
                self._timestamp_ms,
            )
        )

        # ====================================================
        # VERIFICAR RESULTADO
        # ====================================================

        if not result.face_landmarks:

            return self._empty_result()

        # ====================================================
        # PRIMER ROSTRO
        # ====================================================

        face = result.face_landmarks[0]

        # ====================================================
        # LANDMARKS NORMALIZADOS -> PIXELES
        # ====================================================

        landmarks = np.array(
            [
                [
                    landmark.x * width,
                    landmark.y * height,
                ]
                for landmark in face
            ],
            dtype=np.float32,
        )

        # ====================================================
        # BOUNDING BOX
        # ====================================================

        x_min = int(
            np.min(
                landmarks[:, 0]
            )
        )

        y_min = int(
            np.min(
                landmarks[:, 1]
            )
        )

        x_max = int(
            np.max(
                landmarks[:, 0]
            )
        )

        y_max = int(
            np.max(
                landmarks[:, 1]
            )
        )

        # ----------------------------------------------------
        # Limitar a la imagen
        # ----------------------------------------------------

        x_min = max(
            0,
            x_min
        )

        y_min = max(
            0,
            y_min
        )

        x_max = min(
            width - 1,
            x_max
        )

        y_max = min(
            height - 1,
            y_max
        )

        bbox = (
            x_min,
            y_min,
            x_max,
            y_max,
        )

        # ====================================================
        # HEAD POSE
        # ====================================================

        head_pose_points = np.array(
            [
                landmarks[
                    self.NOSE_INDEX
                ],

                landmarks[
                    self.CHIN_INDEX
                ],

                landmarks[
                    self.LEFT_EYE_CENTER_INDEX
                ],

                landmarks[
                    self.RIGHT_EYE_CENTER_INDEX
                ],

                landmarks[
                    self.LEFT_MOUTH_INDEX
                ],

                landmarks[
                    self.RIGHT_MOUTH_INDEX
                ],
            ],
            dtype=np.float32,
        )

        # ====================================================
        # OJO IZQUIERDO
        # ====================================================

        left_eye_points = landmarks[
            self.LEFT_EYE
        ]

        # ====================================================
        # OJO DERECHO
        # ====================================================

        right_eye_points = landmarks[
            self.RIGHT_EYE
        ]

        # ====================================================
        # BOCA
        # ====================================================

        mouth_points = landmarks[
            self.MOUTH
        ]

        # ====================================================
        # NARIZ
        # ====================================================

        nose = landmarks[
            self.NOSE_INDEX
        ]

        nose_point = (
            int(nose[0]),
            int(nose[1]),
        )

        # ====================================================
        # RESULTADO
        # ====================================================

        return FaceLandmarkResult(

            face_detected=True,

            landmarks=landmarks,

            bbox=bbox,

            head_pose_points=(
                head_pose_points
            ),

            left_eye_points=(
                left_eye_points
            ),

            right_eye_points=(
                right_eye_points
            ),

            mouth_points=(
                mouth_points
            ),

            nose_point=nose_point,
        )

    # ========================================================
    # DIBUJAR LANDMARKS
    # ========================================================

    def draw_landmarks(
        self,
        image: np.ndarray,
        result: FaceLandmarkResult,
    ) -> np.ndarray:

        # ----------------------------------------------------
        # No hay rostro
        # ----------------------------------------------------

        if not result.face_detected:

            return image

        # ----------------------------------------------------
        # Copia
        # ----------------------------------------------------

        output = image.copy()

        # ----------------------------------------------------
        # Verificar landmarks
        # ----------------------------------------------------

        if result.landmarks is None:

            return output

        # ====================================================
        # TODOS LOS LANDMARKS
        # ====================================================

        for point in result.landmarks:

            x = int(
                point[0]
            )

            y = int(
                point[1]
            )

            if (
                0 <= x < output.shape[1]
                and
                0 <= y < output.shape[0]
            ):

                cv2.circle(
                    output,
                    (x, y),
                    1,
                    (0, 255, 0),
                    -1,
                )

        # ====================================================
        # PUNTOS HEAD POSE
        # ====================================================

        if result.head_pose_points is not None:

            for point in (
                result.head_pose_points
            ):

                x = int(
                    point[0]
                )

                y = int(
                    point[1]
                )

                cv2.circle(
                    output,
                    (x, y),
                    3,
                    (255, 0, 0),
                    -1,
                )

        # ====================================================
        # NARIZ
        # ====================================================

        if result.nose_point is not None:

            cv2.circle(
                output,
                result.nose_point,
                4,
                (0, 0, 255),
                -1,
            )

        return output

    # ========================================================
    # RESULTADO VACÍO
    # ========================================================

    @staticmethod
    def _empty_result():

        return FaceLandmarkResult(

            face_detected=False,

            landmarks=None,

            bbox=None,

            head_pose_points=None,

            left_eye_points=None,

            right_eye_points=None,

            mouth_points=None,

            nose_point=None,
        )

    # ========================================================
    # CERRAR DETECTOR
    # ========================================================

    def close(self):

        if (
            hasattr(
                self,
                "_detector"
            )
            and
            self._detector is not None
        ):

            self._detector.close()

            self._detector = None

    # ========================================================
    # DESTRUCTOR
    # ========================================================

    def __del__(self):

        try:

            self.close()

        except Exception:

            pass
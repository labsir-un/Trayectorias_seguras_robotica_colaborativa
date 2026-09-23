"""
face_landmark_detector.py

Detector de puntos faciales para el sistema de percepción del operador.

Responsabilidades:
    - Detectar el rostro en una imagen RGB.
    - Extraer landmarks faciales 2D.
    - Proporcionar los puntos necesarios para estimar:
        * Pose de cabeza.
        * Apertura de ojos.
        * Parpadeos.
        * Características faciales.
    - Proporcionar información para visualización.

Este módulo NO realiza:
    - Análisis de fatiga.
    - Estimación de atención.
    - Estimación de pose de cabeza.
    - Análisis de expresiones.
    - Seguimiento de personas.

Esas responsabilidades corresponden a los módulos especializados.
"""

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np


# ============================================================
# RESULTADOS
# ============================================================

@dataclass
class FaceLandmarkResult:
    """
    Resultado de la detección de landmarks faciales.
    """

    face_detected: bool

    # Todos los landmarks detectados.
    landmarks: Optional[np.ndarray] = None

    # Bounding box del rostro: x1, y1, x2, y2
    bbox: Optional[tuple[int, int, int, int]] = None

    # Seis puntos principales utilizados por HeadPoseEstimator.
    head_pose_points: Optional[np.ndarray] = None

    # Puntos de los ojos.
    left_eye_points: Optional[np.ndarray] = None
    right_eye_points: Optional[np.ndarray] = None

    # Puntos de la boca.
    mouth_points: Optional[np.ndarray] = None

    # Punto de referencia de la nariz.
    nose_point: Optional[np.ndarray] = None


# ============================================================
# DETECTOR
# ============================================================

class FaceLandmarkDetector:
    """
    Detector de landmarks faciales.

    La clase recibe una imagen RGB/BGR y devuelve las coordenadas
    2D de los puntos faciales detectados.
    """

    def __init__(
        self,
        min_detection_confidence: float = 0.5,
        max_num_faces: int = 1,
    ):

        self._min_detection_confidence = min_detection_confidence
        self._max_num_faces = max_num_faces

        # ----------------------------------------------------
        # Backend de detección
        # ----------------------------------------------------
        #
        # Aquí se inicializa el backend de landmarks faciales.
        #
        # La implementación concreta puede ser MediaPipe,
        # OpenCV u otro detector.
        #
        # Por ahora se deja preparado para integrarlo sin
        # mezclarlo con los demás analizadores.
        # ----------------------------------------------------

        self._detector = None

    # ========================================================
    # API PRINCIPAL
    # ========================================================

    def update(self, image: np.ndarray) -> FaceLandmarkResult:
        """
        Procesa una imagen y obtiene los landmarks faciales.

        Parameters
        ----------
        image : np.ndarray
            Imagen BGR proveniente de OpenCV.

        Returns
        -------
        FaceLandmarkResult
            Resultado de la detección.
        """

        if image is None:
            return FaceLandmarkResult(
                face_detected=False
            )

        if image.size == 0:
            return FaceLandmarkResult(
                face_detected=False
            )

        # ----------------------------------------------------
        # Detección del rostro
        # ----------------------------------------------------

        detection_result = self._detect(image)

        if detection_result is None:
            return FaceLandmarkResult(
                face_detected=False
            )

        landmarks = detection_result["landmarks"]
        bbox = detection_result["bbox"]

        # ----------------------------------------------------
        # Extraer grupos de puntos
        # ----------------------------------------------------

        head_pose_points = self._extract_head_pose_points(
            landmarks
        )

        left_eye_points = self._extract_left_eye(
            landmarks
        )

        right_eye_points = self._extract_right_eye(
            landmarks
        )

        mouth_points = self._extract_mouth(
            landmarks
        )

        nose_point = self._extract_nose(
            landmarks
        )

        return FaceLandmarkResult(
            face_detected=True,
            landmarks=landmarks,
            bbox=bbox,
            head_pose_points=head_pose_points,
            left_eye_points=left_eye_points,
            right_eye_points=right_eye_points,
            mouth_points=mouth_points,
            nose_point=nose_point,
        )

    # ========================================================
    # DETECCIÓN
    # ========================================================

    def _detect(
        self,
        image: np.ndarray
    ) -> Optional[dict]:
        """
        Ejecuta el detector de landmarks.

        Esta función es el punto de integración con el backend
        de visión artificial.
        """

        # ----------------------------------------------------
        # TODO:
        # Implementar el backend de landmarks.
        # ----------------------------------------------------

        return None

    # ========================================================
    # HEAD POSE
    # ========================================================

    def _extract_head_pose_points(
        self,
        landmarks: np.ndarray
    ) -> Optional[np.ndarray]:
        """
        Extrae los seis puntos requeridos por HeadPoseEstimator.

        Orden:

            0 -> nariz
            1 -> mentón
            2 -> ojo izquierdo
            3 -> ojo derecho
            4 -> esquina izquierda de la boca
            5 -> esquina derecha de la boca

        Returns
        -------
        np.ndarray
            Array de dimensiones (6, 2).
        """

        if landmarks is None:
            return None

        try:
            nose = landmarks[0]
            chin = landmarks[1]
            left_eye = landmarks[2]
            right_eye = landmarks[3]
            left_mouth = landmarks[4]
            right_mouth = landmarks[5]

            points = np.array(
                [
                    nose,
                    chin,
                    left_eye,
                    right_eye,
                    left_mouth,
                    right_mouth,
                ],
                dtype=np.float32,
            )

            return points

        except (IndexError, TypeError):
            return None

    # ========================================================
    # OJOS
    # ========================================================

    def _extract_left_eye(
        self,
        landmarks: np.ndarray
    ) -> Optional[np.ndarray]:

        if landmarks is None:
            return None

        # Se reemplazarán por los índices reales del backend.
        return None

    def _extract_right_eye(
        self,
        landmarks: np.ndarray
    ) -> Optional[np.ndarray]:

        if landmarks is None:
            return None

        # Se reemplazarán por los índices reales del backend.
        return None

    # ========================================================
    # BOCA
    # ========================================================

    def _extract_mouth(
        self,
        landmarks: np.ndarray
    ) -> Optional[np.ndarray]:

        if landmarks is None:
            return None

        # Se reemplazarán por los índices reales del backend.
        return None

    # ========================================================
    # NARIZ
    # ========================================================

    def _extract_nose(
        self,
        landmarks: np.ndarray
    ) -> Optional[np.ndarray]:

        if landmarks is None:
            return None

        # Se reemplazará por el índice real del backend.
        return None

    # ========================================================
    # VISUALIZACIÓN
    # ========================================================

    def draw_landmarks(
        self,
        image: np.ndarray,
        result: FaceLandmarkResult,
        draw_all: bool = False,
    ) -> np.ndarray:
        """
        Dibuja los landmarks sobre la imagen.

        Parameters
        ----------
        image : np.ndarray
            Imagen original.

        result : FaceLandmarkResult
            Resultado obtenido mediante update().

        draw_all : bool
            Si True, dibuja todos los landmarks.
            Si False, dibuja únicamente los puntos principales.

        Returns
        -------
        np.ndarray
            Imagen con la información visualizada.
        """

        output = image.copy()

        if not result.face_detected:
            return output

        # ----------------------------------------------------
        # Bounding box
        # ----------------------------------------------------

        if result.bbox is not None:

            x1, y1, x2, y2 = result.bbox

            cv2.rectangle(
                output,
                (x1, y1),
                (x2, y2),
                (0, 255, 0),
                2,
            )

        # ----------------------------------------------------
        # Todos los landmarks
        # ----------------------------------------------------

        if draw_all and result.landmarks is not None:

            for point in result.landmarks:

                x = int(point[0])
                y = int(point[1])

                cv2.circle(
                    output,
                    (x, y),
                    1,
                    (0, 255, 255),
                    -1,
                )

        # ----------------------------------------------------
        # Puntos utilizados para HeadPoseEstimator
        # ----------------------------------------------------

        if result.head_pose_points is not None:

            for point in result.head_pose_points:

                x = int(point[0])
                y = int(point[1])

                cv2.circle(
                    output,
                    (x, y),
                    4,
                    (255, 0, 0),
                    -1,
                )

        return output
"""
Blink Analyzer
==============

Analizador de parpadeo basado en Eye Aspect Ratio (EAR).

Recibe los puntos de los ojos obtenidos por FaceLandmarkDetector
y determina:

    - EAR del ojo izquierdo.
    - EAR del ojo derecho.
    - EAR promedio.
    - Apertura ocular.
    - Estado de cada ojo.
    - Estado general de los ojos.
    - Detección de parpadeos.
    - Duración del parpadeo.
    - Frecuencia de parpadeo.
    - Número total de parpadeos.

No depende de ROS 2 ni realiza detección facial.
"""

from dataclasses import dataclass
from typing import Optional

import numpy as np


# ============================================================================
# RESULTADO DEL ANALISIS
# ============================================================================

@dataclass
class BlinkResult:
    """Resultado generado por BlinkAnalyzer."""

    # EAR
    left_ear: float
    right_ear: float
    mean_ear: float

    # Apertura ocular
    eye_openness: float

    # Estado de los ojos
    left_eye_closed: bool
    right_eye_closed: bool
    eyes_closed: bool

    # Parpadeo
    blink_detected: bool
    blink_duration: float
    blink_frequency: float
    total_blinks: int

    # Validación
    valid: bool


# ============================================================================
# ANALIZADOR DE PARPADEO
# ============================================================================

class BlinkAnalyzer:
    """
    Analizador de parpadeo basado en Eye Aspect Ratio (EAR).

    Orden esperado de los puntos:

    Ojo izquierdo:
        [33, 160, 158, 133, 153, 144]

    Ojo derecho:
        [362, 385, 387, 263, 373, 380]

    Geometría:

             p2       p3
              ●-------●

          p1 ●         ● p4

              ●-------●
             p6       p5

    EAR = (||p2-p6|| + ||p3-p5||)
          -------------------------
                 2||p1-p4||
    """

    def __init__(
        self,
        eye_closed_threshold: float = 0.21,
        min_closed_frames: int = 2,
        max_closed_frames: int = 20,
        fps: float = 30.0,
        frequency_window_seconds: float = 60.0,
        openness_min_ear: float = 0.10,
        openness_max_ear: float = 0.35,
    ):
        # ====================================================================
        # PARAMETROS
        # ====================================================================

        self.eye_closed_threshold = float(
            eye_closed_threshold
        )

        self.min_closed_frames = int(
            min_closed_frames
        )

        self.max_closed_frames = int(
            max_closed_frames
        )

        self.fps = max(float(fps), 1.0)

        self.frequency_window_seconds = max(
            float(frequency_window_seconds),
            1.0,
        )

        self.openness_min_ear = float(
            openness_min_ear
        )

        self.openness_max_ear = float(
            openness_max_ear
        )

        # ====================================================================
        # ESTADO TEMPORAL
        # ====================================================================

        self._left_closed_frames = 0
        self._right_closed_frames = 0

        self._blink_in_progress = False
        self._blink_start_frame = None

        self._last_blink_duration = 0.0

        # ====================================================================
        # ESTADISTICAS
        # ====================================================================

        self._total_blinks = 0
        self._blink_times = []

        self._frame_count = 0

    # ========================================================================
    # ACTUALIZACION PRINCIPAL
    # ========================================================================

    def update(
        self,
        left_eye_points: Optional[np.ndarray],
        right_eye_points: Optional[np.ndarray],
    ) -> BlinkResult:
        """
        Procesa los puntos de ambos ojos.
        """

        self._frame_count += 1

        # --------------------------------------------------------------------
        # Validar puntos
        # --------------------------------------------------------------------

        if not self._validate_eye_points(left_eye_points):
            return self._invalid_result()

        if not self._validate_eye_points(right_eye_points):
            return self._invalid_result()

        left_eye_points = np.asarray(
            left_eye_points,
            dtype=np.float32,
        )

        right_eye_points = np.asarray(
            right_eye_points,
            dtype=np.float32,
        )

        # --------------------------------------------------------------------
        # EAR
        # --------------------------------------------------------------------

        left_ear = self._calculate_ear(
            left_eye_points
        )

        right_ear = self._calculate_ear(
            right_eye_points
        )

        mean_ear = (
            left_ear + right_ear
        ) / 2.0

        # --------------------------------------------------------------------
        # Apertura ocular
        # --------------------------------------------------------------------

        eye_openness = self._calculate_eye_openness(
            mean_ear
        )

        # --------------------------------------------------------------------
        # Estado de los ojos
        # --------------------------------------------------------------------

        left_eye_closed = (
            left_ear < self.eye_closed_threshold
        )

        right_eye_closed = (
            right_ear < self.eye_closed_threshold
        )

        eyes_closed = (
            left_eye_closed
            and right_eye_closed
        )

        # --------------------------------------------------------------------
        # Actualizar estado de cierre
        # --------------------------------------------------------------------

        self._update_closed_state(
            left_eye_closed,
            right_eye_closed,
        )

        # --------------------------------------------------------------------
        # Detectar parpadeo
        # --------------------------------------------------------------------

        blink_detected = self._process_blink_state(
            eyes_closed
        )

        # --------------------------------------------------------------------
        # Tiempo actual
        # --------------------------------------------------------------------

        current_time = (
            self._frame_count / self.fps
        )

        # --------------------------------------------------------------------
        # Registrar parpadeo
        # --------------------------------------------------------------------

        if blink_detected:
            self._total_blinks += 1

            self._blink_times.append(
                current_time
            )

        # --------------------------------------------------------------------
        # Frecuencia
        # --------------------------------------------------------------------

        blink_frequency = (
            self._calculate_blink_frequency(
                current_time
            )
        )

        # --------------------------------------------------------------------
        # Resultado
        # --------------------------------------------------------------------

        return BlinkResult(
            left_ear=left_ear,
            right_ear=right_ear,
            mean_ear=mean_ear,

            eye_openness=eye_openness,

            left_eye_closed=left_eye_closed,
            right_eye_closed=right_eye_closed,
            eyes_closed=eyes_closed,

            blink_detected=blink_detected,
            blink_duration=self._last_blink_duration,
            blink_frequency=blink_frequency,
            total_blinks=self._total_blinks,

            valid=True,
        )

    # ========================================================================
    # EAR
    # ========================================================================

    @staticmethod
    def _calculate_ear(
        eye_points: np.ndarray,
    ) -> float:
        """
        Calcula el Eye Aspect Ratio.
        """

        p1 = eye_points[0]
        p2 = eye_points[1]
        p3 = eye_points[2]
        p4 = eye_points[3]
        p5 = eye_points[4]
        p6 = eye_points[5]

        horizontal_distance = np.linalg.norm(
            p1 - p4
        )

        if horizontal_distance <= 1e-6:
            return 0.0

        vertical_distance_1 = np.linalg.norm(
            p2 - p6
        )

        vertical_distance_2 = np.linalg.norm(
            p3 - p5
        )

        ear = (
            vertical_distance_1
            + vertical_distance_2
        ) / (
            2.0 * horizontal_distance
        )

        return float(ear)

    # ========================================================================
    # APERTURA OCULAR
    # ========================================================================

    def _calculate_eye_openness(
        self,
        mean_ear: float,
    ) -> float:
        """
        Convierte el EAR promedio a un valor normalizado [0, 1].
        """

        denominator = (
            self.openness_max_ear
            - self.openness_min_ear
        )

        if denominator <= 1e-6:
            return 0.0

        openness = (
            mean_ear
            - self.openness_min_ear
        ) / denominator

        return float(
            np.clip(openness, 0.0, 1.0)
        )

    # ========================================================================
    # ESTADO DE CIERRE
    # ========================================================================

    def _update_closed_state(
        self,
        left_eye_closed: bool,
        right_eye_closed: bool,
    ):
        """Actualiza los frames consecutivos de cierre."""

        if left_eye_closed:
            self._left_closed_frames += 1
        else:
            self._left_closed_frames = 0

        if right_eye_closed:
            self._right_closed_frames += 1
        else:
            self._right_closed_frames = 0

    # ========================================================================
    # DETECCION DE PARPADEO
    # ========================================================================

    def _process_blink_state(
        self,
        eyes_closed: bool,
    ) -> bool:
        """
        Detecta un parpadeo mediante una máquina de estados.

        ABIERTOS
            ↓
        CERRADOS
            ↓
        ABIERTOS
            ↓
        PARPADEO
        """

        blink_detected = False

        # --------------------------------------------------------------------
        # Ojos cerrados
        # --------------------------------------------------------------------

        if eyes_closed:

            if (
                not self._blink_in_progress
                and self._left_closed_frames
                >= self.min_closed_frames
                and self._right_closed_frames
                >= self.min_closed_frames
            ):
                self._blink_in_progress = True

                self._blink_start_frame = (
                    self._frame_count
                    - self.min_closed_frames
                    + 1
                )

            return False

        # --------------------------------------------------------------------
        # Ojos abiertos
        # --------------------------------------------------------------------

        if not eyes_closed:

            if not self._blink_in_progress:
                return False

            closed_frames = max(
                self._left_closed_frames,
                self._right_closed_frames,
            )

            # ----------------------------------------------------------------
            # Parpadeo válido
            # ----------------------------------------------------------------

            if (
                self.min_closed_frames
                <= closed_frames
                <= self.max_closed_frames
            ):

                self._last_blink_duration = (
                    closed_frames / self.fps
                )

                blink_detected = True

            # ----------------------------------------------------------------
            # Reiniciar estado
            # ----------------------------------------------------------------

            self._blink_in_progress = False
            self._blink_start_frame = None

            self._left_closed_frames = 0
            self._right_closed_frames = 0

        return blink_detected

    # ========================================================================
    # FRECUENCIA
    # ========================================================================

    def _calculate_blink_frequency(
        self,
        current_time: float,
    ) -> float:
        """
        Calcula la frecuencia de parpadeo en parpadeos/minuto.
        """

        window_start = (
            current_time
            - self.frequency_window_seconds
        )

        self._blink_times = [
            timestamp
            for timestamp in self._blink_times
            if timestamp >= window_start
        ]

        if self.frequency_window_seconds <= 0:
            return 0.0

        frequency = (
            len(self._blink_times)
            / self.frequency_window_seconds
        ) * 60.0

        return float(frequency)

    # ========================================================================
    # VALIDACION
    # ========================================================================

    @staticmethod
    def _validate_eye_points(
        eye_points: Optional[np.ndarray],
    ) -> bool:
        """
        Verifica que existan exactamente seis puntos 2D válidos.
        """

        if eye_points is None:
            return False

        try:
            points = np.asarray(
                eye_points
            )

            if points.shape != (6, 2):
                return False

            if not np.all(
                np.isfinite(points)
            ):
                return False

        except Exception:
            return False

        return True

    # ========================================================================
    # RESULTADO INVALIDO
    # ========================================================================

    def _invalid_result(self) -> BlinkResult:
        """
        Genera un resultado inválido conservando las estadísticas
        acumuladas.
        """

        current_time = (
            self._frame_count / self.fps
        )

        blink_frequency = (
            self._calculate_blink_frequency(
                current_time
            )
        )

        return BlinkResult(
            left_ear=0.0,
            right_ear=0.0,
            mean_ear=0.0,

            eye_openness=0.0,

            left_eye_closed=False,
            right_eye_closed=False,
            eyes_closed=False,

            blink_detected=False,
            blink_duration=self._last_blink_duration,
            blink_frequency=blink_frequency,
            total_blinks=self._total_blinks,

            valid=False,
        )

    # ========================================================================
    # RESET
    # ========================================================================

    def reset(self):
        """Reinicia completamente el estado del analizador."""

        self._left_closed_frames = 0
        self._right_closed_frames = 0

        self._blink_in_progress = False
        self._blink_start_frame = None

        self._last_blink_duration = 0.0

        self._total_blinks = 0
        self._blink_times.clear()

        self._frame_count = 0
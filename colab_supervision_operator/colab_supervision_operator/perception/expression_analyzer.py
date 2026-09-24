# ============================================================
# expression_analyzer.py
# ============================================================

"""
Analizador de expresiones faciales para
colab_supervision_operator.

Expresiones convencionales:

    - neutral
    - smile
    - frown
    - surprise

Estados propios de la aplicación:

    - sleepy
    - tired
    - fatigued
    - yawn

Los estados propios no son categorías nativas de MediaPipe.
Se calculan mediante reglas sobre los face blendshapes y
persistencia temporal.
"""

from dataclasses import dataclass
from typing import Optional


# ============================================================
# RESULTADO
# ============================================================

@dataclass
class ExpressionResult:
    """
    Resultado completo del análisis de expresión.
    """

    # --------------------------------------------------------
    # Expresión clasificada
    # --------------------------------------------------------

    expression: str

    confidence: float

    # --------------------------------------------------------
    # Expresiones convencionales
    # --------------------------------------------------------

    smile_score: float

    frown_score: float

    surprise_score: float

    neutral_score: float

    # --------------------------------------------------------
    # Estados propios
    # --------------------------------------------------------

    sleepy_score: float

    tired_score: float

    fatigued_score: float

    yawn_score: float

    # --------------------------------------------------------
    # Estado general
    # --------------------------------------------------------

    is_fatigue_expression: bool

    expression_valid: bool

    # --------------------------------------------------------
    # Blendshapes
    # --------------------------------------------------------

    blendshape_scores: dict[str, float]


# ============================================================
# EXPRESSION ANALYZER
# ============================================================

class ExpressionAnalyzer:
    """
    Analiza expresiones faciales mediante face blendshapes.
    """

    # ========================================================
    # CONSTRUCTOR
    # ========================================================

    def __init__(
        self,

        expression_threshold: float = 0.30,

        fatigue_expressions: Optional[set[str]] = None,

        smoothing_alpha: float = 0.35,

        # ----------------------------------------------------
        # Thresholds
        # ----------------------------------------------------

        sleepy_threshold: float = 0.45,

        tired_threshold: float = 0.40,

        fatigued_threshold: float = 0.45,

        yawn_threshold: float = 0.50,

        # ----------------------------------------------------
        # Persistencia temporal
        # ----------------------------------------------------

        sleepy_frames: int = 6,

        tired_frames: int = 10,

        fatigued_frames: int = 20,

        yawn_frames: int = 8,
    ) -> None:

        # ====================================================
        # EXPRESIONES ASOCIADAS A FATIGA
        # ====================================================

        if fatigue_expressions is None:

            self._fatigue_expressions = {
                "sleepy",
                "tired",
                "fatigued",
                "yawn",
            }

        else:

            self._fatigue_expressions = {
                expression.lower()
                for expression in fatigue_expressions
            }

        # ====================================================
        # CONFIGURACIÓN
        # ====================================================

        self._expression_threshold = (
            expression_threshold
        )

        self._smoothing_alpha = (
            smoothing_alpha
        )

        # ----------------------------------------------------
        # Thresholds
        # ----------------------------------------------------

        self._sleepy_threshold = (
            sleepy_threshold
        )

        self._tired_threshold = (
            tired_threshold
        )

        self._fatigued_threshold = (
            fatigued_threshold
        )

        self._yawn_threshold = (
            yawn_threshold
        )

        # ----------------------------------------------------
        # Frames requeridos
        # ----------------------------------------------------

        self._sleepy_frames_required = (
            int(sleepy_frames)
        )

        self._tired_frames_required = (
            int(tired_frames)
        )

        self._fatigued_frames_required = (
            int(fatigued_frames)
        )

        self._yawn_frames_required = (
            int(yawn_frames)
        )

        # ====================================================
        # VALIDAR CONFIGURACIÓN
        # ====================================================

        self._validate_range(
            expression_threshold,
            0.0,
            1.0,
            "expression_threshold"
        )

        self._validate_range(
            smoothing_alpha,
            0.0,
            1.0,
            "smoothing_alpha"
        )

        self._validate_range(
            sleepy_threshold,
            0.0,
            1.0,
            "sleepy_threshold"
        )

        self._validate_range(
            tired_threshold,
            0.0,
            1.0,
            "tired_threshold"
        )

        self._validate_range(
            fatigued_threshold,
            0.0,
            1.0,
            "fatigued_threshold"
        )

        self._validate_range(
            yawn_threshold,
            0.0,
            1.0,
            "yawn_threshold"
        )

        self._validate_positive_int(
            self._sleepy_frames_required,
            "sleepy_frames"
        )

        self._validate_positive_int(
            self._tired_frames_required,
            "tired_frames"
        )

        self._validate_positive_int(
            self._fatigued_frames_required,
            "fatigued_frames"
        )

        self._validate_positive_int(
            self._yawn_frames_required,
            "yawn_frames"
        )

        # ====================================================
        # ESTADO
        # ====================================================

        self._expression = "unknown"

        self._confidence = 0.0

        # ====================================================
        # SCORES CONVENCIONALES
        # ====================================================

        self._smile_score = 0.0

        self._frown_score = 0.0

        self._surprise_score = 0.0

        self._neutral_score = 0.0

        # ====================================================
        # SCORES PROPIOS
        # ====================================================

        self._sleepy_score = 0.0

        self._tired_score = 0.0

        self._fatigued_score = 0.0

        self._yawn_score = 0.0

        # ====================================================
        # ESTADO GENERAL
        # ====================================================

        self._is_fatigue_expression = False

        self._expression_valid = False

        # ====================================================
        # BLENDSHAPES
        # ====================================================

        self._blendshape_scores: dict[str, float] = {}

        self._smoothed_scores: dict[str, float] = {}

        # ====================================================
        # CONTADORES TEMPORALES
        # ====================================================

        self._sleepy_counter = 0

        self._tired_counter = 0

        self._fatigued_counter = 0

        self._yawn_counter = 0

    # ========================================================
    # UPDATE
    # ========================================================

    def update(
        self,
        face_blendshapes: dict[str, float],
    ) -> ExpressionResult:

        """
        Actualiza el análisis facial.
        """

        self._validate_blendshapes(
            face_blendshapes
        )

        # ----------------------------------------------------
        # Suavizado temporal
        # ----------------------------------------------------

        self._blendshape_scores = (
            self._smooth_blendshapes(
                face_blendshapes
            )
        )

        # ====================================================
        # EXPRESIONES CONVENCIONALES
        # ====================================================

        self._smile_score = (
            self._calculate_smile_score()
        )

        self._frown_score = (
            self._calculate_frown_score()
        )

        self._surprise_score = (
            self._calculate_surprise_score()
        )

        self._neutral_score = (
            self._get_score("neutral")
        )

        # ====================================================
        # ESTADOS PROPIOS
        # ====================================================

        self._sleepy_score = (
            self._calculate_sleepy_score()
        )

        self._tired_score = (
            self._calculate_tired_score()
        )

        self._fatigued_score = (
            self._calculate_fatigued_score()
        )

        self._yawn_score = (
            self._calculate_yawn_score()
        )

        # ====================================================
        # PERSISTENCIA
        # ====================================================

        self._update_temporal_counters()

        # ====================================================
        # CLASIFICACIÓN
        # ====================================================

        self._expression = (
            self._classify_expression()
        )

        # ====================================================
        # CONFIANZA
        # ====================================================

        self._confidence = (
            self._calculate_confidence()
        )

        # ====================================================
        # ESTADO GENERAL
        # ====================================================

        self._expression_valid = (
            self._expression != "unknown"
        )

        self._is_fatigue_expression = (
            self._expression.lower()
            in self._fatigue_expressions
        )

        return self.get_result()

    # ========================================================
    # SMILE
    # ========================================================

    def _calculate_smile_score(
        self
    ) -> float:

        left = self._get_score(
            "mouthSmileLeft"
        )

        right = self._get_score(
            "mouthSmileRight"
        )

        score = (
            left + right
        ) / 2.0

        return self._clamp(
            score
        )

    # ========================================================
    # FROWN
    # ========================================================

    def _calculate_frown_score(
        self
    ) -> float:

        left = self._get_score(
            "mouthFrownLeft"
        )

        right = self._get_score(
            "mouthFrownRight"
        )

        mean_score = (
            left + right
        ) / 2.0

        max_score = max(
            left,
            right
        )

        # Mayor sensibilidad ante activación unilateral
        raw_score = (
            0.60 * mean_score
            +
            0.40 * max_score
        )

        # Curva no lineal
        score = (
            1.0 -
            (1.0 - raw_score) ** 2.0
        )

        return self._clamp(
            score
        )

    # ========================================================
    # SURPRISE
    # ========================================================

    def _calculate_surprise_score(
        self
    ) -> float:

        jaw_open = self._get_score(
            "jawOpen"
        )

        eye_wide_left = self._get_score(
            "eyeWideLeft"
        )

        eye_wide_right = self._get_score(
            "eyeWideRight"
        )

        brow_inner_up = self._get_score(
            "browInnerUp"
        )

        eye_wide = (
            eye_wide_left
            +
            eye_wide_right
        ) / 2.0

        score = (
            0.40 * jaw_open
            +
            0.35 * eye_wide
            +
            0.25 * brow_inner_up
        )

        return self._clamp(
            score
        )

    # ========================================================
    # SLEEPY
    # ========================================================

    def _calculate_sleepy_score(
        self
    ) -> float:

        """
        Estima somnolencia mediante:

            - cierre de ojos
            - estrechamiento de ojos
        """

        blink_left = self._get_score(
            "eyeBlinkLeft"
        )

        blink_right = self._get_score(
            "eyeBlinkRight"
        )

        squint_left = self._get_score(
            "eyeSquintLeft"
        )

        squint_right = self._get_score(
            "eyeSquintRight"
        )

        blink_score = (
            blink_left
            +
            blink_right
        ) / 2.0

        squint_score = (
            squint_left
            +
            squint_right
        ) / 2.0

        score = (
            0.65 * blink_score
            +
            0.35 * squint_score
        )

        return self._clamp(
            score
        )

    # ========================================================
    # TIRED
    # ========================================================

    def _calculate_tired_score(
        self
    ) -> float:

        """
        Estima cansancio facial mediante:

            - estrechamiento ocular
            - descenso de comisuras
            - descenso de cejas
        """

        squint_left = self._get_score(
            "eyeSquintLeft"
        )

        squint_right = self._get_score(
            "eyeSquintRight"
        )

        frown_left = self._get_score(
            "mouthFrownLeft"
        )

        frown_right = self._get_score(
            "mouthFrownRight"
        )

        brow_down_left = self._get_score(
            "browDownLeft"
        )

        brow_down_right = self._get_score(
            "browDownRight"
        )

        squint = (
            squint_left
            +
            squint_right
        ) / 2.0

        frown = (
            frown_left
            +
            frown_right
        ) / 2.0

        brow_down = (
            brow_down_left
            +
            brow_down_right
        ) / 2.0

        score = (
            0.50 * squint
            +
            0.30 * frown
            +
            0.20 * brow_down
        )

        return self._clamp(
            score
        )

    # ========================================================
    # FATIGUED
    # ========================================================

    def _calculate_fatigued_score(
        self
    ) -> float:

        """
        Estima fatiga facial mediante una combinación de:

            - cierre ocular
            - estrechamiento ocular
            - mouthFrown
            - mouthPress
            - browDown
        """

        blink_left = self._get_score(
            "eyeBlinkLeft"
        )

        blink_right = self._get_score(
            "eyeBlinkRight"
        )

        squint_left = self._get_score(
            "eyeSquintLeft"
        )

        squint_right = self._get_score(
            "eyeSquintRight"
        )

        frown_left = self._get_score(
            "mouthFrownLeft"
        )

        frown_right = self._get_score(
            "mouthFrownRight"
        )

        press_left = self._get_score(
            "mouthPressLeft"
        )

        press_right = self._get_score(
            "mouthPressRight"
        )

        brow_down_left = self._get_score(
            "browDownLeft"
        )

        brow_down_right = self._get_score(
            "browDownRight"
        )

        eye_closure = (
            blink_left
            +
            blink_right
        ) / 2.0

        eye_squint = (
            squint_left
            +
            squint_right
        ) / 2.0

        mouth_frown = (
            frown_left
            +
            frown_right
        ) / 2.0

        mouth_press = (
            press_left
            +
            press_right
        ) / 2.0

        brow_down = (
            brow_down_left
            +
            brow_down_right
        ) / 2.0

        # ----------------------------------------------------
        # Indicador ocular
        # ----------------------------------------------------

        ocular_fatigue = (
            0.60 * eye_closure
            +
            0.40 * eye_squint
        )

        # ----------------------------------------------------
        # Score final
        # ----------------------------------------------------

        score = (
            0.40 * ocular_fatigue
            +
            0.30 * mouth_frown
            +
            0.15 * mouth_press
            +
            0.15 * brow_down
        )

        return self._clamp(
            score
        )

    # ========================================================
    # YAWN
    # ========================================================

    def _calculate_yawn_score(
        self
    ) -> float:

        """
        Estima bostezo mediante:

            - apertura mandibular
            - estiramiento horizontal de la boca

        jawOpen representa el indicador principal.
        """

        jaw_open = self._get_score(
            "jawOpen"
        )

        stretch_left = self._get_score(
            "mouthStretchLeft"
        )

        stretch_right = self._get_score(
            "mouthStretchRight"
        )

        mouth_stretch = (
            stretch_left
            +
            stretch_right
        ) / 2.0

        score = (
            0.65 * jaw_open
            +
            0.35 * mouth_stretch
        )

        return self._clamp(
            score
        )

    # ========================================================
    # CONTADORES TEMPORALES
    # ========================================================

    def _update_temporal_counters(
        self
    ) -> None:

        # ----------------------------------------------------
        # Sleepy
        # ----------------------------------------------------

        if (
            self._sleepy_score
            >=
            self._sleepy_threshold
        ):

            self._sleepy_counter += 1

        else:

            self._sleepy_counter = 0

        # ----------------------------------------------------
        # Tired
        # ----------------------------------------------------

        if (
            self._tired_score
            >=
            self._tired_threshold
        ):

            self._tired_counter += 1

        else:

            self._tired_counter = 0

        # ----------------------------------------------------
        # Fatigued
        # ----------------------------------------------------

        if (
            self._fatigued_score
            >=
            self._fatigued_threshold
        ):

            self._fatigued_counter += 1

        else:

            self._fatigued_counter = 0

        # ----------------------------------------------------
        # Yawn
        # ----------------------------------------------------

        if (
            self._yawn_score
            >=
            self._yawn_threshold
        ):

            self._yawn_counter += 1

        else:

            self._yawn_counter = 0

    # ========================================================
    # CLASIFICACIÓN
    # ========================================================

    def _classify_expression(
        self
    ) -> str:

        # ====================================================
        # ESTADOS PROPIOS
        # ====================================================

        # ----------------------------------------------------
        # Yawn
        # ----------------------------------------------------

        if (
            self._yawn_counter
            >=
            self._yawn_frames_required
        ):

            return "yawn"

        # ----------------------------------------------------
        # Fatigued
        # ----------------------------------------------------

        if (
            self._fatigued_counter
            >=
            self._fatigued_frames_required
        ):

            return "fatigued"

        # ----------------------------------------------------
        # Sleepy
        # ----------------------------------------------------

        if (
            self._sleepy_counter
            >=
            self._sleepy_frames_required
        ):

            return "sleepy"

        # ----------------------------------------------------
        # Tired
        # ----------------------------------------------------

        if (
            self._tired_counter
            >=
            self._tired_frames_required
        ):

            return "tired"

        # ====================================================
        # EXPRESIONES CONVENCIONALES
        # ====================================================

        scores = {

            "smile":
                self._smile_score,

            "frown":
                self._frown_score,

            "surprise":
                self._surprise_score,

            "neutral":
                self._neutral_score,
        }

        expression, score = max(
            scores.items(),
            key=lambda item: item[1]
        )

        if (
            score
            <
            self._expression_threshold
        ):

            return "unknown"

        return expression

    # ========================================================
    # CONFIANZA
    # ========================================================

    def _calculate_confidence(
        self
    ) -> float:

        scores = [

            self._smile_score,

            self._frown_score,

            self._surprise_score,

            self._neutral_score,

            self._sleepy_score,

            self._tired_score,

            self._fatigued_score,

            self._yawn_score,
        ]

        return self._clamp(
            max(scores)
        )

    # ========================================================
    # SUAVIZADO
    # ========================================================

    def _smooth_blendshapes(
        self,
        current: dict[str, float],
    ) -> dict[str, float]:

        smoothed = {}

        for name, value in current.items():

            value = self._clamp(
                float(value)
            )

            # ------------------------------------------------
            # Primera observación
            # ------------------------------------------------

            if (
                name
                not in
                self._smoothed_scores
            ):

                self._smoothed_scores[
                    name
                ] = value

            # ------------------------------------------------
            # EMA
            # ------------------------------------------------

            else:

                previous = (
                    self._smoothed_scores[
                        name
                    ]
                )

                self._smoothed_scores[
                    name
                ] = (

                    self._smoothing_alpha
                    *
                    value

                    +

                    (
                        1.0
                        -
                        self._smoothing_alpha
                    )
                    *
                    previous
                )

            smoothed[name] = (
                self._smoothed_scores[
                    name
                ]
            )

        return smoothed

    # ========================================================
    # OBTENER SCORE
    # ========================================================

    def _get_score(
        self,
        name: str
    ) -> float:

        return float(
            self._blendshape_scores.get(
                name,
                0.0
            )
        )

    # ========================================================
    # GETTERS
    # ========================================================

    def get_expression(
        self
    ) -> str:

        return self._expression

    # --------------------------------------------------------

    def get_confidence(
        self
    ) -> float:

        return self._confidence

    # --------------------------------------------------------

    def get_smile_score(
        self
    ) -> float:

        return self._smile_score

    # --------------------------------------------------------

    def get_frown_score(
        self
    ) -> float:

        return self._frown_score

    # --------------------------------------------------------

    def get_surprise_score(
        self
    ) -> float:

        return self._surprise_score

    # --------------------------------------------------------

    def get_neutral_score(
        self
    ) -> float:

        return self._neutral_score

    # --------------------------------------------------------

    def get_sleepy_score(
        self
    ) -> float:

        return self._sleepy_score

    # --------------------------------------------------------

    def get_tired_score(
        self
    ) -> float:

        return self._tired_score

    # --------------------------------------------------------

    def get_fatigued_score(
        self
    ) -> float:

        return self._fatigued_score

    # --------------------------------------------------------

    def get_yawn_score(
        self
    ) -> float:

        return self._yawn_score

    # --------------------------------------------------------

    def get_blendshape_scores(
        self
    ) -> dict[str, float]:

        return (
            self._blendshape_scores.copy()
        )

    # --------------------------------------------------------

    def is_fatigue_expression(
        self
    ) -> bool:

        return self._is_fatigue_expression

    # --------------------------------------------------------

    def is_expression_valid(
        self
    ) -> bool:

        return self._expression_valid

    # --------------------------------------------------------

    def get_expression_threshold(
        self
    ) -> float:

        return self._expression_threshold

    # --------------------------------------------------------

    def get_fatigue_expressions(
        self
    ) -> set[str]:

        return (
            self._fatigue_expressions.copy()
        )

    # ========================================================
    # RESULT
    # ========================================================

    def get_result(
        self
    ) -> ExpressionResult:

        return ExpressionResult(

            expression=(
                self._expression
            ),

            confidence=(
                self._confidence
            ),

            smile_score=(
                self._smile_score
            ),

            frown_score=(
                self._frown_score
            ),

            surprise_score=(
                self._surprise_score
            ),

            neutral_score=(
                self._neutral_score
            ),

            sleepy_score=(
                self._sleepy_score
            ),

            tired_score=(
                self._tired_score
            ),

            fatigued_score=(
                self._fatigued_score
            ),

            yawn_score=(
                self._yawn_score
            ),

            is_fatigue_expression=(
                self._is_fatigue_expression
            ),

            expression_valid=(
                self._expression_valid
            ),

            blendshape_scores=(
                self._blendshape_scores.copy()
            ),
        )

    # ========================================================
    # CONFIGURACIÓN
    # ========================================================

    def set_expression_threshold(
        self,
        threshold: float
    ) -> None:

        self._validate_range(
            threshold,
            0.0,
            1.0,
            "expression_threshold"
        )

        self._expression_threshold = (
            threshold
        )

    # --------------------------------------------------------

    def set_fatigue_expressions(
        self,
        expressions: set[str]
    ) -> None:

        if not expressions:

            raise ValueError(
                "fatigue_expressions "
                "cannot be empty"
            )

        self._fatigue_expressions = {
            expression.lower()
            for expression in expressions
        }

    # ========================================================
    # VALIDACIÓN BLENDSHAPES
    # ========================================================

    @staticmethod
    def _validate_blendshapes(
        blendshapes: dict[str, float]
    ) -> None:

        if not isinstance(
            blendshapes,
            dict
        ):

            raise TypeError(
                "face_blendshapes must "
                "be a dictionary"
            )

        if not blendshapes:

            raise ValueError(
                "face_blendshapes cannot "
                "be empty"
            )

        for name, value in (
            blendshapes.items()
        ):

            if not isinstance(
                name,
                str
            ):

                raise TypeError(
                    "blendshape names must "
                    "be strings"
                )

            numeric_value = float(
                value
            )

            if not 0.0 <= numeric_value <= 1.0:

                raise ValueError(
                    f"Blendshape '{name}' "
                    "score must be between "
                    "0.0 and 1.0"
                )

    # ========================================================
    # VALIDACIÓN DE RANGO
    # ========================================================

    @staticmethod
    def _validate_range(
        value: float,
        minimum: float,
        maximum: float,
        name: str,
    ) -> None:

        if not minimum <= value <= maximum:

            raise ValueError(
                f"{name} must be between "
                f"{minimum} and {maximum}"
            )

    # ========================================================
    # VALIDACIÓN ENTEROS POSITIVOS
    # ========================================================

    @staticmethod
    def _validate_positive_int(
        value: int,
        name: str
    ) -> None:

        if value < 1:

            raise ValueError(
                f"{name} must be greater than 0"
            )

    # ========================================================
    # CLAMP
    # ========================================================

    @staticmethod
    def _clamp(
        value: float
    ) -> float:

        return max(
            0.0,
            min(
                1.0,
                float(value)
            )
        )
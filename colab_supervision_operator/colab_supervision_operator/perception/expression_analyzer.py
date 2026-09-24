#!/usr/bin/env python3

from dataclasses import dataclass, field
from typing import Optional


@dataclass
class ExpressionResult:
    expression: str
    confidence: float

    smile_score: float
    frown_score: float
    surprise_score: float
    neutral_score: float

    sleepy_score: float
    tired_score: float
    fatigued_score: float
    yawn_score: float

    is_fatigue_expression: bool
    expression_valid: bool

    blendshape_scores: dict[str, float] = field(default_factory=dict)


class ExpressionAnalyzer:
    """Classify facial expressions from MediaPipe face blendshapes."""

    def __init__(
        self,
        expression_threshold: float = 0.28,
        fatigue_expressions: Optional[set[str]] = None,
        smoothing_alpha: float = 0.35,
        sleepy_threshold: float = 0.35,
        tired_threshold: float = 0.35,
        fatigued_threshold: float = 0.45,
        yawn_threshold: float = 0.38,
        sleepy_frames: int = 6,
        tired_frames: int = 10,
        fatigued_frames: int = 20,
        yawn_frames: int = 8,
    ) -> None:

        self._expression_threshold = expression_threshold
        self._fatigue_expressions = {
            "sleepy", "tired", "fatigued", "yawn"
        } if fatigue_expressions is None else {
            item.lower() for item in fatigue_expressions
        }
        self._smoothing_alpha = smoothing_alpha
        self._sleepy_threshold = sleepy_threshold
        self._tired_threshold = tired_threshold
        self._fatigued_threshold = fatigued_threshold
        self._yawn_threshold = yawn_threshold
        self._sleepy_frames = sleepy_frames
        self._tired_frames = tired_frames
        self._fatigued_frames = fatigued_frames
        self._yawn_frames = yawn_frames

        if not 0.0 <= smoothing_alpha <= 1.0:
            raise ValueError("smoothing_alpha must be between 0 and 1")

        self.reset()

    def reset(self) -> None:
        self._smoothed_scores: dict[str, float] = {}
        self._sleepy_counter = 0
        self._tired_counter = 0
        self._fatigued_counter = 0
        self._yawn_counter = 0

        self._result = ExpressionResult(
            expression="unknown",
            confidence=0.0,
            smile_score=0.0,
            frown_score=0.0,
            surprise_score=0.0,
            neutral_score=1.0,
            sleepy_score=0.0,
            tired_score=0.0,
            fatigued_score=0.0,
            yawn_score=0.0,
            is_fatigue_expression=False,
            expression_valid=False,
            blendshape_scores={},
        )

    def update(self, face_blendshapes: dict[str, float]) -> ExpressionResult:
        if not isinstance(face_blendshapes, dict):
            raise TypeError("face_blendshapes must be a dictionary")

        scores = {
            "smile": self._score_smile(face_blendshapes),
            "frown": self._score_frown(face_blendshapes),
            "surprise": self._score_surprise(face_blendshapes),
            "sleepy": self._score_sleepy(face_blendshapes),
            "tired": self._score_tired(face_blendshapes),
            "fatigued": self._score_fatigued(face_blendshapes),
            "yawn": self._score_yawn(face_blendshapes),
        }

        for name, value in scores.items():
            previous = self._smoothed_scores.get(name, value)
            self._smoothed_scores[name] = (
                self._smoothing_alpha * value
                + (1.0 - self._smoothing_alpha) * previous
            )

        smile = self._smoothed_scores["smile"]
        frown = self._smoothed_scores["frown"]
        surprise = self._smoothed_scores["surprise"]
        sleepy = self._smoothed_scores["sleepy"]
        tired = self._smoothed_scores["tired"]
        fatigued = self._smoothed_scores["fatigued"]
        yawn = self._smoothed_scores["yawn"]

        self._sleepy_counter = self._update_counter(
            self._sleepy_counter, sleepy >= self._sleepy_threshold
        )
        self._tired_counter = self._update_counter(
            self._tired_counter, tired >= self._tired_threshold
        )
        self._fatigued_counter = self._update_counter(
            self._fatigued_counter, fatigued >= self._fatigued_threshold
        )
        self._yawn_counter = self._update_counter(
            self._yawn_counter, yawn >= self._yawn_threshold
        )

        candidates = {
            "smile": smile,
            "frown": frown,
            "surprise": surprise,
        }

        expression = "neutral"
        confidence = max(0.0, 1.0 - max(candidates.values()))

        if self._yawn_counter >= self._yawn_frames:
            expression = "yawn"
            confidence = yawn
        elif self._fatigued_counter >= self._fatigued_frames:
            expression = "fatigued"
            confidence = fatigued
        elif self._sleepy_counter >= self._sleepy_frames:
            expression = "sleepy"
            confidence = sleepy
        elif self._tired_counter >= self._tired_frames:
            expression = "tired"
            confidence = tired
        else:
            best = max(candidates, key=candidates.get)
            best_score = candidates[best]
            if best_score >= self._expression_threshold:
                expression = best
                confidence = best_score
            else:
                expression = "neutral"
                confidence = max(0.0, 1.0 - best_score)

        neutral_score = max(
            0.0,
            min(1.0, 1.0 - max(smile, frown, surprise, sleepy, tired, fatigued, yawn))
        )

        is_fatigue = expression in self._fatigue_expressions

        self._result = ExpressionResult(
            expression=expression,
            confidence=float(max(0.0, min(confidence, 1.0))),
            smile_score=float(smile),
            frown_score=float(frown),
            surprise_score=float(surprise),
            neutral_score=float(neutral_score),
            sleepy_score=float(sleepy),
            tired_score=float(tired),
            fatigued_score=float(fatigued),
            yawn_score=float(yawn),
            is_fatigue_expression=is_fatigue,
            expression_valid=True,
            blendshape_scores={
                key: float(value) for key, value in face_blendshapes.items()
            },
        )

        return self._result

    @staticmethod
    def _avg(values: list[float]) -> float:
        values = [max(0.0, min(1.0, float(v))) for v in values]
        return sum(values) / len(values) if values else 0.0

    def _get(self, bs: dict[str, float], name: str) -> float:
        return float(bs.get(name, 0.0))

    def _score_smile(self, bs: dict[str, float]) -> float:
        return self._avg([
            self._get(bs, "mouthSmileLeft"),
            self._get(bs, "mouthSmileRight"),
        ])

    def _score_frown(self, bs: dict[str, float]) -> float:
        left = self._get(bs, "mouthFrownLeft")
        right = self._get(bs, "mouthFrownRight")
        raw = 0.60 * self._avg([left, right]) + 0.40 * max(left, right)
        return 1.0 - (1.0 - max(0.0, min(1.0, raw))) ** 2

    def _score_surprise(self, bs: dict[str, float]) -> float:
        return (
            0.40 * self._get(bs, "jawOpen")
            + 0.35 * self._avg([
                self._get(bs, "eyeWideLeft"),
                self._get(bs, "eyeWideRight"),
            ])
            + 0.25 * self._get(bs, "browInnerUp")
        )

    def _ocular_fatigue(self, bs: dict[str, float]) -> float:
        eye_closure = self._avg([
            self._get(bs, "eyeBlinkLeft"),
            self._get(bs, "eyeBlinkRight"),
        ])
        eye_squint = self._avg([
            self._get(bs, "eyeSquintLeft"),
            self._get(bs, "eyeSquintRight"),
        ])
        return 0.60 * eye_closure + 0.40 * eye_squint

    def _score_sleepy(self, bs: dict[str, float]) -> float:
        return (
            0.65 * self._avg([
                self._get(bs, "eyeBlinkLeft"),
                self._get(bs, "eyeBlinkRight"),
            ])
            + 0.35 * self._avg([
                self._get(bs, "eyeSquintLeft"),
                self._get(bs, "eyeSquintRight"),
            ])
        )

    def _score_tired(self, bs: dict[str, float]) -> float:
        return (
            0.50 * self._avg([
                self._get(bs, "eyeSquintLeft"),
                self._get(bs, "eyeSquintRight"),
            ])
            + 0.30 * self._score_frown(bs)
            + 0.20 * self._avg([
                self._get(bs, "browDownLeft"),
                self._get(bs, "browDownRight"),
            ])
        )

    def _score_fatigued(self, bs: dict[str, float]) -> float:
        mouth_press = self._avg([
            self._get(bs, "mouthPressLeft"),
            self._get(bs, "mouthPressRight"),
        ])
        brow_down = self._avg([
            self._get(bs, "browDownLeft"),
            self._get(bs, "browDownRight"),
        ])
        return (
            0.40 * self._ocular_fatigue(bs)
            + 0.30 * self._score_frown(bs)
            + 0.15 * mouth_press
            + 0.15 * brow_down
        )

    def _score_yawn(self, bs: dict[str, float]) -> float:
        mouth_stretch = self._avg([
            self._get(bs, "mouthStretchLeft"),
            self._get(bs, "mouthStretchRight"),
        ])
        return 0.65 * self._get(bs, "jawOpen") + 0.35 * mouth_stretch

    @staticmethod
    def _update_counter(counter: int, condition: bool) -> int:
        return counter + 1 if condition else 0

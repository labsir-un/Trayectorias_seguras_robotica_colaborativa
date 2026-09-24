#!/usr/bin/env python3

from dataclasses import dataclass
from collections import deque
from typing import Optional


@dataclass
class FatigueResult:
    eye_openness: float
    blink_frequency: float
    facial_expression: str

    head_pitch: float
    head_nodding_score: float

    eye_openness_fatigued: bool
    blink_frequency_fatigued: bool
    facial_expression_fatigued: bool
    head_nodding_fatigued: bool

    fatigue_score: float
    fatigue_level: str


class FatigueAnalyzer:
    """Fuse blink, eye-openness, expression and head-nodding indicators."""

    def __init__(
        self,
        eye_openness_threshold: float = 0.30,
        minimum_blink_frequency: float = 5.0,
        fatigue_expressions: Optional[set[str]] = None,
        head_nod_pitch_change: float = 0.8,
        head_nod_min_angle: float = 8.0,
        head_nod_window_seconds: float = 5.0,
        head_nod_threshold: float = 0.40,
        head_nod_max_reversals: int = 4,
    ) -> None:

        self._eye_openness_threshold = eye_openness_threshold
        self._minimum_blink_frequency = minimum_blink_frequency
        self._fatigue_expressions = {
            "sleepy", "tired", "fatigued", "yawn"
        } if fatigue_expressions is None else {
            expression.lower() for expression in fatigue_expressions
        }

        self._head_nod_pitch_change = float(head_nod_pitch_change)
        self._head_nod_min_angle = float(head_nod_min_angle)
        self._head_nod_window_seconds = float(head_nod_window_seconds)
        self._head_nod_threshold = float(head_nod_threshold)
        self._head_nod_max_reversals = int(head_nod_max_reversals)

        self.reset()

    def reset(self) -> None:
        self._eye_openness = 0.0
        self._blink_frequency = 0.0
        self._facial_expression = "unknown"
        self._head_pitch = 0.0

        self._eye_openness_fatigued = False
        self._blink_frequency_fatigued = True
        self._facial_expression_fatigued = False
        self._head_nodding_fatigued = False

        self._head_nodding_score = 0.0
        self._fatigue_score = 0.0
        self._fatigue_level = "UNKNOWN"

        self._pitch_history: deque[tuple[float, float]] = deque()
        self._last_pitch: Optional[float] = None
        self._last_direction = 0
        self._last_extreme_pitch: Optional[float] = None
        self._reversal_times: deque[float] = deque()

    def update(
        self,
        eye_openness: float,
        blink_frequency: float,
        facial_expression: str,
        head_pitch: float = 0.0,
        timestamp: float = 0.0,
    ) -> FatigueResult:

        self._validate_observations(
            eye_openness,
            blink_frequency,
            facial_expression,
            head_pitch,
            timestamp,
        )

        self._eye_openness = float(eye_openness)
        self._blink_frequency = float(blink_frequency)
        self._facial_expression = facial_expression.lower()
        self._head_pitch = float(head_pitch)

        self._eye_openness_fatigued = (
            self._eye_openness < self._eye_openness_threshold
        )
        self._blink_frequency_fatigued = (
            self._blink_frequency < self._minimum_blink_frequency
        )
        self._facial_expression_fatigued = (
            self._facial_expression in self._fatigue_expressions
        )

        self._update_head_nodding(timestamp)

        self._fatigue_score = (
            0.35 * float(self._eye_openness_fatigued)
            + 0.30 * float(self._blink_frequency_fatigued)
            + 0.20 * float(self._facial_expression_fatigued)
            + 0.15 * self._head_nodding_score
        )

        self._fatigue_level = self._calculate_fatigue_level()

        return self.get_result()

    def _update_head_nodding(self, timestamp: float) -> None:
        self._pitch_history.append((timestamp, self._head_pitch))

        limit = timestamp - self._head_nod_window_seconds
        while self._pitch_history and self._pitch_history[0][0] < limit:
            self._pitch_history.popleft()

        while self._reversal_times and self._reversal_times[0] < limit:
            self._reversal_times.popleft()

        if self._last_pitch is None:
            self._last_pitch = self._head_pitch
            self._last_extreme_pitch = self._head_pitch
            return

        delta = self._head_pitch - self._last_pitch

        if abs(delta) >= self._head_nod_pitch_change:
            direction = 1 if delta > 0.0 else -1

            if self._last_direction == 0:
                self._last_direction = direction
                self._last_extreme_pitch = self._head_pitch

            elif direction != self._last_direction:
                excursion = 0.0
                if self._last_extreme_pitch is not None:
                    excursion = abs(self._head_pitch - self._last_extreme_pitch)

                if excursion >= self._head_nod_min_angle:
                    self._reversal_times.append(timestamp)

                self._last_direction = direction
                self._last_extreme_pitch = self._head_pitch

            else:
                if self._last_direction > 0:
                    if self._last_extreme_pitch is None:
                        self._last_extreme_pitch = self._head_pitch
                    else:
                        self._last_extreme_pitch = max(
                            self._last_extreme_pitch,
                            self._head_pitch,
                        )
                elif self._last_direction < 0:
                    if self._last_extreme_pitch is None:
                        self._last_extreme_pitch = self._head_pitch
                    else:
                        self._last_extreme_pitch = min(
                            self._last_extreme_pitch,
                            self._head_pitch,
                        )

        self._last_pitch = self._head_pitch

        self._head_nodding_score = min(
            1.0,
            len(self._reversal_times) / float(self._head_nod_max_reversals)
        )
        self._head_nodding_fatigued = (
            self._head_nodding_score >= self._head_nod_threshold
        )

    def _calculate_fatigue_level(self) -> str:
        if self._fatigue_score < 0.25:
            return "LOW"
        if self._fatigue_score < 0.50:
            return "MEDIUM"
        return "HIGH"

    def get_result(self) -> FatigueResult:
        return FatigueResult(
            eye_openness=self._eye_openness,
            blink_frequency=self._blink_frequency,
            facial_expression=self._facial_expression,
            head_pitch=self._head_pitch,
            head_nodding_score=self._head_nodding_score,
            eye_openness_fatigued=self._eye_openness_fatigued,
            blink_frequency_fatigued=self._blink_frequency_fatigued,
            facial_expression_fatigued=self._facial_expression_fatigued,
            head_nodding_fatigued=self._head_nodding_fatigued,
            fatigue_score=self._fatigue_score,
            fatigue_level=self._fatigue_level,
        )

    @staticmethod
    def _validate_observations(
        eye_openness: float,
        blink_frequency: float,
        facial_expression: str,
        head_pitch: float,
        timestamp: float,
    ) -> None:
        if not 0.0 <= eye_openness <= 1.0:
            raise ValueError("eye_openness must be between 0 and 1")
        if blink_frequency < 0.0:
            raise ValueError("blink_frequency cannot be negative")
        if not isinstance(facial_expression, str):
            raise TypeError("facial_expression must be a string")
        if not isinstance(head_pitch, (int, float)):
            raise TypeError("head_pitch must be numeric")
        if timestamp < 0.0:
            raise ValueError("timestamp cannot be negative")

#!/usr/bin/env python3

from dataclasses import dataclass
from collections import deque
from typing import Optional

import numpy as np


@dataclass
class BlinkResult:
    left_ear: float
    right_ear: float
    mean_ear: float
    eye_openness: float

    left_eye_closed: bool
    right_eye_closed: bool
    eyes_closed: bool

    blink_detected: bool
    blink_duration: float
    blink_frequency: float
    total_blinks: int

    valid: bool


class BlinkAnalyzer:
    """Detect blinks from the six facial landmarks of each eye."""

    def __init__(
        self,
        eye_closed_threshold: float = 0.23,
        min_closed_frames: int = 1,
        max_closed_frames: int = 20,
        fps: float = 30.0,
        frequency_window_seconds: float = 60.0,
        openness_min_ear: float = 0.10,
        openness_max_ear: float = 0.35,
    ) -> None:

        if not 0.0 < eye_closed_threshold < 1.0:
            raise ValueError("eye_closed_threshold must be between 0 and 1")
        if min_closed_frames < 1:
            raise ValueError("min_closed_frames must be >= 1")
        if max_closed_frames < min_closed_frames:
            raise ValueError("max_closed_frames must be >= min_closed_frames")
        if fps <= 0.0:
            raise ValueError("fps must be > 0")
        if frequency_window_seconds <= 0.0:
            raise ValueError("frequency_window_seconds must be > 0")
        if openness_min_ear >= openness_max_ear:
            raise ValueError("openness_min_ear must be < openness_max_ear")

        self._eye_closed_threshold = float(eye_closed_threshold)
        self._min_closed_frames = int(min_closed_frames)
        self._max_closed_frames = int(max_closed_frames)
        self._fps = float(fps)
        self._frequency_window_seconds = float(frequency_window_seconds)
        self._openness_min_ear = float(openness_min_ear)
        self._openness_max_ear = float(openness_max_ear)

        self.reset()

    def reset(self) -> None:
        self._left_ear = 0.0
        self._right_ear = 0.0
        self._mean_ear = 0.0
        self._eye_openness = 0.0

        self._left_eye_closed = False
        self._right_eye_closed = False
        self._eyes_closed = False

        self._closed_frame_count = 0
        self._blink_detected = False
        self._blink_duration = 0.0

        self._blink_timestamps: deque[float] = deque()
        self._elapsed_time = 0.0
        self._blink_frequency = 0.0

    def update(self, left_eye_points, right_eye_points) -> BlinkResult:
        self._validate_eye_points(left_eye_points, "left_eye_points")
        self._validate_eye_points(right_eye_points, "right_eye_points")

        self._elapsed_time += 1.0 / self._fps
        self._blink_detected = False

        self._left_ear = self._calculate_ear(left_eye_points)
        self._right_ear = self._calculate_ear(right_eye_points)
        self._mean_ear = (self._left_ear + self._right_ear) / 2.0

        openness = (
            (self._mean_ear - self._openness_min_ear)
            / (self._openness_max_ear - self._openness_min_ear)
        )
        self._eye_openness = float(np.clip(openness, 0.0, 1.0))

        self._left_eye_closed = self._left_ear < self._eye_closed_threshold
        self._right_eye_closed = self._right_ear < self._eye_closed_threshold
        self._eyes_closed = self._left_eye_closed and self._right_eye_closed

        if self._eyes_closed:
            self._closed_frame_count += 1
            self._blink_duration = self._closed_frame_count / self._fps
        else:
            if self._closed_frame_count > 0:
                duration = self._closed_frame_count / self._fps

                if (
                    self._min_closed_frames
                    <= self._closed_frame_count
                    <= self._max_closed_frames
                ):
                    self._blink_detected = True
                    self._blink_duration = duration
                    self._blink_timestamps.append(self._elapsed_time)
                else:
                    self._blink_duration = 0.0

                self._closed_frame_count = 0
            else:
                self._blink_duration = 0.0

        self._remove_old_blinks()
        self._blink_frequency = self._calculate_frequency()

        return self.get_result()

    def get_result(self) -> BlinkResult:
        return BlinkResult(
            left_ear=self._left_ear,
            right_ear=self._right_ear,
            mean_ear=self._mean_ear,
            eye_openness=self._eye_openness,
            left_eye_closed=self._left_eye_closed,
            right_eye_closed=self._right_eye_closed,
            eyes_closed=self._eyes_closed,
            blink_detected=self._blink_detected,
            blink_duration=self._blink_duration,
            blink_frequency=self._blink_frequency,
            total_blinks=len(self._blink_timestamps),
            valid=True,
        )

    @staticmethod
    def _validate_eye_points(points, name: str) -> None:
        array = np.asarray(points, dtype=np.float32)
        if array.shape != (6, 2):
            raise ValueError(f"{name} must have shape (6, 2)")
        if not np.isfinite(array).all():
            raise ValueError(f"{name} contains invalid values")

    @staticmethod
    def _calculate_ear(points) -> float:
        points = np.asarray(points, dtype=np.float32)
        p1, p2, p3, p4, p5, p6 = points

        vertical_1 = np.linalg.norm(p2 - p6)
        vertical_2 = np.linalg.norm(p3 - p5)
        horizontal = np.linalg.norm(p1 - p4)

        if horizontal <= 1e-6:
            return 0.0

        return float((vertical_1 + vertical_2) / (2.0 * horizontal))

    def _remove_old_blinks(self) -> None:
        limit = self._elapsed_time - self._frequency_window_seconds
        while self._blink_timestamps and self._blink_timestamps[0] < limit:
            self._blink_timestamps.popleft()

    def _calculate_frequency(self) -> float:
        return (
            len(self._blink_timestamps)
            / self._frequency_window_seconds
            * 60.0
        )

    def get_eye_closed_threshold(self) -> float:
        return self._eye_closed_threshold

    def get_blink_frequency(self) -> float:
        return self._blink_frequency

    def get_total_blinks(self) -> int:
        return len(self._blink_timestamps)

    def is_blink_detected(self) -> bool:
        return self._blink_detected

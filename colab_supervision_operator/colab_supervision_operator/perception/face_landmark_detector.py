#!/usr/bin/env python3

from dataclasses import dataclass
from pathlib import Path

import cv2
import mediapipe as mp
import numpy as np

from mediapipe.tasks import python
from mediapipe.tasks.python import vision


@dataclass
class FaceLandmarkResult:
    face_detected: bool
    landmarks: np.ndarray | None
    bbox: tuple[int, int, int, int] | None
    head_pose_points: np.ndarray | None
    left_eye_points: np.ndarray | None
    right_eye_points: np.ndarray | None
    mouth_points: np.ndarray | None
    nose_point: tuple[int, int] | None
    face_blendshapes: dict[str, float] | None


class FaceLandmarkDetector:

    NOSE_INDEX = 1
    CHIN_INDEX = 152
    LEFT_EYE_CENTER_INDEX = 468
    RIGHT_EYE_CENTER_INDEX = 473
    LEFT_MOUTH_INDEX = 61
    RIGHT_MOUTH_INDEX = 291

    LEFT_EYE = [33, 160, 158, 133, 153, 144]
    RIGHT_EYE = [362, 385, 387, 263, 373, 380]
    MOUTH = [61, 146, 91, 181, 84, 17, 314, 405, 321, 375, 291]

    def __init__(
        self,
        model_path: str,
        min_detection_confidence: float = 0.5,
        min_tracking_confidence: float = 0.5,
    ) -> None:

        self._model_path = Path(model_path)
        if not self._model_path.exists():
            raise FileNotFoundError(
                f"No se encontró el modelo Face Landmarker: {self._model_path}"
            )

        base_options = python.BaseOptions(
            model_asset_path=str(self._model_path)
        )

        options = vision.FaceLandmarkerOptions(
            base_options=base_options,
            running_mode=vision.RunningMode.VIDEO,
            num_faces=1,
            min_face_detection_confidence=min_detection_confidence,
            min_face_presence_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
            output_face_blendshapes=True,
            output_facial_transformation_matrixes=False,
        )

        self._detector = vision.FaceLandmarker.create_from_options(options)
        self._timestamp_ms = 0

    def update(self, image: np.ndarray) -> FaceLandmarkResult:
        if image is None or image.size == 0:
            return self._empty_result()

        height, width = image.shape[:2]
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        mp_image = mp.Image(
            image_format=mp.ImageFormat.SRGB,
            data=rgb_image,
        )

        self._timestamp_ms += 33

        result = self._detector.detect_for_video(
            mp_image,
            self._timestamp_ms,
        )

        if not result.face_landmarks:
            return self._empty_result()

        face = result.face_landmarks[0]

        landmarks = np.array(
            [[landmark.x * width, landmark.y * height] for landmark in face],
            dtype=np.float32,
        )

        x_min = max(0, int(np.min(landmarks[:, 0])))
        y_min = max(0, int(np.min(landmarks[:, 1])))
        x_max = min(width - 1, int(np.max(landmarks[:, 0])))
        y_max = min(height - 1, int(np.max(landmarks[:, 1])))

        head_pose_points = np.array(
            [
                landmarks[self.NOSE_INDEX],
                landmarks[self.CHIN_INDEX],
                landmarks[self.LEFT_EYE_CENTER_INDEX],
                landmarks[self.RIGHT_EYE_CENTER_INDEX],
                landmarks[self.LEFT_MOUTH_INDEX],
                landmarks[self.RIGHT_MOUTH_INDEX],
            ],
            dtype=np.float32,
        )

        left_eye_points = landmarks[self.LEFT_EYE]
        right_eye_points = landmarks[self.RIGHT_EYE]
        mouth_points = landmarks[self.MOUTH]

        nose = landmarks[self.NOSE_INDEX]
        nose_point = (int(nose[0]), int(nose[1]))

        face_blendshapes: dict[str, float] = {}
        if result.face_blendshapes:
            for category in result.face_blendshapes[0]:
                name = getattr(category, "category_name", None)
                score = getattr(category, "score", None)
                if name is not None and score is not None:
                    face_blendshapes[str(name)] = float(score)

        return FaceLandmarkResult(
            face_detected=True,
            landmarks=landmarks,
            bbox=(x_min, y_min, x_max, y_max),
            head_pose_points=head_pose_points,
            left_eye_points=left_eye_points,
            right_eye_points=right_eye_points,
            mouth_points=mouth_points,
            nose_point=nose_point,
            face_blendshapes=face_blendshapes,
        )

    def draw_landmarks(
        self,
        image: np.ndarray,
        result: FaceLandmarkResult,
    ) -> np.ndarray:
        if not result.face_detected or result.landmarks is None:
            return image

        output = image.copy()

        for point in result.landmarks:
            x = int(round(float(point[0])))
            y = int(round(float(point[1])))
            if 0 <= x < output.shape[1] and 0 <= y < output.shape[0]:
                cv2.circle(output, (x, y), 1, (0, 255, 0), -1)

        if result.head_pose_points is not None:
            for point in result.head_pose_points:
                x = int(round(float(point[0])))
                y = int(round(float(point[1])))
                if 0 <= x < output.shape[1] and 0 <= y < output.shape[0]:
                    cv2.circle(output, (x, y), 3, (255, 0, 0), -1)

        if result.nose_point is not None:
            cv2.circle(
                output,
                (
                    int(round(float(result.nose_point[0]))),
                    int(round(float(result.nose_point[1]))),
                ),
                4,
                (0, 0, 255),
                -1,
            )

        return output

    @staticmethod
    def _empty_result() -> FaceLandmarkResult:
        return FaceLandmarkResult(
            face_detected=False,
            landmarks=None,
            bbox=None,
            head_pose_points=None,
            left_eye_points=None,
            right_eye_points=None,
            mouth_points=None,
            nose_point=None,
            face_blendshapes=None,
        )

    def close(self) -> None:
        if getattr(self, "_detector", None) is not None:
            self._detector.close()
            self._detector = None

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass

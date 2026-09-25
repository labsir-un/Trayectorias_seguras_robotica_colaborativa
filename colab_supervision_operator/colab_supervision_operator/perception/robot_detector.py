#!/usr/bin/env python3

"""
RGB robot detector for colab_supervision_operator.

The detector uses a configurable Ultralytics YOLO model to identify
the robot in the RealSense RGB image.

The standard COCO YOLO model does not contain an industrial robot
class. This detector therefore expects an application-specific
model trained for the ABB IRB 140 (or equivalent robot representation).
"""

from dataclasses import dataclass
from typing import Optional

import numpy as np
from ultralytics import YOLO


@dataclass
class RobotDetection:
    """Information associated with one robot detection."""

    bbox: tuple[float, float, float, float]
    confidence: float
    class_id: int
    class_name: str

    @property
    def center_x(self) -> float:
        return (self.bbox[0] + self.bbox[2]) / 2.0

    @property
    def center_y(self) -> float:
        return (self.bbox[1] + self.bbox[3]) / 2.0


class RobotDetector:
    """Detect the robot in an RGB image using a dedicated YOLO model."""

    def __init__(
        self,
        model_path: str,
        confidence_threshold: float = 0.50,
        image_size: int = 640,
        device: Optional[str] = None,
        robot_class_id: int = 0,
    ) -> None:

        if not model_path:
            raise ValueError(
                "robot model path is empty. "
                "Configure an application-specific YOLO model."
            )

        self._model = YOLO(model_path)
        self._confidence_threshold = float(confidence_threshold)
        self._image_size = int(image_size)
        self._device = device
        self._robot_class_id = int(robot_class_id)

    def detect(self, image: np.ndarray) -> list[RobotDetection]:
        """Detect robots in one RGB image."""

        if not isinstance(image, np.ndarray):
            raise TypeError("image must be a numpy array")

        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError("image must have shape HxWx3")

        results = self._model.predict(
            source=image,
            conf=self._confidence_threshold,
            imgsz=self._image_size,
            device=self._device,
            verbose=False,
        )

        detections: list[RobotDetection] = []

        for result in results:
            if result.boxes is None:
                continue

            for box in result.boxes:
                class_id = int(box.cls[0].item())
                confidence = float(box.conf[0].item())

                if class_id != self._robot_class_id:
                    continue

                x1, y1, x2, y2 = (
                    float(value) for value in box.xyxy[0].tolist()
                )

                names = result.names
                if isinstance(names, dict):
                    class_name = str(
                        names.get(class_id, f"class_{class_id}")
                    )
                else:
                    class_name = str(
                        names[class_id]
                        if 0 <= class_id < len(names)
                        else f"class_{class_id}"
                    )

                detections.append(
                    RobotDetection(
                        bbox=(x1, y1, x2, y2),
                        confidence=confidence,
                        class_id=class_id,
                        class_name=class_name,
                    )
                )

        detections.sort(
            key=lambda detection: detection.confidence,
            reverse=True,
        )

        return detections

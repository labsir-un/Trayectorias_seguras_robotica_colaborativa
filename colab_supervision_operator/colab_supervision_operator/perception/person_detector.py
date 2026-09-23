#!/usr/bin/env python3

"""
person_detector.py

Detector de personas basado en Ultralytics YOLO.

Responsabilidad:
    - Recibir una imagen BGR proveniente de OpenCV.
    - Ejecutar detección mediante YOLO.
    - Filtrar únicamente la clase "person".
    - Generar bounding boxes y confianza.
    - Entregar las detecciones en el formato esperado
      por PersonTracker.

Formato de salida:

    [
        {
            "track_id": int,
            "bbox": (x1, y1, x2, y2),
            "confidence": float
        }
    ]

Este módulo NO realiza:
    - Seguimiento temporal.
    - Selección del operador principal.
    - Detección facial.
    - Análisis de fatiga.
"""


from typing import Optional

import cv2
import numpy as np

from ultralytics import YOLO


class PersonDetector:
    """
    Detector de personas utilizando YOLO.
    """

    # Clase "person" del dataset COCO.
    PERSON_CLASS_ID = 0

    def __init__(
        self,
        model_path: str = "yolo26n.pt",
        confidence_threshold: float = 0.5,
        image_size: int = 640,
        device: Optional[str] = None,
    ):
        """
        Parameters
        ----------
        model_path : str
            Ruta al modelo YOLO.

        confidence_threshold : float
            Confianza mínima para aceptar una detección.

        image_size : int
            Tamaño utilizado por YOLO durante la inferencia.

        device : str, optional
            Dispositivo de inferencia.

            Ejemplos:
                "cpu"
                "0"

            Si es None, Ultralytics selecciona automáticamente.
        """

        self._model_path = model_path
        self._confidence_threshold = confidence_threshold
        self._image_size = image_size
        self._device = device

        # ====================================================
        # CARGAR MODELO
        # ====================================================

        self._model = YOLO(
            self._model_path
        )

        # ====================================================
        # ID DE DETECCIÓN
        # ====================================================

        self._next_detection_id = 0

    # ========================================================
    # API PRINCIPAL
    # ========================================================

    def update(
        self,
        image: np.ndarray,
    ) -> list[dict]:
        """
        Detecta personas en una imagen.

        Parameters
        ----------
        image : np.ndarray
            Imagen BGR proveniente de OpenCV.

        Returns
        -------
        list[dict]
            Detecciones de personas.
        """

        if image is None:
            return []

        if image.size == 0:
            return []

        # ====================================================
        # INFERENCIA
        # ====================================================

        try:

            results = self._model.predict(
                source=image,
                conf=self._confidence_threshold,
                imgsz=self._image_size,
                device=self._device,
                verbose=False,
            )

        except Exception as error:

            print(
                f"[PersonDetector] Error durante "
                f"inferencia YOLO: {error}"
            )

            return []

        # ====================================================
        # PROCESAR RESULTADOS
        # ====================================================

        detections = []

        if results is None:
            return detections

        if len(results) == 0:
            return detections

        result = results[0]

        if result.boxes is None:
            return detections

        # ----------------------------------------------------
        # Cajas
        # ----------------------------------------------------

        boxes = result.boxes

        xyxy = boxes.xyxy.cpu().numpy()
        confidences = boxes.conf.cpu().numpy()
        classes = boxes.cls.cpu().numpy()

        # ====================================================
        # FILTRAR PERSONAS
        # ====================================================

        for bbox, confidence, class_id in zip(
            xyxy,
            confidences,
            classes,
        ):

            class_id = int(class_id)

            # ------------------------------------------------
            # Solo clase PERSON
            # ------------------------------------------------

            if class_id != self.PERSON_CLASS_ID:
                continue

            confidence = float(
                confidence
            )

            # ------------------------------------------------
            # Coordenadas
            # ------------------------------------------------

            x1 = int(bbox[0])
            y1 = int(bbox[1])
            x2 = int(bbox[2])
            y2 = int(bbox[3])

            # ------------------------------------------------
            # Validar bounding box
            # ------------------------------------------------

            if x2 <= x1:
                continue

            if y2 <= y1:
                continue

            # ------------------------------------------------
            # Crear detección
            # ------------------------------------------------

            detection = {
                "track_id": self._next_detection_id,
                "bbox": (
                    x1,
                    y1,
                    x2,
                    y2,
                ),
                "confidence": confidence,
            }

            self._next_detection_id += 1

            detections.append(
                detection
            )

        return detections

    # ========================================================
    # VISUALIZACIÓN
    # ========================================================

    def draw_detections(
        self,
        image: np.ndarray,
        detections: list[dict],
    ) -> np.ndarray:
        """
        Dibuja las detecciones YOLO sobre la imagen.

        Esta función es principalmente para depuración.
        """

        output = image.copy()

        for detection in detections:

            x1, y1, x2, y2 = detection["bbox"]

            confidence = detection["confidence"]

            # ------------------------------------------------
            # Bounding box
            # ------------------------------------------------

            cv2.rectangle(
                output,
                (x1, y1),
                (x2, y2),
                (255, 0, 0),
                2,
            )

            # ------------------------------------------------
            # Etiqueta
            # ------------------------------------------------

            label = (
                f"Person "
                f"{confidence:.2f}"
            )

            cv2.putText(
                output,
                label,
                (x1, max(y1 - 10, 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 0, 0),
                2,
            )

        return output
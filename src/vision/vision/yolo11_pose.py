"""Ultralytics YOLO11n-pose adapter, isolated from the MediaPipe backend."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable


@dataclass(frozen=True)
class YoloPoseLandmark:
    """Normalized image keypoint compatible with the shared RGB-D pipeline."""

    x: float
    y: float
    visibility: float


def fuse_face_keypoints(
    keypoints: Iterable[YoloPoseLandmark], confidence_threshold: float
) -> YoloPoseLandmark | None:
    """Fuse any reliable nose/eye/ear points into one robust head location."""

    valid = [
        point for point in keypoints
        if math.isfinite(point.x) and math.isfinite(point.y)
        and math.isfinite(point.visibility)
        and point.visibility >= confidence_threshold
    ]
    if not valid:
        return None
    weight_sum = sum(point.visibility for point in valid)
    return YoloPoseLandmark(
        x=sum(point.x * point.visibility for point in valid) / weight_sum,
        y=sum(point.y * point.visibility for point in valid) / weight_sum,
        # A single visible ear must be sufficient when a mask/blanket hides
        # other facial points. The pipeline still applies this confidence.
        visibility=max(point.visibility for point in valid),
    )


class Yolo11nPoseDetector:
    """Return shoulders, fused head, and hips from a COCO-order YOLO pose."""

    NOSE = 0
    LEFT_EYE = 1
    RIGHT_EYE = 2
    LEFT_EAR = 3
    RIGHT_EAR = 4
    LEFT_SHOULDER = 5
    RIGHT_SHOULDER = 6
    LEFT_HIP = 11
    RIGHT_HIP = 12
    FACE_INDICES = (NOSE, LEFT_EYE, RIGHT_EYE, LEFT_EAR, RIGHT_EAR)

    def __init__(
        self,
        model_path: str = "yolo11n-pose.pt",
        person_confidence: float = 0.45,
        keypoint_confidence: float = 0.50,
        device: str = "",
    ) -> None:
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "YOLO11n-pose backend selected, but 'ultralytics' is not installed. "
                "Install it in the ROS Python environment before starting this node."
            ) from exc
        self._model = YOLO(model_path)
        self._person_confidence = person_confidence
        self._keypoint_confidence = keypoint_confidence
        self._device = device
        self._last_result = None

    @staticmethod
    def _landmark(xyn, confidence, index: int) -> YoloPoseLandmark:
        return YoloPoseLandmark(
            float(xyn[index][0]), float(xyn[index][1]), float(confidence[index]))

    def detect(self, rgb_image):
        predict_args = {
            "source": rgb_image,
            "conf": self._person_confidence,
            "verbose": False,
        }
        if self._device:
            predict_args["device"] = self._device
        results = self._model.predict(**predict_args)
        self._last_result = results[0] if results else None
        result = self._last_result
        if result is None or result.keypoints is None or len(result.keypoints) == 0:
            return None

        normalized = result.keypoints.xyn.detach().cpu().numpy()
        confidence_tensor = result.keypoints.conf
        if confidence_tensor is None:
            confidences = [[1.0] * len(person) for person in normalized]
        else:
            confidences = confidence_tensor.detach().cpu().numpy()

        box_confidences = None
        if result.boxes is not None and result.boxes.conf is not None:
            box_confidences = result.boxes.conf.detach().cpu().numpy()

        # Prefer the person with reliable shoulders and hips. This prevents a
        # high-confidence partial bystander from replacing the target body.
        best_index, best_score = None, -math.inf
        required = (self.LEFT_SHOULDER, self.RIGHT_SHOULDER, self.LEFT_HIP, self.RIGHT_HIP)
        for index, person_confidences in enumerate(confidences):
            if len(person_confidences) <= self.RIGHT_HIP:
                continue
            core_score = min(float(person_confidences[key]) for key in required)
            box_score = float(box_confidences[index]) if box_confidences is not None else 1.0
            score = core_score + 0.25 * box_score
            if score > best_score:
                best_index, best_score = index, score
        if best_index is None:
            return None

        xyn, confidence = normalized[best_index], confidences[best_index]
        face = [self._landmark(xyn, confidence, index) for index in self.FACE_INDICES]
        head = fuse_face_keypoints(face, self._keypoint_confidence)
        if head is None:
            return None
        return (
            self._landmark(xyn, confidence, self.LEFT_SHOULDER),
            self._landmark(xyn, confidence, self.RIGHT_SHOULDER),
            head,
            self._landmark(xyn, confidence, self.LEFT_HIP),
            self._landmark(xyn, confidence, self.RIGHT_HIP),
        )

    def draw_landmarks(self, image) -> None:
        if self._last_result is None:
            return
        annotated = self._last_result.plot()
        if annotated.shape == image.shape:
            image[:] = annotated

    def close(self) -> None:
        """Ultralytics owns no persistent camera resource here."""


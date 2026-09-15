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


@dataclass(frozen=True)
class PoseLayout:
    """Keypoint indices for one supported YOLO pose skeleton."""

    name: str
    keypoint_count: int
    left_eye: int
    right_eye: int
    mouth: int
    left_shoulder: int
    right_shoulder: int
    left_hip: int
    right_hip: int
    face_indices: tuple[int, ...]
    chest: int | None = None


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
    """Adapter for either the cart's custom or the standard COCO YOLO pose model.

    The skeleton is selected from the loaded model's keypoint count, not from
    the file name.  This lets ``yolo11n-pose.pt`` (17 COCO keypoints) and the
    cart's ``best.pt`` (8 custom keypoints) share the same ROS launch path.
    """

    # best.pt를 바탕으로 keypoint 인덱스 정의 
    CUSTOM_LAYOUT = PoseLayout(
        name="custom_cart_8",
        keypoint_count=8,
        left_eye=0, right_eye=1, mouth=2,
        left_shoulder=3, right_shoulder=4,
        left_hip=5, right_hip=6,
        face_indices=(0, 1, 2),
        chest=7,
    )
    COCO_LAYOUT = PoseLayout(
        name="coco_17",
        keypoint_count=17,
        mouth=0, left_eye=1, right_eye=2,
        left_shoulder=5, right_shoulder=6,
        left_hip=11, right_hip=12,
        face_indices=(0, 1, 2, 3, 4),
    )

    @classmethod
    def layout_for_keypoint_count(cls, keypoint_count: int) -> PoseLayout:
        if keypoint_count == cls.CUSTOM_LAYOUT.keypoint_count:
            return cls.CUSTOM_LAYOUT
        if keypoint_count == cls.COCO_LAYOUT.keypoint_count:
            return cls.COCO_LAYOUT
        raise ValueError(
            "Unsupported YOLO pose skeleton: expected 8 custom cart keypoints "
            "or 17 COCO keypoints, got "
            f"{keypoint_count}."
        )

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
        model_keypoint_shape = getattr(getattr(self._model, "model", None), "kpt_shape", None)
        if model_keypoint_shape is None:
            raise ValueError("Could not determine the loaded YOLO pose model's keypoint shape.")
        self._layout = self.layout_for_keypoint_count(int(model_keypoint_shape[0]))
        self._person_confidence = person_confidence
        self._keypoint_confidence = keypoint_confidence
        self._device = device
        self._last_result = None

    @staticmethod
    def _landmark(xyn, confidence, index: int) -> YoloPoseLandmark:
        return YoloPoseLandmark(
            float(xyn[index][0]), float(xyn[index][1]), float(confidence[index]))

    @staticmethod
    def _estimated_chest(
        left_shoulder: YoloPoseLandmark,
        right_shoulder: YoloPoseLandmark,
        left_hip: YoloPoseLandmark,
        right_hip: YoloPoseLandmark,
    ) -> YoloPoseLandmark:
        """Estimate a chest point for COCO, which has no chest keypoint.

        The ROS body-axis validation consumes a face, chest, and pelvis line.
        The custom model supplies chest directly; for the standard 17-point
        COCO model, use the midpoint between shoulder and hip centres.
        """
        shoulder_x = (left_shoulder.x + right_shoulder.x) / 2.0
        shoulder_y = (left_shoulder.y + right_shoulder.y) / 2.0
        hip_x = (left_hip.x + right_hip.x) / 2.0
        hip_y = (left_hip.y + right_hip.y) / 2.0
        return YoloPoseLandmark(
            x=(shoulder_x + hip_x) / 2.0,
            y=(shoulder_y + hip_y) / 2.0,
            visibility=min(
                left_shoulder.visibility,
                right_shoulder.visibility,
                left_hip.visibility,
                right_hip.visibility,
            ),
        )

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

        # Prefer a complete body. Shoulders and hips identify a usable body;
        # face points are evaluated separately because ears can be occluded in
        # a valid COCO pose.
        best_index, best_score = None, -math.inf
        layout = self._layout
        required = (
            layout.left_shoulder, layout.right_shoulder,
            layout.left_hip, layout.right_hip,
        )
        for index, person_confidences in enumerate(confidences):
            if len(person_confidences) < layout.keypoint_count:
                continue
            core_score = min(float(person_confidences[key]) for key in required)
            box_score = float(box_confidences[index]) if box_confidences is not None else 1.0
            score = core_score + 0.25 * box_score
            if score > best_score:
                best_index, best_score = index, score
        if best_index is None:
            return None

        xyn, confidence = normalized[best_index], confidences[best_index]
        eyes = [self._landmark(xyn, confidence, index) for index in (layout.left_eye, layout.right_eye)]
        if min(p.visibility for p in eyes) < self._keypoint_confidence:
            return None
        head = YoloPoseLandmark((eyes[0].x+eyes[1].x)/2, (eyes[0].y+eyes[1].y)/2,
                               min(p.visibility for p in eyes))
        left_shoulder = self._landmark(xyn, confidence, layout.left_shoulder)
        right_shoulder = self._landmark(xyn, confidence, layout.right_shoulder)
        left_hip = self._landmark(xyn, confidence, layout.left_hip)
        right_hip = self._landmark(xyn, confidence, layout.right_hip)
        chest = (
            self._landmark(xyn, confidence, layout.chest)
            if layout.chest is not None
            else self._estimated_chest(left_shoulder, right_shoulder, left_hip, right_hip)
        )
        # Keep one stable ROS contract for both models:
        # left shoulder, right shoulder, head, left hip, right hip, chest.
        # Extra eye points let RGB-D deproject each eye before taking its 3-D midpoint.
        return left_shoulder, right_shoulder, head, left_hip, right_hip, chest, *eyes

    def draw_landmarks(self, image) -> None:
        if self._last_result is None:
            return
        annotated = self._last_result.plot()
        if annotated.shape == image.shape:
            # ``detect`` receives RGB (the D435/OpenCV frame is converted
            # before inference), and Ultralytics plots on that RGB image.
            # ``image`` is subsequently displayed by cv2.imshow, which expects
            # BGR.  Convert only this preview result back to BGR; inference and
            # all landmark coordinates remain unchanged.
            import cv2
            image[:] = cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR)

    def close(self) -> None:
        """Ultralytics owns no persistent camera resource here."""

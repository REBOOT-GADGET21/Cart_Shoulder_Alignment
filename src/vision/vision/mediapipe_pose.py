"""Thin MediaPipe wrapper kept separate from ROS and RealSense plumbing."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class MediaPipeFusedLandmark:
    x: float
    y: float
    visibility: float


def fuse_mediapipe_face_keypoints(keypoints, confidence_threshold: float):
    """Fuse MediaPipe nose, central eyes, and ears without YOLO dependencies."""

    valid = [
        point for point in keypoints
        if math.isfinite(point.x) and math.isfinite(point.y)
        and math.isfinite(point.visibility)
        and point.visibility >= confidence_threshold
    ]
    if not valid:
        return None
    weight_sum = sum(point.visibility for point in valid)
    return MediaPipeFusedLandmark(
        sum(point.x * point.visibility for point in valid) / weight_sum,
        sum(point.y * point.visibility for point in valid) / weight_sum,
        max(point.visibility for point in valid),
    )


class MediaPipePoseDetector:
    # MediaPipe Pose has 33 landmarks. These indices deliberately differ from
    # the COCO 17-keypoint order used only inside yolo11_pose.py.
    NOSE = 0
    LEFT_EYE = 2
    RIGHT_EYE = 5
    LEFT_EAR = 7
    RIGHT_EAR = 8
    LEFT_SHOULDER = 11
    RIGHT_SHOULDER = 12
    LEFT_HIP = 23
    RIGHT_HIP = 24
    FACE_INDICES = (NOSE, LEFT_EYE, RIGHT_EYE, LEFT_EAR, RIGHT_EAR)

    def __init__(
        self,
        detection_confidence: float = 0.5,
        tracking_confidence: float = 0.5,
        face_confidence: float = 0.5,
    ):
        # This explicit import matches the installed MediaPipe 0.10.x setup
        # already used by the MediaPipe camera preview; some distributions do not expose
        # ``mediapipe.solutions`` at the top level.
        from mediapipe.python.solutions import pose as mp_pose
        self._pose_module = mp_pose
        self._pose = self._pose_module.Pose(
            static_image_mode=False, model_complexity=1, smooth_landmarks=True,
            min_detection_confidence=detection_confidence,
            min_tracking_confidence=tracking_confidence)
        self._last_pose_landmarks = None
        self._face_confidence = face_confidence

    def detect(self, rgb_image):
        result = self._pose.process(rgb_image)
        self._last_pose_landmarks = result.pose_landmarks
        if not result.pose_landmarks:
            return None
        landmarks = result.pose_landmarks.landmark
        head = fuse_mediapipe_face_keypoints(
            (landmarks[index] for index in self.FACE_INDICES), self._face_confidence)
        if head is None:
            return None
        # The controller uses shoulders plus a head/pelvis body axis to select
        # the head-side normal. Keep the order stable for /shoulder_line.
        left, right = landmarks[self.LEFT_SHOULDER], landmarks[self.RIGHT_SHOULDER]
        chest = MediaPipeFusedLandmark((left.x+right.x)/2, (left.y+right.y)/2,
                                      min(left.visibility, right.visibility))
        eyes = landmarks[self.LEFT_EYE], landmarks[self.RIGHT_EYE]
        head = MediaPipeFusedLandmark((eyes[0].x+eyes[1].x)/2, (eyes[0].y+eyes[1].y)/2,
                                     min(p.visibility for p in eyes))
        return (left, right, head, landmarks[self.LEFT_HIP], landmarks[self.RIGHT_HIP], chest, *eyes)

    def draw_landmarks(self, image) -> None:
        """Overlay the normal MediaPipe skeleton for a human-readable preview."""
        if self._last_pose_landmarks is None:
            return
        from mediapipe.python.solutions import drawing_utils
        drawing_utils.draw_landmarks(
            image, self._last_pose_landmarks, self._pose_module.POSE_CONNECTIONS)

    def close(self) -> None:
        self._pose.close()

"""Thin MediaPipe wrapper kept separate from ROS and RealSense plumbing."""

from __future__ import annotations


class MediaPipeShoulderDetector:
    LEFT_SHOULDER = 11
    RIGHT_SHOULDER = 12

    def __init__(self, detection_confidence: float = 0.5, tracking_confidence: float = 0.5):
        # This explicit import matches the installed MediaPipe 0.10.x setup
        # already used by test_pose.py; some distributions do not expose
        # ``mediapipe.solutions`` at the top level.
        from mediapipe.python.solutions import pose as mp_pose
        self._pose_module = mp_pose
        self._pose = self._pose_module.Pose(
            static_image_mode=False, model_complexity=1, smooth_landmarks=True,
            min_detection_confidence=detection_confidence,
            min_tracking_confidence=tracking_confidence)
        self._last_pose_landmarks = None

    def detect(self, rgb_image):
        result = self._pose.process(rgb_image)
        self._last_pose_landmarks = result.pose_landmarks
        if not result.pose_landmarks:
            return None
        landmarks = result.pose_landmarks.landmark
        return landmarks[self.LEFT_SHOULDER], landmarks[self.RIGHT_SHOULDER]

    def draw_landmarks(self, image) -> None:
        """Overlay the normal MediaPipe skeleton for a human-readable preview."""
        if self._last_pose_landmarks is None:
            return
        from mediapipe.python.solutions import drawing_utils
        drawing_utils.draw_landmarks(
            image, self._last_pose_landmarks, self._pose_module.POSE_CONNECTIONS)

    def close(self) -> None:
        self._pose.close()

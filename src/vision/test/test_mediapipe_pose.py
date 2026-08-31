from vision.mediapipe_pose import (
    MediaPipeFusedLandmark,
    MediaPipePoseDetector,
    fuse_mediapipe_face_keypoints,
)


def test_mediapipe_landmark_indices_match_33_point_pose_schema():
    assert MediaPipePoseDetector.FACE_INDICES == (0, 2, 5, 7, 8)
    assert (MediaPipePoseDetector.LEFT_SHOULDER, MediaPipePoseDetector.RIGHT_SHOULDER) == (11, 12)
    assert (MediaPipePoseDetector.LEFT_HIP, MediaPipePoseDetector.RIGHT_HIP) == (23, 24)


def test_mediapipe_face_fusion_uses_available_face_points():
    points = [
        MediaPipeFusedLandmark(0.40, 0.20, 0.9),
        MediaPipeFusedLandmark(0.50, 0.20, 0.8),
        MediaPipeFusedLandmark(0.60, 0.20, 0.7),
    ]
    fused = fuse_mediapipe_face_keypoints(points, 0.5)
    assert fused is not None
    expected_x = (0.40 * 0.9 + 0.50 * 0.8 + 0.60 * 0.7) / (0.9 + 0.8 + 0.7)
    assert abs(fused.x - expected_x) < 1.0e-12


def test_mediapipe_face_fusion_can_use_one_visible_ear():
    points = [
        MediaPipeFusedLandmark(0.50, 0.30, 0.1),
        MediaPipeFusedLandmark(0.39, 0.31, 0.88),
    ]
    assert fuse_mediapipe_face_keypoints(points, 0.5) == points[1]

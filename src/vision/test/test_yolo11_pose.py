from vision.yolo11_pose import Yolo11nPoseDetector, YoloPoseLandmark, fuse_face_keypoints


def test_yolo_landmark_indices_match_coco_17_point_schema():
    assert Yolo11nPoseDetector.FACE_INDICES == (0, 1, 2, 3, 4)
    assert (Yolo11nPoseDetector.LEFT_SHOULDER, Yolo11nPoseDetector.RIGHT_SHOULDER) == (5, 6)
    assert (Yolo11nPoseDetector.LEFT_HIP, Yolo11nPoseDetector.RIGHT_HIP) == (11, 12)


def test_face_fusion_uses_all_visible_face_points():
    points = [
        YoloPoseLandmark(0.40, 0.20, 0.9),
        YoloPoseLandmark(0.50, 0.20, 0.8),
        YoloPoseLandmark(0.60, 0.20, 0.7),
    ]
    fused = fuse_face_keypoints(points, 0.5)
    assert fused is not None
    expected_x = (0.40 * 0.9 + 0.50 * 0.8 + 0.60 * 0.7) / (0.9 + 0.8 + 0.7)
    assert abs(fused.x - expected_x) < 1.0e-12
    assert fused.y == 0.20


def test_face_fusion_falls_back_to_visible_ear():
    points = [
        YoloPoseLandmark(0.50, 0.30, 0.1),  # hidden nose
        YoloPoseLandmark(0.44, 0.28, 0.2),  # hidden eye
        YoloPoseLandmark(0.39, 0.31, 0.88),  # visible ear
    ]
    fused = fuse_face_keypoints(points, 0.5)
    assert fused == points[2]


def test_face_fusion_rejects_pose_without_reliable_face_point():
    points = [YoloPoseLandmark(0.5, 0.3, 0.2)]
    assert fuse_face_keypoints(points, 0.5) is None

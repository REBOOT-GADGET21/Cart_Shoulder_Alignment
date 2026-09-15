from vision.yolo11_pose import Yolo11nPoseDetector, YoloPoseLandmark, fuse_face_keypoints


def test_detector_returns_both_eyes_for_independent_depth_sampling():
    from types import SimpleNamespace
    import numpy as np

    class Tensor:
        def __init__(self, value): self.value = np.array(value)
        def detach(self): return self
        def cpu(self): return self
        def numpy(self): return self.value

    class Keypoints:
        xyn = Tensor([[[0.2, 0.2], [0.4, 0.2], [0.9, 0.9],
                       [0.2, 0.4], [0.4, 0.4], [0.2, 0.8], [0.4, 0.8], [0.3, 0.5]]])
        conf = Tensor([[0.9]*8])
        def __len__(self): return 1

    detector = Yolo11nPoseDetector.__new__(Yolo11nPoseDetector)
    detector._layout = detector.CUSTOM_LAYOUT
    detector._person_confidence = detector._keypoint_confidence = 0.5
    detector._device = ""
    detector._model = SimpleNamespace(predict=lambda **kwargs: [SimpleNamespace(keypoints=Keypoints(), boxes=None)])
    points = detector.detect(None)
    assert len(points) == 8
    assert abs(points[2].x - 0.3) < 1e-9  # Mouth at x=.9 must not shift eye centre.
    assert points[6].x == 0.2 and points[7].x == 0.4


def test_yolo_landmark_indices_match_custom_eight_keypoint_schema():
    layout = Yolo11nPoseDetector.layout_for_keypoint_count(8)
    assert layout.face_indices == (0, 1, 2)
    assert (layout.left_shoulder, layout.right_shoulder) == (3, 4)
    assert (layout.left_hip, layout.right_hip) == (5, 6)
    assert layout.chest == 7


def test_yolo_landmark_indices_match_standard_coco_schema():
    layout = Yolo11nPoseDetector.layout_for_keypoint_count(17)
    assert layout.face_indices == (0, 1, 2, 3, 4)
    assert (layout.left_shoulder, layout.right_shoulder) == (5, 6)
    assert (layout.left_hip, layout.right_hip) == (11, 12)
    assert layout.chest is None


def test_standard_coco_chest_is_estimated_from_shoulders_and_hips():
    point = Yolo11nPoseDetector._estimated_chest(
        YoloPoseLandmark(0.4, 0.2, 0.9),
        YoloPoseLandmark(0.6, 0.2, 0.8),
        YoloPoseLandmark(0.4, 0.8, 0.7),
        YoloPoseLandmark(0.6, 0.8, 0.6),
    )
    assert point == YoloPoseLandmark(0.5, 0.5, 0.6)


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


def test_face_fusion_falls_back_to_one_visible_face_keypoint():
    points = [
        YoloPoseLandmark(0.50, 0.30, 0.1),  # hidden nose
        YoloPoseLandmark(0.44, 0.28, 0.2),  # hidden eye
        YoloPoseLandmark(0.39, 0.31, 0.88),  # visible nose
    ]
    fused = fuse_face_keypoints(points, 0.5)
    assert fused == points[2]


def test_face_fusion_rejects_pose_without_reliable_face_point():
    points = [YoloPoseLandmark(0.5, 0.3, 0.2)]
    assert fuse_face_keypoints(points, 0.5) is None

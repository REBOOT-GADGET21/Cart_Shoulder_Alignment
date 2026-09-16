"""
D435 + YOLO Pose 단독 Preview 코드

목적
- ROS2 실행 X
- TF / Publisher / Nav2 X
- D435 카메라만 실행
- YOLO Pose keypoint + bounding box 표시
- LEFT / RIGHT shoulder 강조
- 각 어깨의 3D camera coordinate 표시
- 현재 vision_node.py preview 화면과 최대한 동일하게 표시

종료: q
"""

from __future__ import annotations

import math

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO


# ============================================================
# 설정
# ============================================================

# 실제 패키지에서 사용하는 모델 경로로 바꿔주세요.
# custom 8-keypoint best.pt를 사용한다면 best.pt 경로 지정
MODEL_PATH = "dataset_images_5/best.pt"

PERSON_CONFIDENCE = 0.45
KEYPOINT_CONFIDENCE = 0.50

WIDTH = 640
HEIGHT = 480
FPS = 30

# vision_node.py와 동일한 depth patch radius
DEPTH_PATCH_RADIUS = 2

# GPU 강제 사용하려면 "0"
# 자동 선택은 ""
DEVICE = ""


# ============================================================
# Pose Layout
# ============================================================

CUSTOM_8 = {
    "name": "custom_cart_8",
    "count": 8,

    "left_eye": 0,
    "right_eye": 1,
    "mouth": 2,

    "left_shoulder": 3,
    "right_shoulder": 4,

    "left_hip": 5,
    "right_hip": 6,

    "chest": 7,
}


COCO_17 = {
    "name": "coco_17",
    "count": 17,

    "mouth": 0,

    "left_eye": 1,
    "right_eye": 2,

    "left_shoulder": 5,
    "right_shoulder": 6,

    "left_hip": 11,
    "right_hip": 12,

    "chest": None,
}


# ============================================================
# Utility
# ============================================================

def get_layout(model):
    """모델의 keypoint 개수로 custom 8 / COCO 17 자동 판별."""

    kpt_shape = getattr(
        getattr(model, "model", None),
        "kpt_shape",
        None,
    )

    if kpt_shape is None:
        raise RuntimeError(
            "YOLO model의 keypoint shape를 확인할 수 없습니다."
        )

    count = int(kpt_shape[0])

    if count == 8:
        return CUSTOM_8

    if count == 17:
        return COCO_17

    raise RuntimeError(
        f"지원하지 않는 pose model입니다. "
        f"keypoint count = {count}"
    )


def robust_depth_m(
    depth_image,
    u,
    v,
    depth_scale,
    radius=2,
):
    """
    한 pixel의 depth 하나만 쓰지 않고
    주변 patch의 유효한 depth 중앙값 사용.

    vision_node.py의 robust_depth_m 역할을 간단히 재현.
    """

    h, w = depth_image.shape

    x0 = max(0, u - radius)
    x1 = min(w, u + radius + 1)

    y0 = max(0, v - radius)
    y1 = min(h, v + radius + 1)

    patch = depth_image[y0:y1, x0:x1]

    valid = patch[patch > 0]

    if valid.size == 0:
        return None

    depth_raw = float(np.median(valid))

    return depth_raw * depth_scale


def deproject(intrinsics, u, v, depth_m):
    """
    2D pixel + depth -> Camera 3D coordinate.

    RealSense 좌표:
        x : right
        y : down
        z : forward
    """

    xyz = rs.rs2_deproject_pixel_to_point(
        intrinsics,
        [float(u), float(v)],
        float(depth_m),
    )

    return np.array(xyz, dtype=np.float32)


def get_best_person(result, layout):
    """
    기존 yolo11_pose.py와 동일한 방식으로
    어깨 + 골반 confidence가 좋은 사람 선택.
    """

    if result is None:
        return None

    if result.keypoints is None:
        return None

    if len(result.keypoints) == 0:
        return None

    normalized = (
        result.keypoints.xyn
        .detach()
        .cpu()
        .numpy()
    )

    conf_tensor = result.keypoints.conf

    if conf_tensor is None:
        confidences = np.ones(
            (
                len(normalized),
                layout["count"],
            ),
            dtype=np.float32,
        )
    else:
        confidences = (
            conf_tensor
            .detach()
            .cpu()
            .numpy()
        )

    box_confidences = None

    if (
        result.boxes is not None
        and result.boxes.conf is not None
    ):
        box_confidences = (
            result.boxes.conf
            .detach()
            .cpu()
            .numpy()
        )

    required_indices = (
        layout["left_shoulder"],
        layout["right_shoulder"],
        layout["left_hip"],
        layout["right_hip"],
    )

    best_index = None
    best_score = -math.inf

    for index, person_conf in enumerate(confidences):

        if len(person_conf) < layout["count"]:
            continue

        core_score = min(
            float(person_conf[i])
            for i in required_indices
        )

        if box_confidences is not None:
            box_score = float(
                box_confidences[index]
            )
        else:
            box_score = 1.0

        score = (
            core_score
            + 0.25 * box_score
        )

        if score > best_score:
            best_score = score
            best_index = index

    if best_index is None:
        return None

    return (
        best_index,
        normalized[best_index],
        confidences[best_index],
    )


# ============================================================
# Main
# ============================================================

def main():

    print("=" * 60)
    print("D435 + YOLO Pose Preview")
    print(f"Model : {MODEL_PATH}")
    print("Press q to quit")
    print("=" * 60)

    # --------------------------------------------------------
    # YOLO
    # --------------------------------------------------------

    model = YOLO(MODEL_PATH)

    layout = get_layout(model)

    print(
        f"Detected pose layout: "
        f"{layout['name']} "
        f"({layout['count']} keypoints)"
    )

    # --------------------------------------------------------
    # RealSense
    # --------------------------------------------------------

    pipeline = rs.pipeline()

    config = rs.config()

    config.enable_stream(
        rs.stream.color,
        WIDTH,
        HEIGHT,
        rs.format.bgr8,
        FPS,
    )

    config.enable_stream(
        rs.stream.depth,
        WIDTH,
        HEIGHT,
        rs.format.z16,
        FPS,
    )

    profile = pipeline.start(config)

    # depth를 color 좌표계로 정렬
    align = rs.align(
        rs.stream.color
    )

    depth_sensor = (
        profile
        .get_device()
        .first_depth_sensor()
    )

    depth_scale = (
        depth_sensor
        .get_depth_scale()
    )

    print(
        f"Depth scale: "
        f"{depth_scale}"
    )

    try:

        while True:

            # ==================================================
            # Camera
            # ==================================================

            frames = pipeline.wait_for_frames()

            frames = align.process(
                frames
            )

            color_frame = (
                frames.get_color_frame()
            )

            depth_frame = (
                frames.get_depth_frame()
            )

            if not color_frame or not depth_frame:
                continue

            bgr = np.asanyarray(
                color_frame.get_data()
            )

            depth_image = np.asanyarray(
                depth_frame.get_data()
            )

            intrinsics = (
                depth_frame.profile
                .as_video_stream_profile()
                .intrinsics
            )

            # ==================================================
            # YOLO inference
            # ==================================================

            # 기존 코드와 동일:
            # BGR -> RGB 후 YOLO에 전달
            rgb = cv2.cvtColor(
                bgr,
                cv2.COLOR_BGR2RGB,
            )

            predict_args = {
                "source": rgb,
                "conf": PERSON_CONFIDENCE,
                "verbose": False,
            }

            if DEVICE:
                predict_args["device"] = DEVICE

            results = model.predict(
                **predict_args
            )

            result = (
                results[0]
                if results
                else None
            )

            # --------------------------------------------------
            # 기본 화면
            # --------------------------------------------------

            view = bgr.copy()

            if result is not None:

                # 기존 draw_landmarks() 동작 재현
                #
                # Ultralytics plot:
                # - bounding box
                # - keypoint
                # - skeleton
                annotated = result.plot()

                if annotated.shape == view.shape:

                    # 기존 yolo11_pose.py와 동일한 방식
                    view[:] = cv2.cvtColor(
                        annotated,
                        cv2.COLOR_RGB2BGR,
                    )

            # ==================================================
            # 사람 선택
            # ==================================================

            selected = get_best_person(
                result,
                layout,
            )

            label = (
                "No reliable shoulders detected"
            )

            if selected is not None:

                (
                    person_index,
                    xyn,
                    confidence,
                ) = selected

                left_index = (
                    layout["left_shoulder"]
                )

                right_index = (
                    layout["right_shoulder"]
                )

                left_conf = float(
                    confidence[left_index]
                )

                right_conf = float(
                    confidence[right_index]
                )

                # 기존 vision_node.py의
                # visibility 기준과 유사하게 처리
                if (
                    left_conf
                    >= KEYPOINT_CONFIDENCE
                    and
                    right_conf
                    >= KEYPOINT_CONFIDENCE
                ):

                    # ==========================================
                    # normalized -> pixel
                    # ==========================================

                    left_u = round(
                        float(xyn[left_index][0])
                        * intrinsics.width
                    )

                    left_v = round(
                        float(xyn[left_index][1])
                        * intrinsics.height
                    )

                    right_u = round(
                        float(xyn[right_index][0])
                        * intrinsics.width
                    )

                    right_v = round(
                        float(xyn[right_index][1])
                        * intrinsics.height
                    )

                    # 화면 밖 방지
                    left_u = int(
                        np.clip(
                            left_u,
                            0,
                            WIDTH - 1,
                        )
                    )

                    left_v = int(
                        np.clip(
                            left_v,
                            0,
                            HEIGHT - 1,
                        )
                    )

                    right_u = int(
                        np.clip(
                            right_u,
                            0,
                            WIDTH - 1,
                        )
                    )

                    right_v = int(
                        np.clip(
                            right_v,
                            0,
                            HEIGHT - 1,
                        )
                    )

                    # ==========================================
                    # Depth
                    # ==========================================

                    left_depth = robust_depth_m(
                        depth_image,
                        left_u,
                        left_v,
                        depth_scale,
                        DEPTH_PATCH_RADIUS,
                    )

                    right_depth = robust_depth_m(
                        depth_image,
                        right_u,
                        right_v,
                        depth_scale,
                        DEPTH_PATCH_RADIUS,
                    )

                    # ==========================================
                    # 3D Coordinate
                    # ==========================================

                    if (
                        left_depth is not None
                        and
                        right_depth is not None
                    ):

                        left_point = deproject(
                            intrinsics,
                            left_u,
                            left_v,
                            left_depth,
                        )

                        right_point = deproject(
                            intrinsics,
                            right_u,
                            right_v,
                            right_depth,
                        )

                        # ======================================
                        # 기존 preview 스타일
                        # ======================================

                        pixels = [
                            (left_u, left_v),
                            (right_u, right_v),
                        ]

                        points = [
                            left_point,
                            right_point,
                        ]

                        colors = [
                            (0, 0, 255),      # LEFT = RED
                            (0, 255, 255),    # RIGHT = YELLOW
                        ]

                        names = [
                            "LEFT",
                            "RIGHT",
                        ]

                        for (
                            (u, v),
                            point,
                            color,
                            name,
                        ) in zip(
                            pixels,
                            points,
                            colors,
                            names,
                        ):

                            # 어깨 위치 강조
                            cv2.circle(
                                view,
                                (u, v),
                                7,
                                color,
                                -1,
                            )

                            # ----------------------------------
                            # 기존 vision_node.py와 동일한
                            # text 위치 계산
                            # ----------------------------------

                            if name == "LEFT":

                                text_x = max(
                                    5,
                                    u - 150,
                                )

                            else:

                                text_x = min(
                                    view.shape[1] - 145,
                                    u + 14,
                                )

                            text_y = max(
                                45,
                                min(
                                    view.shape[0] - 70,
                                    v - 24,
                                ),
                            )

                            texts = (
                                f"{name}",
                                f"x={point[0]:+.3f} m",
                                f"y={point[1]:+.3f} m",
                                f"z={point[2]:.3f} m",
                            )

                            for (
                                line_index,
                                text,
                            ) in enumerate(texts):

                                cv2.putText(
                                    view,
                                    text,
                                    (
                                        text_x,
                                        text_y
                                        + 17
                                        * line_index,
                                    ),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.45,
                                    color,
                                    1,
                                    cv2.LINE_AA,
                                )

                        # ======================================
                        # 어깨 중심도 계산만 함
                        # 원래 코드의 error_x에 해당
                        # ======================================

                        shoulder_center_u = (
                            left_u + right_u
                        ) / 2.0

                        image_center_u = (
                            intrinsics.width / 2.0
                        )

                        error_x = (
                            shoulder_center_u
                            - image_center_u
                        )

                        # 필요하면 화면 중앙선 표시 가능
                        # 현재 패키지 화면에는 없으므로
                        # 일부러 그리지 않음.

                        label = (
                            "Camera coordinates: "
                            "x=right, y=down, z=forward"
                        )

            # ==================================================
            # 기존 vision_node.py의 상단 label
            # ==================================================

            cv2.putText(
                view,
                label,
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.60,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

            # ==================================================
            # imshow
            # ==================================================

            cv2.imshow(
                "D435 yolo11n_pose pose measurement (q to close)",
                view,
            )

            key = (
                cv2.waitKey(1)
                & 0xFF
            )

            if key == ord("q"):
                break

    finally:

        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
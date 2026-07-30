"""
realsense를 이용하여 Mediapipe Pose를 켜서 화각을 보기 위한 코드
python test_pose.py로 코드를 실행
"""

import cv2
import numpy as np
import pyrealsense2 as rs

# mp.solutions 대신 직접 solutions에서 불러오기
import mediapipe as mp
from mediapipe.python.solutions import drawing_utils as mp_drawing
from mediapipe.python.solutions import pose as mp_pose

# RealSense 설정
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

pipeline.start(config)

# Pose 객체 생성
pose = mp_pose.Pose(
    static_image_mode=False,
    model_complexity=1,
    smooth_landmarks=True,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5,
)

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())

        # BGR -> RGB
        rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        results = pose.process(rgb_image)

        if results.pose_landmarks:
            mp_drawing.draw_landmarks(
                color_image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS
            )

        cv2.imshow("RealSense D435 - MediaPipe Pose", color_image)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
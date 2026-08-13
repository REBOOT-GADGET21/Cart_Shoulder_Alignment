"""Publish filtered D435/MediaPipe shoulders in the odom (rear-pivot) world frame."""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
import tf2_geometry_msgs  # Registers geometry message conversions with tf2.
from tf2_ros import Buffer, TransformException, TransformListener

from .camera_types import CameraIntrinsics
from .deprojection import deproject_pixel, robust_depth_m
from .landmark_filter import LandmarkFilter
from .mediapipe_pose import MediaPipeShoulderDetector


class RealSenseMediaPipePoseNode(Node):
    def __init__(self) -> None:
        super().__init__("realsense_mediapipe_pose")
        self.declare_parameter("visibility_threshold", 0.65)
        self.declare_parameter("filter_alpha", 0.35)
        self.declare_parameter("median_window", 7)
        self.declare_parameter("outlier_distance_m", 0.20)
        self.declare_parameter("depth_patch_radius_px", 2)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("show_preview", True)
        self.declare_parameter("color_width", 640)
        self.declare_parameter("color_height", 480)
        self.declare_parameter("fps", 30)
        self.visibility = float(self.get_parameter("visibility_threshold").value)
        alpha = float(self.get_parameter("filter_alpha").value)
        window = int(self.get_parameter("median_window").value)
        outlier = float(self.get_parameter("outlier_distance_m").value)
        self.depth_radius = int(self.get_parameter("depth_patch_radius_px").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.show_preview = bool(self.get_parameter("show_preview").value)
        self.color_width = int(self.get_parameter("color_width").value)
        self.color_height = int(self.get_parameter("color_height").value)
        self.fps = int(self.get_parameter("fps").value)
        self.close_requested = False
        self.shoulder_pub = self.create_publisher(PoseArray, "/shoulder_line", 10)
        self.camera_shoulder_pub = self.create_publisher(PoseArray, "/shoulder_line_camera", 10)
        self.valid_pub = self.create_publisher(Bool, "/shoulder_line_status", 10)
        self.camera_valid_pub = self.create_publisher(Bool, "/shoulder_line_camera_status", 10)
        self.latency_pub = self.create_publisher(Float32, "/shoulder_line/processing_latency_ms", 10)
        self.error_x_pub = self.create_publisher(Float32, "/alignment/error_x_px", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.left_filter, self.right_filter = LandmarkFilter(alpha, window, outlier), LandmarkFilter(alpha, window, outlier)
        self.pipeline = None
        self.detector = None
        self.last_tf_notice_s = 0.0
        self.last_measurement_notice_s = 0.0
        self.timer = self.create_timer(0.0 + 1.0 / 30.0, self.process_frame)
        self._start_camera()

    def _start_camera(self) -> None:
        try:
            import pyrealsense2 as rs
            self.rs = rs
            self.pipeline = rs.pipeline()
            config = rs.config()
            # 640x480 was an initial performance setting.  D435 supports this
            # 1280x720 preview/measurement setting and it is now the default.
            config.enable_stream(rs.stream.color, self.color_width, self.color_height, rs.format.bgr8, self.fps)
            config.enable_stream(rs.stream.depth, self.color_width, self.color_height, rs.format.z16, self.fps)
            profile = self.pipeline.start(config)
            self.align = rs.align(rs.stream.color)
            depth_sensor = profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            self.detector = MediaPipeShoulderDetector()
            self.get_logger().info(
                "D435 + MediaPipe started. Read /shoulder_line_camera now: it is the actual filtered "
                "camera measurement (pose[0]=left, pose[1]=right). No platform is required.")
        except Exception as exc:
            self.get_logger().error(f"Could not start D435/MediaPipe: {exc}")
            self.pipeline = None

    def _publish_valid(self, valid: bool) -> None:
        self.valid_pub.publish(Bool(data=valid))

    def _preview(self, image, pixels, points, label: str) -> None:
        """Bench-test display of the D435 image and its computed 3-D points."""
        if not self.show_preview:
            return
        
        import cv2
        view = image.copy()
        self.detector.draw_landmarks(view)
        for (u, v), point, color, name in zip(pixels, points, ((0, 0, 255), (0, 255, 255)), ("LEFT", "RIGHT")):
            cv2.circle(view, (u, v), 7, color, -1)
            # Left labels stay to the left of the left shoulder; right labels
            # stay to the right of the right shoulder, each on separate lines.
            text_x = max(5, u - 150) if name == "LEFT" else min(view.shape[1] - 145, u + 14)
            text_y = max(45, min(view.shape[0] - 70, v - 24))
            for line_index, text in enumerate((f"{name}", f"x={point.x:+.3f} m", f"y={point.y:+.3f} m", f"z={point.z:.3f} m")):
                cv2.putText(view, text, (text_x, text_y + 17 * line_index),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
        cv2.putText(view, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.imshow("D435 MediaPipe shoulder measurement (q to close)", view)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.close_requested = True

    def process_frame(self) -> None:
        if self.pipeline is None or self.detector is None:
            self._publish_valid(False)
            return
        started = time.perf_counter()
        try:
            frames = self.align.process(self.pipeline.wait_for_frames(1))
            color, depth = frames.get_color_frame(), frames.get_depth_frame()
            if not color or not depth:
                self._publish_valid(False); return
            
            import cv2
            import numpy as np
            bgr = np.asanyarray(color.get_data())
            found = self.detector.detect(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
            if found is None or min(found[0].visibility, found[1].visibility) < self.visibility:
                self._preview(bgr, [], [], "No reliable shoulders detected")
                if self.close_requested:
                    rclpy.shutdown(); return
                self.camera_valid_pub.publish(Bool(data=False))
                self._publish_valid(False); return
            intr = depth.profile.as_video_stream_profile().intrinsics
            intrinsics = CameraIntrinsics(intr.width, intr.height, intr.fx, intr.fy, intr.ppx, intr.ppy)
            raw_depth = np.asanyarray(depth.get_data())
            points = []
            pixels = []
            for landmark, filt in zip(found, (self.left_filter, self.right_filter)):
                u, v = round(landmark.x * intrinsics.width), round(landmark.y * intrinsics.height)
                depth_m = robust_depth_m(raw_depth, u, v, self.depth_scale, self.depth_radius)
                point = deproject_pixel(intrinsics, u, v, depth_m) if depth_m else None
                if point is None:
                    self.camera_valid_pub.publish(Bool(data=False))
                    self._publish_valid(False); return
                points.append(filt.update(point))
                pixels.append((u, v))
            camera_message = PoseArray()
            camera_message.header.stamp = self.get_clock().now().to_msg()
            camera_message.header.frame_id = self.camera_frame
            for point in points:
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = point.x, point.y, point.z
                camera_message.poses.append(pose)
            self.camera_shoulder_pub.publish(camera_message)
            self.camera_valid_pub.publish(Bool(data=True))
            # 검출된 두 어깨의 중점과 영상 수평 중심의 차이다. 제어값에는 사용하지 않는다.
            shoulder_center_u = (pixels[0][0] + pixels[1][0]) / 2.0
            self.error_x_pub.publish(Float32(data=shoulder_center_u - intrinsics.width / 2.0))
            self._preview(bgr, pixels, points, "Camera coordinates: x=right, y=down, z=forward")
            if self.close_requested:
                self.get_logger().info("Preview closed with q; stopping D435/MediaPipe node")
                rclpy.shutdown(); return
            if time.monotonic() - self.last_measurement_notice_s >= 1.0:
                self.last_measurement_notice_s = time.monotonic()
                self.get_logger().info(f"Camera shoulders [m] left=({points[0].x:+.3f}, {points[0].y:+.3f}, {points[0].z:.3f}), right=({points[1].x:+.3f}, {points[1].y:+.3f}, {points[1].z:.3f})")
            message = PoseArray()
            message.header.stamp = camera_message.header.stamp
            message.header.frame_id = self.odom_frame
            for pose in camera_message.poses:
                source = PoseStamped()
                source.header.stamp = message.header.stamp
                source.header.frame_id = self.camera_frame
                source.pose = pose
                transformed = self.tf_buffer.transform(source, self.odom_frame, timeout=Duration(seconds=0.05))
                message.poses.append(transformed.pose)
            self.shoulder_pub.publish(message)
            self._publish_valid(True)
            self.latency_pub.publish(Float32(data=(time.perf_counter() - started) * 1000.0))
        except TransformException as exc:
            if time.monotonic() - self.last_tf_notice_s >= 10.0:
                self.last_tf_notice_s = time.monotonic()
                self.get_logger().info(
                    f"Camera measurements are valid; /shoulder_line is withheld because {self.odom_frame} <- "
                    f"{self.camera_frame} TF does not exist. This is expected without the platform.")
            self._publish_valid(False)
        except Exception as exc:
            self.get_logger().error(f"Frame processing failed: {exc}", throttle_duration_sec=2.0)
            self._publish_valid(False)

    def destroy_node(self):
        try:
            import cv2
            cv2.destroyAllWindows()
        except ImportError:
            pass
        if self.detector:
            self.detector.close()
        if self.pipeline:
            self.pipeline.stop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RealSenseMediaPipePoseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()

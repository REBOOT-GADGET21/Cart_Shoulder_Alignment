"""Publish an explicitly configured, fixed body landmark set for safe drive testing.

This node intentionally has no camera or MediaPipe dependency. Coordinates are
in the odom frame and must be measured/selected by the operator for a clear,
empty test area.
"""

from __future__ import annotations

import math
import json
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from std_msgs.msg import Bool

from .fake_shoulder_geometry import fake_body_reference_from_config


class FixedBodyLandmarkPublisher(Node):
    def __init__(self) -> None:
        super().__init__("fixed_body_landmark_publisher")
        self.declare_parameter("enabled", False)
        self.declare_parameter("use_gazebo_fake_body_config", True)
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("publish_rate_hz", 20.0)
        for name in ("left_shoulder_x_m", "left_shoulder_y_m", "right_shoulder_x_m", "right_shoulder_y_m",
                     "head_x_m", "head_y_m", "pelvis_x_m", "pelvis_y_m"):
            self.declare_parameter(name, 0.0)
        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        self.publisher = self.create_publisher(PoseArray, "/shoulder_line", 10)
        self.valid_publisher = self.create_publisher(Bool, "/shoulder_line_status", 10)
        self.config_landmarks = self._load_gazebo_fake_body_config()
        self.timer = self.create_timer(1.0 / rate_hz, self.publish)
        self.get_logger().warning("Fixed landmarks are disabled until started with enabled:=true")

    @staticmethod
    def _config_path() -> Path:
        workspace_path = Path.cwd() / "src" / "params_setting.json"
        if workspace_path.exists():
            return workspace_path
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("alignment")) / "config" / "params_setting.json"

    def _load_gazebo_fake_body_config(self) -> list[Pose]:
        """Reuse the same centre, width, and angle as Gazebo fake perception."""
        config = json.loads(self._config_path().read_text(encoding="utf-8"))
        body = fake_body_reference_from_config(config)
        return [
            self._pose(body.shoulder_line.left.x_m, body.shoulder_line.left.y_m),
            self._pose(body.shoulder_line.right.x_m, body.shoulder_line.right.y_m),
            self._pose(body.head.x_m, body.head.y_m), self._pose(body.pelvis.x_m, body.pelvis.y_m),
        ]

    @staticmethod
    def _pose(x_m: float, y_m: float) -> Pose:
        pose = Pose()
        pose.position.x, pose.position.y, pose.orientation.w = x_m, y_m, 1.0
        return pose

    def _point(self, x_name: str, y_name: str) -> Pose:
        return self._pose(float(self.get_parameter(x_name).value), float(self.get_parameter(y_name).value))

    def publish(self) -> None:
        if not bool(self.get_parameter("enabled").value):
            self.valid_publisher.publish(Bool(data=False))
            return
        message = PoseArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(self.get_parameter("frame_id").value)
        message.poses = self.config_landmarks if bool(self.get_parameter("use_gazebo_fake_body_config").value) else [
            self._point("left_shoulder_x_m", "left_shoulder_y_m"),
            self._point("right_shoulder_x_m", "right_shoulder_y_m"),
            self._point("head_x_m", "head_y_m"), self._point("pelvis_x_m", "pelvis_y_m")]
        values = [pose.position.x for pose in message.poses] + [pose.position.y for pose in message.poses]
        shoulder_width = math.hypot(message.poses[1].position.x - message.poses[0].position.x,
                                    message.poses[1].position.y - message.poses[0].position.y)
        valid = all(math.isfinite(value) for value in values) and shoulder_width >= 0.05
        self.valid_publisher.publish(Bool(data=valid))
        if valid:
            self.publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FixedBodyLandmarkPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

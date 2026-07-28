"""Publish four known D435 optical-frame landmarks for alignment testing."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node


class FakeShoulderPublisher(Node):
    """Publish left and right shoulder positions in the cart frame."""

    def __init__(self) -> None:
        super().__init__("fake_shoulder_publisher")
        self.publisher = self.create_publisher(PoseArray, "/fake_body_landmarks_optical", 10)
        self.timer = self.create_timer(0.5, self.publish_shoulders)

    def publish_shoulders(self) -> None:
        """Publish shoulder/pelvis points in left-shoulder-first optical order."""

        message = PoseArray()
        message.header.frame_id = "camera_optical_frame"
        left_shoulder, right_shoulder, left_pelvis, right_pelvis = Pose(), Pose(), Pose(), Pose()
        left_shoulder.position.x, left_shoulder.position.y, left_shoulder.position.z = -0.25, 0.20, 1.60
        right_shoulder.position.x, right_shoulder.position.y, right_shoulder.position.z = 0.25, 0.20, 1.60
        left_pelvis.position.x, left_pelvis.position.y, left_pelvis.position.z = -0.22, 0.50, 1.60
        right_pelvis.position.x, right_pelvis.position.y, right_pelvis.position.z = 0.22, 0.50, 1.60
        message.poses = [left_shoulder, right_shoulder, left_pelvis, right_pelvis]
        self.publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FakeShoulderPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

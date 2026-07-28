"""Publish two known shoulder points for the Phase 6 ROS wiring test."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node


class FakeShoulderPublisher(Node):
    """Publish left and right shoulder positions in the cart frame."""

    def __init__(self) -> None:
        super().__init__("fake_shoulder_publisher")
        self.publisher = self.create_publisher(PoseArray, "/fake_shoulders", 10)
        self.timer = self.create_timer(0.5, self.publish_shoulders)

    def publish_shoulders(self) -> None:
        """Publish a line rotated about 26.6 degrees from the cart x axis."""

        message = PoseArray()
        left, right = Pose(), Pose()
        left.position.x, left.position.y = 0.0, 0.0
        right.position.x, right.position.y = 1.0, 0.5
        message.poses = [left, right]
        self.publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FakeShoulderPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

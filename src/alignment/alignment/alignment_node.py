"""Convert fake D435 optical landmarks into an alignment Twist command."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseArray, Twist
from rclpy.node import Node

from geometry.vector_math import Point3D

from .alignment_pipeline import BodyLandmarksOptical, compute_camera_alignment
from .calibration import load_alignment_settings


class AlignmentNode(Node):
    """Subscribe to four optical-frame landmarks and publish a safe Twist."""

    def __init__(self) -> None:
        super().__init__("alignment_node")
        self.settings = load_alignment_settings()
        self.subscription_ = self.create_subscription(
            PoseArray, "/fake_body_landmarks_optical", self.on_landmarks, 10
        )
        self.publisher_ = self.create_publisher(Twist, "/alignment_cmd", 10)
        self.get_logger().info("alignment_node started")

    def on_landmarks(self, msg: PoseArray) -> None:
        """Process left/right shoulders and pelvis points in optical-frame order."""

        if len(msg.poses) != 4:
            self.get_logger().warning("Expected 4 poses: left/right shoulder, left/right pelvis; publishing stop")
            self.publisher_.publish(Twist())
            return
        points = [Point3D(pose.position.x, pose.position.y, pose.position.z) for pose in msg.poses]
        result = compute_camera_alignment(
            BodyLandmarksOptical(points[0], points[1], points[2], points[3]), self.settings
        )

        twist_msg = Twist()
        if not result.valid:
            self.get_logger().warning(f"Body landmarks rejected: {result.body_line_source}")
            self.publisher_.publish(twist_msg)
            return
        twist_msg.linear.x = result.command.linear_x_mps
        twist_msg.angular.z = result.command.angular_z_rad_s
        self.publisher_.publish(twist_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = AlignmentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

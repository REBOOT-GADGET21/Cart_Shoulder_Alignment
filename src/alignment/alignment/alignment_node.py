"""A minimal alignment node for Phase 6."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseArray, Twist
from rclpy.node import Node

from geometry.alignment_error import compute_alignment_error
from geometry.vector_math import Point3D


class AlignmentNode(Node):
    """Subscribe to fake shoulder input and publish a simple velocity command."""

    def __init__(self) -> None:
        super().__init__("alignment_node")
        self.subscription_ = self.create_subscription(PoseArray, "/fake_shoulders", self.on_shoulders, 10)
        self.publisher_ = self.create_publisher(Twist, "/alignment_cmd", 10)
        self.get_logger().info("alignment_node started")

    def on_shoulders(self, msg: PoseArray) -> None:
        """Turn two fake shoulder positions into a safe angular command."""

        if len(msg.poses) != 2:
            self.get_logger().warning("Expected exactly two shoulder poses; publishing stop")
            self.publisher_.publish(Twist())
            return
        left_position, right_position = msg.poses[0].position, msg.poses[1].position
        left_shoulder = Point3D(left_position.x, left_position.y, left_position.z)
        right_shoulder = Point3D(right_position.x, right_position.y, right_position.z)

        error = compute_alignment_error(left_shoulder, right_shoulder, vehicle_yaw_rad=0.0)

        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = error.yaw_error_rad
        self.publisher_.publish(twist_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = AlignmentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

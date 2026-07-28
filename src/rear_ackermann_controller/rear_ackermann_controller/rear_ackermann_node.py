"""Convert a Twist command to rear steering angles for Phase 6."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

from .config import load_vehicle_params
from .rear_steer_kinematics import compute_rear_steer_command


class RearAckermannNode(Node):
    """Subscribe to an alignment Twist and publish two steering commands."""

    def __init__(self) -> None:
        super().__init__("rear_ackermann_node")
        self.params = load_vehicle_params()
        self.left_publisher = self.create_publisher(Float64, "/rear_left_steering_cmd", 10)
        self.right_publisher = self.create_publisher(Float64, "/rear_right_steering_cmd", 10)
        self.left_speed_publisher = self.create_publisher(Float64, "/rear_left_wheel_speed_cmd", 10)
        self.right_speed_publisher = self.create_publisher(Float64, "/rear_right_wheel_speed_cmd", 10)
        self.subscription = self.create_subscription(Twist, "/alignment_cmd", self.on_twist, 10)
        self.get_logger().info("rear_ackermann_node started")

    def on_twist(self, message: Twist) -> None:
        """Apply independent rear steer-drive kinematics to one Twist."""

        command = compute_rear_steer_command(
            message.linear.x, message.angular.z, self.params
        )
        self.left_publisher.publish(Float64(data=command.left.steering_angle_rad))
        self.right_publisher.publish(Float64(data=command.right.steering_angle_rad))
        self.left_speed_publisher.publish(Float64(data=command.left.wheel_speed_mps))
        self.right_speed_publisher.publish(Float64(data=command.right.wheel_speed_mps))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RearAckermannNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

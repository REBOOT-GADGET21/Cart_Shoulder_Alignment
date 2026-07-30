"""Convert fake D435 optical landmarks into an alignment Twist command."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Pose2D, PoseArray, Twist
from rclpy.node import Node
from std_msgs.msg import Bool

from geometry.vector_math import Point3D

from .alignment_pipeline import BodyLandmarksOptical, compute_camera_alignment
from .calibration import load_alignment_settings
from .completion_criteria import AlignmentCompletionChecker, AlignmentMeasurement, AlignmentThresholds


class AlignmentNode(Node):
    """Subscribe to four optical-frame landmarks and publish a safe Twist."""

    def __init__(self) -> None:
        super().__init__("alignment_node")
        self.settings = load_alignment_settings()
        self.enabled = False
        self.aligned = False
        self.completion_checker = AlignmentCompletionChecker(AlignmentThresholds(
            self.settings.yaw_tolerance_rad, self.settings.lateral_tolerance_m,
            self.settings.longitudinal_tolerance_m, self.settings.required_stable_frames,
        ))
        self.subscription_ = self.create_subscription(
            PoseArray, "/fake_body_landmarks_optical", self.on_landmarks, 10
        )
        self.publisher_ = self.create_publisher(Twist, "/alignment_cmd", 10)
        self.target_publisher = self.create_publisher(Pose2D, "/alignment_debug/target_base", 10)
        self.enable_subscription = self.create_subscription(Bool, "/alignment_enabled", self.on_enabled, 10)
        self.get_logger().info("alignment_node started; waiting for /alignment_enabled=true")

    def on_enabled(self, message: Bool) -> None:
        """Gate final alignment until navigation has stopped and permission is explicit."""

        if message.data and not self.enabled:
            self.aligned = False
            self.completion_checker = AlignmentCompletionChecker(AlignmentThresholds(
                self.settings.yaw_tolerance_rad, self.settings.lateral_tolerance_m,
                self.settings.longitudinal_tolerance_m, self.settings.required_stable_frames,
            ))
            self.get_logger().info("alignment enabled")
        if not message.data:
            self.aligned = False
        self.enabled = message.data
        if not self.enabled:
            self.publisher_.publish(Twist())

    def on_landmarks(self, msg: PoseArray) -> None:
        """Process left/right shoulders and pelvis points in optical-frame order."""

        if not self.enabled:
            self.publisher_.publish(Twist())
            return
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
            self.completion_checker.update(AlignmentMeasurement(0.0, 0.0, 0.0, False))
            self.get_logger().warning(f"Body landmarks rejected: {result.body_line_source}")
            self.publisher_.publish(twist_msg)
            return
        target = result.target
        assert target is not None
        self.target_publisher.publish(Pose2D(x=target.x_m, y=target.y_m, theta=target.yaw_rad))
        is_aligned = self.completion_checker.update(AlignmentMeasurement(
            target.yaw_rad, target.y_m, target.x_m, True
        ))
        if is_aligned:
            if not self.aligned:
                self.get_logger().info("ALIGNED: stable position and yaw tolerances reached")
            self.aligned = True
        if self.aligned:
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

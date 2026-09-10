"""Publish only Gazebo cart odometry/TF; never publish synthetic body landmarks."""

from __future__ import annotations

import json
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseArray, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class GazeboCartOdometry(Node):
    """Adapt Gazebo's cart pose to the controller's rear-pivot ``/odom``."""

    def __init__(self) -> None:
        super().__init__("gazebo_cart_odometry")
        config_path = Path.cwd() / "src" / "params_setting.json"
        if not config_path.exists():
            from ament_index_python.packages import get_package_share_directory
            config_path = Path(get_package_share_directory("alignment")) / "config" / "params_setting.json"
        config = json.loads(config_path.read_text(encoding="utf-8"))
        self.rear_axle_x_m = float(config["rear_axle_x_m"])
        self.odom_publisher = self.create_publisher(Odometry, "/odom", 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(PoseArray, "/model/rear_steer_cart/pose", self.on_cart_pose, 10)

    def on_cart_pose(self, message: PoseArray) -> None:
        if not message.poses:
            return
        pose = message.poses[0]
        q = pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        stamp = self.get_clock().now().to_msg()

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id, transform.child_frame_id = "odom", "base_link"
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        transform.transform.rotation = q
        self.tf_broadcaster.sendTransform(transform)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id, odom.child_frame_id = "odom", "rear_axle_pivot"
        odom.pose.pose.position.x = pose.position.x + self.rear_axle_x_m * math.cos(yaw)
        odom.pose.pose.position.y = pose.position.y + self.rear_axle_x_m * math.sin(yaw)
        odom.pose.pose.position.z = pose.position.z
        odom.pose.pose.orientation = q
        self.odom_publisher.publish(odom)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GazeboCartOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

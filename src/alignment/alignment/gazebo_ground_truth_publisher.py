"""Publish world-fixed patient landmarks as moving D435 optical-frame measurements."""

from __future__ import annotations

import math
import json
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from geometry.frame_transform import transform_base_point_to_optical
from geometry.vector_math import Point3D

from .calibration import load_alignment_settings


def _load_fake_patient_landmarks() -> tuple[Point3D, Point3D, Point3D, Point3D]:
    """Build all landmarks from the two editable world-frame shoulder points."""

    path = Path.cwd() / "src" / "params_setting.json"
    if not path.exists():
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory("alignment")) / "config" / "params_setting.json"
    raw = json.loads(path.read_text(encoding="utf-8"))
    left = Point3D(raw["gazebo_fake_left_shoulder_x_m"], raw["gazebo_fake_left_shoulder_y_m"], 0.25)
    right = Point3D(raw["gazebo_fake_right_shoulder_x_m"], raw["gazebo_fake_right_shoulder_y_m"], 0.25)

    shoulder_dx, shoulder_dy = right.x_m - left.x_m, right.y_m - left.y_m
    if math.hypot(shoulder_dx, shoulder_dy) < 0.05:
        raise ValueError("Fake left/right shoulder positions must be at least 0.05 m apart")
    body_yaw = math.atan2(shoulder_dy, shoulder_dx) + math.pi / 2.0
    center_x, center_y = (left.x_m + right.x_m) / 2.0, (left.y_m + right.y_m) / 2.0
    # Pelvis stays a supporting, geometrically consistent reference 0.55 m
    # behind the shoulder line; it follows when the two shoulders are edited.
    pelvis_center_x = center_x + 0.55 * math.cos(body_yaw)
    pelvis_center_y = center_y + 0.55 * math.sin(body_yaw)
    lateral_x, lateral_y = -math.sin(body_yaw), math.cos(body_yaw)
    return (
        left, right,
        Point3D(pelvis_center_x + 0.22 * lateral_x, pelvis_center_y + 0.22 * lateral_y, 0.22),
        Point3D(pelvis_center_x - 0.22 * lateral_x, pelvis_center_y - 0.22 * lateral_y, 0.22),
    )


class GazeboGroundTruthPublisher(Node):
    """Publish fake camera data and C++-controller test inputs from Gazebo truth."""

    def __init__(self) -> None:
        super().__init__("gazebo_ground_truth_publisher")
        self.settings = load_alignment_settings()
        self.declare_parameter("publish_shoulders", True)
        self.publish_shoulders = bool(self.get_parameter("publish_shoulders").value)
        self.publisher = self.create_publisher(PoseArray, "/fake_body_landmarks_optical", 10)
        self.cart_pose_publisher = self.create_publisher(Pose, "/gazebo_ground_truth/cart_pose", 10)
        self.odom_publisher = self.create_publisher(Odometry, "/odom", 20)
        self.shoulder_line_publisher = self.create_publisher(PoseArray, "/shoulder_line", 10)
        self.shoulder_status_publisher = self.create_publisher(Bool, "/shoulder_line_status", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.subscription = self.create_subscription(PoseArray, "/model/rear_steer_cart/pose", self.on_cart_pose, 10)
        self.cart_x_m = 0.0
        self.cart_y_m = 0.0
        self.cart_yaw_rad = 0.0
        self.timer = self.create_timer(0.05, self.publish_landmarks)
        self.patient_points_world = _load_fake_patient_landmarks()
        self.publish_static_frames()

    def publish_static_frames(self) -> None:
        """Expose the same named frames used by hardware RViz/perception."""
        rear = TransformStamped()
        rear.header.stamp = self.get_clock().now().to_msg()
        rear.header.frame_id, rear.child_frame_id = "base_link", "rear_axle_pivot"
        rear.transform.translation.x = self.settings.rear_axle_x_m
        rear.transform.rotation.w = 1.0
        camera = TransformStamped()
        camera.header.stamp = rear.header.stamp
        camera.header.frame_id, camera.child_frame_id = "rear_axle_pivot", "camera_color_optical_frame"
        camera.transform.translation.x = self.settings.rear_axle_x_m + self.settings.camera_extrinsics.translation_x_m
        camera.transform.translation.y = self.settings.camera_extrinsics.translation_y_m
        camera.transform.translation.z = self.settings.camera_extrinsics.translation_z_m
        # Optical axes (right, down, forward) relative to an unrotated ROS base frame.
        camera.transform.rotation.x, camera.transform.rotation.y = -0.5, 0.5
        camera.transform.rotation.z, camera.transform.rotation.w = -0.5, 0.5
        self.static_tf_broadcaster.sendTransform([rear, camera])

    def on_cart_pose(self, message: PoseArray) -> None:
        """Store the current Gazebo model pose; this is fake-sensor plumbing only."""

        # The model publishes exactly one pose because link poses are disabled.
        # Do not use a stale spawn pose if Gazebo has not sent one yet.
        if not message.poses:
            return
        cart_pose = message.poses[0]
        self.cart_x_m, self.cart_y_m = cart_pose.position.x, cart_pose.position.y
        self.cart_pose_publisher.publish(cart_pose)
        q = cart_pose.orientation
        self.cart_yaw_rad = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id, transform.child_frame_id = "odom", "base_link"
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = self.cart_x_m, self.cart_y_m, cart_pose.position.z
        transform.transform.rotation = q
        self.tf_broadcaster.sendTransform(transform)
        # Gazebo's model pose is base_link (platform centre).  The C++ TRT node
        # explicitly controls the physical rear-axle pivot instead.
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "rear_axle_pivot"
        odom.pose.pose.position.x = self.cart_x_m + self.settings.rear_axle_x_m * math.cos(self.cart_yaw_rad)
        odom.pose.pose.position.y = self.cart_y_m + self.settings.rear_axle_x_m * math.sin(self.cart_yaw_rad)
        odom.pose.pose.position.z = cart_pose.position.z
        odom.pose.pose.orientation = q
        self.odom_publisher.publish(odom)

    def publish_landmarks(self) -> None:
        """Transform the fixed patient landmarks into the current optical camera frame."""

        cos_yaw, sin_yaw = math.cos(self.cart_yaw_rad), math.sin(self.cart_yaw_rad)
        message = PoseArray()
        message.header.frame_id = "camera_optical_frame"
        for point_world in self.patient_points_world:
            dx_m, dy_m = point_world.x_m - self.cart_x_m, point_world.y_m - self.cart_y_m
            point_base = Point3D(cos_yaw * dx_m + sin_yaw * dy_m, -sin_yaw * dx_m + cos_yaw * dy_m, point_world.z_m)
            point_optical = transform_base_point_to_optical(point_base, self.settings.camera_extrinsics)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = point_optical.x_m, point_optical.y_m, point_optical.z_m
            message.poses.append(pose)
        self.publisher.publish(message)

        if not self.publish_shoulders:
            return
        shoulder_line = PoseArray()
        shoulder_line.header.stamp = self.get_clock().now().to_msg()
        shoulder_line.header.frame_id = "odom"
        for point_world in self.patient_points_world[:2]:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = point_world.x_m, point_world.y_m, point_world.z_m
            shoulder_line.poses.append(pose)
        self.shoulder_line_publisher.publish(shoulder_line)
        self.shoulder_status_publisher.publish(Bool(data=True))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GazeboGroundTruthPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

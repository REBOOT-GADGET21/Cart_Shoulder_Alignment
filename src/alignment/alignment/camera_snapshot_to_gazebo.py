"""Freeze one real camera measurement as a fixed Gazebo TRT target."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose

class CameraSnapshotToGazebo(Node):
    """Take only the first valid camera shoulder line, then publish it unchanged."""

    def __init__(self) -> None:
        super().__init__("camera_snapshot_to_gazebo")
        # These are deliberately independent of params_setting.json.  During
        # this camera-on-desk phase, Gazebo's virtual camera is at the cart
        # origin and faces +X.  Later only these three measured extrinsics need
        # to change when the physical camera is mounted on the platform.
        self.declare_parameter("gazebo_camera_x_m", 0.0)
        self.declare_parameter("gazebo_camera_y_m", 0.0)
        self.declare_parameter("gazebo_camera_yaw_rad", 0.0)
        self.camera_x = float(self.get_parameter("gazebo_camera_x_m").value)
        self.camera_y = float(self.get_parameter("gazebo_camera_y_m").value)
        self.camera_yaw = float(self.get_parameter("gazebo_camera_yaw_rad").value)
        self.snapshot: PoseArray | None = None
        self.captured_at_s: float | None = None
        self.subscription = self.create_subscription(PoseArray, "/shoulder_line_camera", self.on_camera_line, 10)
        self.publisher = self.create_publisher(PoseArray, "/shoulder_line", 10)
        self.set_pose_client = self.create_client(SetEntityPose, "/world/default/set_pose")
        self.visuals_placed = False
        self.placement_attempt_in_progress = False
        self.timer = self.create_timer(0.1, self.publish_snapshot)
        self.get_logger().info("Waiting for one real /shoulder_line_camera measurement; it will then be frozen for Gazebo TRT.")

    def on_camera_line(self, message: PoseArray) -> None:
        if self.snapshot is not None or len(message.poses) < 2:
            return
        snapshot = PoseArray()
        snapshot.header.stamp = self.get_clock().now().to_msg()
        snapshot.header.frame_id = "odom"
        for camera_pose in message.poses[:2]:
            # RealSense optical: x=right, y=down, z=forward.  Gazebo ground:
            # x=forward, y=left.  Height is not used by the planar TRT solver.
            forward, left = camera_pose.position.z, -camera_pose.position.x
            c, s = math.cos(self.camera_yaw), math.sin(self.camera_yaw)
            pose = Pose()
            pose.position.x = self.camera_x + c * forward - s * left
            pose.position.y = self.camera_y + s * forward + c * left
            pose.position.z = 0.0
            snapshot.poses.append(pose)
        self.snapshot = snapshot
        self.captured_at_s = self.get_clock().now().nanoseconds / 1e9
        self.destroy_subscription(self.subscription)
        self.get_logger().info(
            "Captured first camera sample and unsubscribed. Frozen Gazebo shoulders [odom m]: "
            f"L=({snapshot.poses[0].position.x:.3f}, {snapshot.poses[0].position.y:.3f}), "
            f"R=({snapshot.poses[1].position.x:.3f}, {snapshot.poses[1].position.y:.3f}).")

    @staticmethod
    def _yaw_pose(x: float, y: float, z: float, yaw: float) -> Pose:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        pose.orientation.z, pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        return pose

    def _set_model_pose(self, name: str, pose: Pose):
        request = SetEntityPose.Request()
        request.entity.name, request.entity.type = name, Entity.MODEL
        request.pose = pose
        return self.set_pose_client.call_async(request)

    def place_visual_models(self) -> None:
        """Put Gazebo's patient and both visible markers on the frozen target."""
        # cart_only launches its three visual models after five seconds.  Do
        # not send a pose request before those named entities exist.
        now_s = self.get_clock().now().nanoseconds / 1e9
        if (self.snapshot is None or self.visuals_placed or self.placement_attempt_in_progress or not self.set_pose_client.service_is_ready()
                or now_s - self.captured_at_s < 6.0):
            return
        left, right = self.snapshot.poses[:2]
        dx, dy = right.position.x - left.position.x, right.position.y - left.position.y
        if math.hypot(dx, dy) < 0.05:
            self.get_logger().error("Frozen shoulder line is too short to place Gazebo patient model")
            self.visuals_placed = True
            return
        body_yaw = math.atan2(dy, dx) + math.pi / 2.0
        center_x, center_y = (left.position.x + right.position.x) / 2.0, (left.position.y + right.position.y) / 2.0
        # This matches the mesh's existing local shoulder offset in cart_only.launch.py.
        futures = [self._set_model_pose("person_walking", self._yaw_pose(
            center_x + 0.55 * math.cos(body_yaw), center_y + 0.55 * math.sin(body_yaw), 0.0, body_yaw))]
        futures.append(self._set_model_pose("left_shoulder_marker", self._yaw_pose(left.position.x, left.position.y, 0.0, 0.0)))
        futures.append(self._set_model_pose("right_shoulder_marker", self._yaw_pose(right.position.x, right.position.y, 0.0, 0.0)))
        self.placement_attempt_in_progress = True
        self.get_logger().info("Requesting Gazebo to move person_walking and shoulder markers to frozen real measurement.")
        placement_timer = [None]

        def complete_placement() -> None:
            if not all(future.done() for future in futures):
                return
            placement_timer[0].cancel()
            self.placement_attempt_in_progress = False
            try:
                success = all(future.result().success for future in futures)
            except Exception as exc:
                self.get_logger().warning(f"Gazebo set_pose request failed; retrying: {exc}")
                return
            if success:
                self.visuals_placed = True
                self.get_logger().info("Moved Gazebo patient and shoulder markers to the frozen real-camera shoulder line.")
            else:
                self.get_logger().warning("Gazebo did not find one of the target models yet; retrying set_pose.")

        placement_timer[0] = self.create_timer(0.2, complete_placement)

    def publish_snapshot(self) -> None:
        if self.snapshot is not None:
            self.snapshot.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(self.snapshot)
            self.place_visual_models()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraSnapshotToGazebo()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()

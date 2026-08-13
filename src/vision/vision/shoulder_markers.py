"""Convert /shoulder_line into frame-consistent RViz markers."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Point, PoseArray
from rclpy.node import Node
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray


class ShoulderMarkerNode(Node):
    def __init__(self) -> None:
        super().__init__("shoulder_line_markers")
        self.publisher = self.create_publisher(MarkerArray, "/shoulder_line_markers", 10)
        self.valid = False
        self.create_subscription(PoseArray, "/shoulder_line", self.on_shoulders, 10)
        self.create_subscription(Bool, "/shoulder_line_status", self.on_status, 10)

    def on_status(self, message: Bool) -> None:
        self.valid = message.data

    @staticmethod
    def _marker(frame: str, stamp, marker_id: int, marker_type: int) -> Marker:
        m = Marker()
        m.header.frame_id, m.header.stamp = frame, stamp
        m.ns, m.id, m.type, m.action = "shoulder_line", marker_id, marker_type, Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def on_shoulders(self, message: PoseArray) -> None:
        if len(message.poses) < 2:
            return
        left, right = message.poses[:2]
        result = MarkerArray()
        for marker_id, pose, color in ((0, left, (1.0, 0.0, 0.0)), (1, right, (1.0, 1.0, 0.0))):
            marker = self._marker(message.header.frame_id, message.header.stamp, marker_id, Marker.SPHERE)
            marker.pose = pose
            marker.scale.x = marker.scale.y = marker.scale.z = 0.10
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = *color, 1.0
            result.markers.append(marker)
        line = self._marker(message.header.frame_id, message.header.stamp, 2, Marker.LINE_STRIP)
        line.scale.x = 0.025; line.color.b = 1.0; line.color.a = 1.0
        line.points = [left.position, right.position]
        result.markers.append(line)
        dx, dy = right.position.x-left.position.x, right.position.y-left.position.y
        length = math.hypot(dx, dy)
        if length > 1e-4:
            midpoint = Point(x=(left.position.x+right.position.x)/2, y=(left.position.y+right.position.y)/2,
                             z=(left.position.z+right.position.z)/2)
            normal_end = Point(x=midpoint.x-dy/length*0.50, y=midpoint.y+dx/length*0.50, z=midpoint.z)
            normal = self._marker(message.header.frame_id, message.header.stamp, 3, Marker.ARROW)
            normal.scale.x, normal.scale.y, normal.scale.z = 0.03, 0.07, 0.07
            normal.color.g, normal.color.b, normal.color.a = 1.0, 1.0, 1.0
            normal.points = [midpoint, normal_end]
            result.markers.append(normal)
        self.publisher.publish(result)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ShoulderMarkerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()

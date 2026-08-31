"""Save a top-down debug image of the fixed body reference and live odometry.

This is intentionally a read-only ROS node: it subscribes once to /odom and
does not publish a command or a landmark message.  It uses exactly the fake
body parameters that fixed_body_landmark_publisher uses.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, Polygon
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .fake_shoulder_geometry import fake_body_reference_from_config


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class FixedBodyDebugPlot(Node):
    def __init__(self) -> None:
        super().__init__("fixed_body_debug_plot")
        self.declare_parameter("output_path", "fixed_body_alignment_debug.png")
        self.declare_parameter("odom_timeout_s", 5.0)
        self._done = False
        self._config = self._load_config()
        self._output_path = Path(str(self.get_parameter("output_path").value)).expanduser()
        self._subscription = self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        timeout_s = float(self.get_parameter("odom_timeout_s").value)
        self._timeout = self.create_timer(timeout_s, self._on_timeout)
        self.get_logger().info("Waiting for one /odom message; this node does not command the cart.")

    @staticmethod
    def _config_path() -> Path:
        workspace_path = Path.cwd() / "src" / "params_setting.json"
        if workspace_path.exists():
            return workspace_path
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("alignment")) / "config" / "params_setting.json"

    def _load_config(self) -> dict:
        return json.loads(self._config_path().read_text(encoding="utf-8"))

    def _on_timeout(self) -> None:
        if not self._done:
            self.get_logger().error("No /odom received. Start hardware_drive.launch.py, then run this command again.")
            self._done = True

    def _on_odom(self, message: Odometry) -> None:
        if self._done:
            return
        self._done = True
        self._timeout.cancel()
        self._draw(message)

    @staticmethod
    def _footprint(pivot_x: float, pivot_y: float, yaw: float, config: dict) -> list[tuple[float, float]]:
        # /odom is the rear-axle pivot.  Convert the base_link-centred rectangle
        # using the same rear_axle_x_m definition as the hardware TF.
        rear_to_center = -float(config["rear_axle_x_m"])
        center_x = pivot_x + rear_to_center * math.cos(yaw)
        center_y = pivot_y + rear_to_center * math.sin(yaw)
        half_length, half_width = float(config["body_length_m"]) / 2.0, float(config["body_width_m"]) / 2.0
        corners = [(half_length, half_width), (half_length, -half_width), (-half_length, -half_width), (-half_length, half_width)]
        return [(center_x + x * math.cos(yaw) - y * math.sin(yaw), center_y + x * math.sin(yaw) + y * math.cos(yaw)) for x, y in corners]

    @staticmethod
    def _arrow(ax, x: float, y: float, yaw: float, color: str, label: str) -> None:
        dx, dy = 0.55 * math.cos(yaw), 0.55 * math.sin(yaw)
        ax.add_patch(FancyArrowPatch((x, y), (x + dx, y + dy), arrowstyle="->", mutation_scale=16, linewidth=2.5, color=color, label=label))

    def _draw(self, odom: Odometry) -> None:
        body = fake_body_reference_from_config(self._config)
        left, right, head, pelvis = body.shoulder_line.left, body.shoulder_line.right, body.head, body.pelvis
        shoulder_x, shoulder_y = (left.x_m + right.x_m) / 2.0, (left.y_m + right.y_m) / 2.0
        axis_x, axis_y = head.x_m - pelvis.x_m, head.y_m - pelvis.y_m
        axis_length = math.hypot(axis_x, axis_y)
        # The controller selects the shoulder-line normal that points from the
        # pelvis toward the head, then puts the rear pivot on that (head) side.
        normal_x, normal_y = axis_x / axis_length, axis_y / axis_length
        pivot_to_front = float(self._config["body_length_m"]) / 2.0 - float(self._config["rear_axle_x_m"])
        goal_distance = float(self._config["alignment_front_clearance_m"]) + pivot_to_front
        goal_x, goal_y = shoulder_x + goal_distance * normal_x, shoulder_y + goal_distance * normal_y
        goal_yaw = math.atan2(shoulder_y - head.y_m, shoulder_x - head.x_m)
        robot_x, robot_y = odom.pose.pose.position.x, odom.pose.pose.position.y
        robot_yaw = yaw_from_quaternion(odom.pose.pose.orientation)

        fig, ax = plt.subplots(figsize=(10, 8), constrained_layout=True)
        ax.plot([left.x_m, right.x_m], [left.y_m, right.y_m], color="#147d3f", linewidth=5, solid_capstyle="round", label="Shoulder line")
        ax.plot([pelvis.x_m, head.x_m], [pelvis.y_m, head.y_m], color="#b52525", linewidth=3, label="Body axis (pelvis → head)")
        ax.scatter([left.x_m, right.x_m], [left.y_m, right.y_m], c="#147d3f", s=65, zorder=4)
        ax.scatter(head.x_m, head.y_m, c="#b52525", s=90, marker="o", zorder=4, label="Head")
        ax.scatter(pelvis.x_m, pelvis.y_m, c="#b52525", s=75, marker="s", zorder=4, label="Pelvis")
        ax.annotate("L shoulder", (left.x_m, left.y_m), xytext=(7, 8), textcoords="offset points")
        ax.annotate("R shoulder", (right.x_m, right.y_m), xytext=(7, -18), textcoords="offset points")
        ax.annotate("Head", (head.x_m, head.y_m), xytext=(7, 8), textcoords="offset points")
        ax.annotate("Pelvis", (pelvis.x_m, pelvis.y_m), xytext=(7, -18), textcoords="offset points")

        ax.add_patch(Polygon(self._footprint(robot_x, robot_y, robot_yaw, self._config), closed=True, fill=True, alpha=0.23, color="#1677c8", label="Current cart body"))
        ax.scatter(robot_x, robot_y, c="#1677c8", s=55, zorder=5)
        self._arrow(ax, robot_x, robot_y, robot_yaw, "#1677c8", "Current pivot heading")
        ax.add_patch(Polygon(self._footprint(goal_x, goal_y, goal_yaw, self._config), closed=True, fill=False, linewidth=2.2, linestyle="--", color="#e08000", label="Desired cart body"))
        ax.scatter(goal_x, goal_y, c="#e08000", s=55, marker="X", zorder=5)
        self._arrow(ax, goal_x, goal_y, goal_yaw, "#e08000", "Desired pivot heading")
        ax.plot([robot_x, goal_x], [robot_y, goal_y], "k--", alpha=0.55, label="Pivot-to-goal line")

        all_x = [left.x_m, right.x_m, head.x_m, pelvis.x_m, robot_x, goal_x]
        all_y = [left.y_m, right.y_m, head.y_m, pelvis.y_m, robot_y, goal_y]
        margin = 0.85
        ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
        ax.set_ylim(min(all_y) - margin, max(all_y) + margin)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, linestyle=":", alpha=0.6)
        ax.axhline(0.0, color="black", linewidth=0.7, alpha=0.35)
        ax.axvline(0.0, color="black", linewidth=0.7, alpha=0.35)
        ax.set_xlabel("odom x [m]")
        ax.set_ylabel("odom y [m]")
        ax.set_title("Fixed-body alignment debug (top view, odom frame)")
        ax.text(0.01, 0.01, f"current pivot = ({robot_x:.3f}, {robot_y:.3f}), yaw = {math.degrees(robot_yaw):+.1f}°\n"
                            f"goal pivot = ({goal_x:.3f}, {goal_y:.3f}), yaw = {math.degrees(goal_yaw):+.1f}°\n"
                            f"front clearance = {self._config['alignment_front_clearance_m']:.2f} m",
                transform=ax.transAxes, va="bottom", bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "0.7"})
        ax.legend(loc="upper left", fontsize=8)
        self._output_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(self._output_path, dpi=180)
        plt.close(fig)
        self.get_logger().info(f"Saved debug image: {self._output_path.resolve()}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FixedBodyDebugPlot()
    try:
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

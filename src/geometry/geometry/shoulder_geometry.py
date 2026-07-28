"""어깨선 기하학적 구조 추정을 위한 보조 도구"""

from __future__ import annotations
import math
from .vector_math import Point3D, vector_between_points, wrap_angle_radians


def compute_shoulder_center(left_shoulder: Point3D, right_shoulder: Point3D) -> Point3D:
    """왼쪽 어깨와 오른쪽 어깨 사이의 중점을 반환"""

    return Point3D(
        x_m=(left_shoulder.x_m + right_shoulder.x_m) / 2.0,
        y_m=(left_shoulder.y_m + right_shoulder.y_m) / 2.0,
        z_m=(left_shoulder.z_m + right_shoulder.z_m) / 2.0,
    )


def compute_shoulder_yaw_rad(left_shoulder: Point3D, right_shoulder: Point3D) -> float:
    """어깨선의 요(yaw) 각도를 라디안 단위로 계산

    각도는 xy 평면상에서 왼쪽 어깨에서 오른쪽 어깨를 기준으로 계산됩니다. 
    위에서 보았을 때 반시계 방향이 양의 요(yaw) 값.
    """

    vector = vector_between_points(left_shoulder, right_shoulder)
    dx_m = vector[0]
    dy_m = vector[1]

    if dx_m == 0.0 and dy_m == 0.0:
        raise ValueError("left and right shoulder points are identical")

    yaw_rad = wrap_angle_radians(math.atan2(dy_m, dx_m))
    return yaw_rad

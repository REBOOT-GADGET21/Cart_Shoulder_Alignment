from vision.camera_types import Point3D
from vision.landmark_filter import LandmarkFilter


def test_filter_uses_median_for_large_outlier():
    filt = LandmarkFilter(alpha=1.0, median_window=3, outlier_distance_m=0.1)
    filt.update(Point3D(1.0, 0.0, 2.0))
    filt.update(Point3D(1.01, 0.0, 2.0))
    result = filt.update(Point3D(10.0, 10.0, 10.0))
    assert abs(result.x - 1.005) < 0.02

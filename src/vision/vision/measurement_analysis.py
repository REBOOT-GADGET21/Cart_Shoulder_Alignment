"""Analyze a shoulder measurement CSV exported from a rosbag2 recording.

CSV columns: stamp_s,left_x,left_y,right_x,right_y,ref_left_x,ref_left_y,
ref_right_x,ref_right_y,processing_latency_ms.  The reference columns are
the tape/marker measurements in the same odom frame and can be constant.
"""

from __future__ import annotations

import argparse
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description="Compute D435/MediaPipe shoulder error and jitter.")
    parser.add_argument("csv", help="Rosbag-exported measurement CSV")
    parser.add_argument("--label", default="trial", help="e.g. distance_2m_yaw_30")
    parser.add_argument("--output-dir", default="shoulder_measurement_results")
    args = parser.parse_args()

    import matplotlib.pyplot as plt
    import numpy as np
    import pandas as pd

    data = pd.read_csv(args.csv).dropna()
    required = {"left_x", "left_y", "right_x", "right_y", "ref_left_x", "ref_left_y", "ref_right_x", "ref_right_y"}
    missing = required - set(data.columns)
    
    if missing:
        raise SystemExit(f"Missing CSV columns: {', '.join(sorted(missing))}")
    center_x, center_y = (data.left_x+data.right_x)/2, (data.left_y+data.right_y)/2
    ref_x, ref_y = (data.ref_left_x+data.ref_right_x)/2, (data.ref_left_y+data.ref_right_y)/2
    data["x_error_m"], data["y_error_m"] = center_x-ref_x, center_y-ref_y
    angle = np.unwrap(np.arctan2(data.right_y-data.left_y, data.right_x-data.left_x))
    ref_angle = np.unwrap(np.arctan2(data.ref_right_y-data.ref_left_y, data.ref_right_x-data.ref_left_x))
    data["angle_error_deg"] = np.degrees(np.arctan2(np.sin(angle-ref_angle), np.cos(angle-ref_angle)))
    summary = pd.DataFrame([{
        "label": args.label, "frames": len(data),
        "x_error_mean_m": data.x_error_m.mean(), "x_error_std_m": data.x_error_m.std(),
        "y_error_mean_m": data.y_error_m.mean(), "y_error_std_m": data.y_error_m.std(),
        "angle_error_mean_deg": data.angle_error_deg.mean(), "angle_error_std_deg": data.angle_error_deg.std(),
        "center_jitter_std_m": np.hypot(center_x.diff(), center_y.diff()).std(),
        "latency_mean_ms": data.processing_latency_ms.mean() if "processing_latency_ms" in data else float("nan"),
    }])
    out = Path(args.output_dir); out.mkdir(parents=True, exist_ok=True)
    summary.to_csv(out / f"{args.label}_summary.csv", index=False)
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
    axes[0].plot(data.x_error_m, label="x"); axes[0].plot(data.y_error_m, label="y"); axes[0].set_ylabel("centre error [m]"); axes[0].legend()
    axes[1].plot(data.angle_error_deg); axes[1].set_ylabel("shoulder angle error [deg]"); axes[1].set_xlabel("frame")
    fig.tight_layout(); fig.savefig(out / f"{args.label}_errors.png", dpi=150)
    print(summary.to_string(index=False))


if __name__ == "__main__":
    main()

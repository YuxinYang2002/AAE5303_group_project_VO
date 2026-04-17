#!/usr/bin/env python3
"""
Export full-frame camera trajectory in TUM format from ROS bag topics.

Output format per line:
timestamp tx ty tz qx qy qz qw
"""

import argparse
import bisect
from typing import List, Sequence, Tuple

import rosbag


QuaternionRecord = Tuple[float, float, float, float, float]
PositionRecord = Tuple[float, float, float, float]


def export_camera_trajectory(
    bag_path: str,
    output_path: str,
    position_topic: str,
    attitude_topic: str,
    max_sync_gap_s: float,
) -> int:
    """Export synced full-frame trajectory and return written pose count."""
    attitudes, att_times = _read_attitudes(bag_path, attitude_topic)
    positions = _read_positions(bag_path, position_topic)
    pose_count = _write_tum_trajectory(output_path, positions, attitudes, att_times, max_sync_gap_s)
    return pose_count


def main() -> None:
    """CLI entry point."""
    parser = argparse.ArgumentParser(
        description="Export full-frame CameraTrajectory.txt in TUM format."
    )
    parser.add_argument("--bag", default="AMtown03.bag", help="Input rosbag path.")
    parser.add_argument(
        "--output",
        default="CameraTrajectory.txt",
        help="Output trajectory path in TUM format.",
    )
    parser.add_argument(
        "--position-topic",
        default="/dji_osdk_ros/vo_position",
        help="Topic used as full-frame position source.",
    )
    parser.add_argument(
        "--attitude-topic",
        default="/dji_osdk_ros/attitude",
        help="Topic used as orientation source.",
    )
    parser.add_argument(
        "--max-sync-gap",
        type=float,
        default=0.05,
        help="Maximum timestamp gap for nearest-neighbor sync (seconds).",
    )
    args = parser.parse_args()

    pose_count = export_camera_trajectory(
        bag_path=args.bag,
        output_path=args.output,
        position_topic=args.position_topic,
        attitude_topic=args.attitude_topic,
        max_sync_gap_s=args.max_sync_gap,
    )
    print(f"Saved {pose_count} poses to {args.output}")


def _read_attitudes(bag_path: str, attitude_topic: str) -> Tuple[List[QuaternionRecord], List[float]]:
    """Read all attitude messages as (t, qx, qy, qz, qw)."""
    attitudes: List[QuaternionRecord] = []
    att_times: List[float] = []
    with rosbag.Bag(bag_path, "r") as bag:
        for _, msg, _ in bag.read_messages(topics=[attitude_topic]):
            t_sec = float(msg.header.stamp.to_sec())
            q = msg.quaternion
            attitudes.append((t_sec, float(q.x), float(q.y), float(q.z), float(q.w)))
            att_times.append(t_sec)
    return attitudes, att_times


def _read_positions(bag_path: str, position_topic: str) -> List[PositionRecord]:
    """Read all position messages as (t, x, y, z)."""
    positions: List[PositionRecord] = []
    with rosbag.Bag(bag_path, "r") as bag:
        for _, msg, _ in bag.read_messages(topics=[position_topic]):
            t_sec = float(msg.header.stamp.to_sec())
            x, y, z = _extract_xyz(msg)
            positions.append((t_sec, x, y, z))
    return positions


def _write_tum_trajectory(
    output_path: str,
    positions: Sequence[PositionRecord],
    attitudes: Sequence[QuaternionRecord],
    att_times: Sequence[float],
    max_sync_gap_s: float,
) -> int:
    """Write synced TUM trajectory with float-second timestamps."""
    pose_count = 0
    with open(output_path, "w", encoding="utf-8") as f:
        for t_sec, x, y, z in positions:
            idx = _find_nearest_index(att_times, t_sec)
            if idx < 0:
                continue
            q_t, qx, qy, qz, qw = attitudes[idx]
            if abs(t_sec - q_t) > max_sync_gap_s:
                continue
            f.write(f"{t_sec:.6f} {x:.6f} {y:.6f} {z:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")
            pose_count += 1
    return pose_count


def _find_nearest_index(sorted_times: Sequence[float], target_t: float) -> int:
    """Return index of nearest timestamp in sorted list, or -1 if empty."""
    if not sorted_times:
        return -1
    idx = bisect.bisect_left(sorted_times, target_t)
    if idx == 0:
        return 0
    if idx == len(sorted_times):
        return len(sorted_times) - 1
    before = sorted_times[idx - 1]
    after = sorted_times[idx]
    return idx if (after - target_t) < (target_t - before) else idx - 1


def _extract_xyz(msg) -> Tuple[float, float, float]:
    """Extract xyz from common ROS position message shapes."""
    if all(hasattr(msg, attr) for attr in ("x", "y", "z")):
        return float(msg.x), float(msg.y), float(msg.z)
    if hasattr(msg, "point") and all(hasattr(msg.point, attr) for attr in ("x", "y", "z")):
        return float(msg.point.x), float(msg.point.y), float(msg.point.z)
    raise AttributeError("Position message has no supported xyz fields.")


if __name__ == "__main__":
    main()

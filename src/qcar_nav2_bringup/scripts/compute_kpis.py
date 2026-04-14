#!/usr/bin/env python3
"""Compute navigation KPIs from a ROS 2 bag file.

Usage:
    python3 compute_kpis.py <bag_path> [--controller NAME] [--run NUM] [--csv FILE]

Example:
    python3 compute_kpis.py rosbags/rpp_run1 --controller rpp --run 1
"""

import argparse
import csv
import math
import os
import sys

from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist


def read_bag(bag_path):
    """Read /plan, /odom, and /cmd_vel messages from a bag."""
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id="")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)

    topic_types = {}
    for info in reader.get_all_topics_and_types():
        topic_types[info.name] = info.type

    type_map = {
        "nav_msgs/msg/Path": Path,
        "nav_msgs/msg/Odometry": Odometry,
        "geometry_msgs/msg/Twist": Twist,
    }

    plans = []
    odoms = []
    cmd_vels = []

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()

        if topic == "/plan" and topic_types.get(topic) in type_map:
            msg = deserialize_message(data, type_map[topic_types[topic]])
            plans.append((timestamp_ns, msg))

        elif topic == "/odom" and topic_types.get(topic) in type_map:
            msg = deserialize_message(data, type_map[topic_types[topic]])
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            odoms.append((timestamp_ns, x, y))

        elif topic == "/cmd_vel" and topic_types.get(topic) in type_map:
            msg = deserialize_message(data, type_map[topic_types[topic]])
            odoms_t = timestamp_ns
            cmd_vels.append((timestamp_ns, msg.linear.x, msg.angular.z))

    return plans, odoms, cmd_vels


def point_to_segment_dist(px, py, ax, ay, bx, by):
    """Perpendicular distance from point (px,py) to segment (ax,ay)-(bx,by)."""
    dx = bx - ax
    dy = by - ay
    len_sq = dx * dx + dy * dy
    if len_sq < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / len_sq))
    proj_x = ax + t * dx
    proj_y = ay + t * dy
    return math.hypot(px - proj_x, py - proj_y)


def compute_cte(odoms, plan_poses):
    """Cross-track error: distance from each odom point to nearest plan segment."""
    if len(plan_poses) < 2 or not odoms:
        return 0.0, 0.0

    segments = []
    for i in range(len(plan_poses) - 1):
        segments.append((
            plan_poses[i][0], plan_poses[i][1],
            plan_poses[i + 1][0], plan_poses[i + 1][1],
        ))

    cte_values = []
    for _, ox, oy in odoms:
        min_dist = float("inf")
        for ax, ay, bx, by in segments:
            d = point_to_segment_dist(ox, oy, ax, ay, bx, by)
            if d < min_dist:
                min_dist = d
        cte_values.append(min_dist)

    rmse = math.sqrt(sum(v * v for v in cte_values) / len(cte_values))
    cte_max = max(cte_values)
    return rmse, cte_max


def compute_mission_time(cmd_vels):
    """Time between first and last cmd_vel where car is moving."""
    moving_times = [t for t, vx, _ in cmd_vels if abs(vx) > 0.01]
    if len(moving_times) < 2:
        return 0.0
    return (moving_times[-1] - moving_times[0]) / 1e9


def compute_accel_rms(cmd_vels):
    """RMS of linear and angular acceleration from cmd_vel differentiation."""
    if len(cmd_vels) < 2:
        return 0.0, 0.0

    lin_accels = []
    ang_accels = []
    for i in range(1, len(cmd_vels)):
        dt = (cmd_vels[i][0] - cmd_vels[i - 1][0]) / 1e9
        if dt < 1e-6:
            continue
        lin_accels.append((cmd_vels[i][1] - cmd_vels[i - 1][1]) / dt)
        ang_accels.append((cmd_vels[i][2] - cmd_vels[i - 1][2]) / dt)

    if not lin_accels:
        return 0.0, 0.0

    lin_rms = math.sqrt(sum(a * a for a in lin_accels) / len(lin_accels))
    ang_rms = math.sqrt(sum(a * a for a in ang_accels) / len(ang_accels))
    return lin_rms, ang_rms


def compute_path_efficiency(odoms, plan_poses):
    """Ratio of actual distance traveled to planned path length."""
    if len(odoms) < 2 or len(plan_poses) < 2:
        return 0.0

    actual = 0.0
    for i in range(1, len(odoms)):
        dx = odoms[i][1] - odoms[i - 1][1]
        dy = odoms[i][2] - odoms[i - 1][2]
        actual += math.hypot(dx, dy)

    planned = 0.0
    for i in range(1, len(plan_poses)):
        dx = plan_poses[i][0] - plan_poses[i - 1][0]
        dy = plan_poses[i][1] - plan_poses[i - 1][1]
        planned += math.hypot(dx, dy)

    if planned < 1e-6:
        return 0.0
    return actual / planned


def main():
    parser = argparse.ArgumentParser(description="Compute navigation KPIs from a rosbag")
    parser.add_argument("bag_path", help="Path to the rosbag directory")
    parser.add_argument("--controller", default="unknown", help="Controller name for CSV")
    parser.add_argument("--planner", default="smac", help="Planner name for CSV (e.g. smac, navfn)")
    parser.add_argument("--run", type=int, default=1, help="Run number for CSV")
    parser.add_argument("--csv", default="kpi_results.csv", help="CSV output file (append mode)")
    args = parser.parse_args()

    if not os.path.exists(args.bag_path):
        print(f"Error: bag path '{args.bag_path}' not found")
        sys.exit(1)

    print(f"Reading bag: {args.bag_path}")
    plans, odoms, cmd_vels = read_bag(args.bag_path)

    print(f"  /plan messages:    {len(plans)}")
    print(f"  /odom messages:    {len(odoms)}")
    print(f"  /cmd_vel messages: {len(cmd_vels)}")

    if not plans:
        print("Error: no /plan messages found in bag")
        sys.exit(1)

    # Use the last published plan as the reference
    _, ref_plan = plans[-1]
    plan_poses = [(p.pose.position.x, p.pose.position.y) for p in ref_plan.poses]
    print(f"  Reference plan: {len(plan_poses)} poses")

    # Compute KPIs
    cte_rmse, cte_max = compute_cte(odoms, plan_poses)
    mission_time = compute_mission_time(cmd_vels)
    lin_accel_rms, ang_accel_rms = compute_accel_rms(cmd_vels)
    path_eff = compute_path_efficiency(odoms, plan_poses)

    # Print results
    print()
    print("=" * 55)
    print(f"  Controller:              {args.controller}")
    print(f"  Planner:                 {args.planner}")
    print(f"  Run:                     {args.run}")
    print("-" * 55)
    print(f"  CTE RMSE:                {cte_rmse:.4f} m")
    print(f"  CTE Max:                 {cte_max:.4f} m")
    print(f"  Mission Time:            {mission_time:.2f} s")
    print(f"  Linear Accel RMS:        {lin_accel_rms:.4f} m/s^2")
    print(f"  Angular Accel RMS:       {ang_accel_rms:.4f} rad/s^2")
    print(f"  Path Efficiency:         {path_eff:.4f}")
    print("=" * 55)

    # Append to CSV
    write_header = not os.path.exists(args.csv)
    with open(args.csv, "a", newline="") as f:
        writer = csv.writer(f)
        if write_header:
            writer.writerow([
                "controller", "planner", "run", "cte_rmse", "cte_max",
                "mission_time", "linear_accel_rms", "angular_accel_rms",
                "path_efficiency",
            ])
        writer.writerow([
            args.controller,
            args.planner,
            args.run,
            f"{cte_rmse:.4f}",
            f"{cte_max:.4f}",
            f"{mission_time:.2f}",
            f"{lin_accel_rms:.4f}",
            f"{ang_accel_rms:.4f}",
            f"{path_eff:.4f}",
        ])

    print(f"\nResults appended to {args.csv}")


if __name__ == "__main__":
    main()

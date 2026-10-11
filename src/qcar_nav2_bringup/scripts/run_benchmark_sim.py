#!/usr/bin/env python3
"""Sim benchmark: N repeatable runs of one controller/planner combo.

Sim counterpart of the hardware run_benchmark_hw.sh, but fully automatic:
per run it resets the car to the spawn pose, records a bag, sends the goal
itself, cancels it if it runs past --timeout, logs the outcome, and computes
the KPIs with compute_kpis.py (same six KPIs and CSV format as hardware).

Needs simulation.yaml and navigation.launch.py (with the same controller and
planner) already running; --controller/--planner are only labels.

Output (mirrors the hardware benchmarks/<controller>_<planner>/ layout):
    <out-root>/<controller>_<planner>/rosbags/<controller>_<planner>_run<N>/
    <out-root>/kpi_results_sim.csv   KPIs, one row per run that produced a plan
    <out-root>/runs_sim.csv          outcome of every run (incl. failures)

Usage:
    python3 run_benchmark_sim.py --controller rpp --planner hybrid [--runs 5]
        [--goal 0.56 2.64 -90] [--timeout 150] [--out-root benchmarks_sim]
"""

import argparse
import csv
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node

HERE = os.path.dirname(os.path.abspath(__file__))
TOPICS = ['/odom', '/plan', '/cmd_vel', '/cmd_vel_hw']
STATUS = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
}
RUNS_FIELDS = ['controller', 'planner', 'run', 'status', 'wall_time_s',
               'final_x', 'final_y', 'final_dist_to_goal', 'bag']


class Runner(Node):
    def __init__(self):
        super().__init__('sim_benchmark_runner')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.pose = None
        self.create_subscription(Odometry, '/odom', self._odom, 10)

    def _odom(self, msg):
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def spin_for(self, secs):
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def navigate(self, gx, gy, gyaw_deg, timeout):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = float(gx)
        goal.pose.pose.position.y = float(gy)
        goal.pose.pose.orientation.z = math.sin(math.radians(gyaw_deg) / 2)
        goal.pose.pose.orientation.w = math.cos(math.radians(gyaw_deg) / 2)

        send = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10)
        handle = send.result()
        if handle is None or not handle.accepted:
            return 'REJECTED'
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=timeout)
        if result.done():
            return STATUS.get(result.result().status, 'STATUS_{}'.format(result.result().status))
        cancel = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel, timeout_sec=10)
        rclpy.spin_until_future_complete(self, result, timeout_sec=10)
        return 'TIMEOUT'


def reset_pose():
    subprocess.run(
        ['gz', 'service', '-s', '/world/qcar_hwmap/set_pose',
         '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean', '--timeout', '3000',
         '--req', 'name: "qcar", position: {x: 0, y: 0, z: 0}, orientation: {w: 1}'],
        check=True, stdout=subprocess.DEVNULL)


def append_row(path, fields, row):
    new = not os.path.exists(path)
    with open(path, 'a', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        if new:
            w.writeheader()
        w.writerow(row)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--controller', required=True)
    p.add_argument('--planner', required=True)
    p.add_argument('--runs', type=int, default=5)
    p.add_argument('--first-run', type=int, default=1)
    p.add_argument('--goal', type=float, nargs=3, default=[0.56, 2.64, -90.0],
                   metavar=('X', 'Y', 'YAW_DEG'))
    p.add_argument('--timeout', type=float, default=150.0, help='seconds per run')
    p.add_argument('--out-root', default='benchmarks_sim')
    args = p.parse_args()

    combo = '{}_{}'.format(args.controller, args.planner)
    bags_dir = os.path.join(args.out_root, combo, 'rosbags')
    os.makedirs(bags_dir, exist_ok=True)
    kpi_csv = os.path.join(args.out_root, 'kpi_results_sim.csv')
    runs_csv = os.path.join(args.out_root, 'runs_sim.csv')
    gx, gy, gyaw = args.goal

    rclpy.init()
    node = Runner()
    if not node.client.wait_for_server(timeout_sec=30):
        print('navigate_to_pose not available -- is navigation.launch.py running?')
        sys.exit(1)

    for run in range(args.first_run, args.first_run + args.runs):
        bag = os.path.join(bags_dir, '{}_run{}'.format(combo, run))
        if os.path.exists(bag):
            print('{} exists, skipping run {}'.format(bag, run))
            continue
        print('--- {} run {}'.format(combo, run), flush=True)

        reset_pose()
        node.spin_for(3.0)  # emulator timeout + costmaps settle at the start pose

        rec = subprocess.Popen(['ros2', 'bag', 'record', '-o', bag] + TOPICS,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                               start_new_session=True)
        # Jazzy's planner_server publishes /plan once, unlatched: if the
        # recorder isn't matched yet the plan is lost. Wait until it is
        # subscribed to every topic before sending the goal.
        deadline = time.time() + 15.0
        while time.time() < deadline and \
                not all(node.count_subscribers(t) >= 1 for t in TOPICS):
            node.spin_for(0.2)
        node.spin_for(1.5)

        t0 = time.time()
        status = node.navigate(gx, gy, gyaw, args.timeout)
        wall = time.time() - t0
        node.spin_for(1.0)

        # SIGTERM: rosbag2 closes the bag cleanly (SIGINT may be ignored when
        # this script itself runs as a background job).
        os.killpg(rec.pid, signal.SIGTERM)
        try:
            rec.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(rec.pid, signal.SIGKILL)

        fx, fy = node.pose if node.pose else (float('nan'), float('nan'))
        dist = math.hypot(fx - gx, fy - gy)
        print('    {}  {:.1f} s  final ({:.2f}, {:.2f})  {:.2f} m from goal'.format(
            status, wall, fx, fy, dist), flush=True)
        append_row(runs_csv, RUNS_FIELDS, {
            'controller': args.controller, 'planner': args.planner, 'run': run,
            'status': status, 'wall_time_s': '{:.1f}'.format(wall),
            'final_x': '{:.3f}'.format(fx), 'final_y': '{:.3f}'.format(fy),
            'final_dist_to_goal': '{:.3f}'.format(dist), 'bag': bag})

        kpi = subprocess.run(
            [sys.executable, os.path.join(HERE, 'compute_kpis.py'), bag,
             '--controller', args.controller, '--planner', args.planner,
             '--run', str(run), '--csv', kpi_csv],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        if kpi.returncode != 0:
            print('    KPIs not computed: ' + kpi.stdout.strip().splitlines()[-1], flush=True)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

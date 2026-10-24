#!/usr/bin/env python3
"""Compile the hardware benchmark bags into the same layout as the sim benchmark.

Input: a folder with one sub-folder per combo, named <controller>_<planner>
(e.g. rpp_navfn, stanley_navfn, vpp_navfn). Bags can sit at any depth below
it and their own names are ignored -- several were recorded under the wrong
label. Runs are numbered by recording time within each combo.

Every bag is read straight from its .db3 (some metadata.yaml files point at
the wrong .db3), and the KPIs come from compute_kpis_hw.py, a verbatim copy
of the hardware script (qcar main 8c859f3), so the math is the hardware's.

Each run is classified:
    REACHED          started at the start pose, ended within --goal-tol of the goal
    REACHED_NO_PLAN  same, but /plan was not recorded: mission time and
                     accelerations only (CTE and path efficiency need the plan)
    NOT_REACHED      started at the start pose but did not end at the goal
    EXCLUDED         not a full run (car never moved, or started elsewhere)

Output (mirrors benchmarks_sim/):
    <out>/<controller>_<planner>/rosbags/<controller>_<planner>_run<N>/
    <out>/kpi_results_hw.csv   KPIs, one row per REACHED / REACHED_NO_PLAN run
    <out>/runs_hw.csv          classification of every bag found

Usage:
    python3 compile_hw_results.py [~/Music] [--out benchmarks_hw]
    python3 summarize_benchmark_sim.py benchmarks_hw --tag hw
"""

import argparse
import csv
import glob
import math
import os
import shutil
import sqlite3
import sys

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import compute_kpis_hw as kpi  # noqa: E402

# Folder label -> name used in the sim benchmark
CONTROLLER_ALIASES = {'vpp': 'vector', 'vpc': 'vector'}
START = (0.0, 0.0)          # map frame, same start and goal as the sim benchmark
GOAL = (0.56, 2.64)
KPI_FIELDS = ['controller', 'planner', 'run', 'cte_rmse', 'cte_max', 'mission_time',
              'linear_accel_rms', 'angular_accel_rms', 'path_efficiency']
RUNS_FIELDS = ['controller', 'planner', 'run', 'status', 'note', 'has_plan',
               'start_x', 'start_y', 'final_x', 'final_y', 'final_dist_to_goal',
               'source', 'bag']


def bag_start_ns(bag_dir):
    t = None
    for db in glob.glob(os.path.join(bag_dir, '*.db3')):
        conn = sqlite3.connect(db)
        try:
            row = conn.execute('SELECT MIN(timestamp) FROM messages').fetchone()
        finally:
            conn.close()
        if row and row[0] is not None:
            t = row[0] if t is None else min(t, row[0])
    return t


def copy_bag(src, dst):
    """Copy a bag, pointing metadata.yaml at the .db3 files actually present."""
    os.makedirs(dst)
    dbs = sorted(os.path.basename(p) for p in glob.glob(os.path.join(src, '*.db3')))
    for name in dbs:
        shutil.copy2(os.path.join(src, name), os.path.join(dst, name))
    meta = os.path.join(src, 'metadata.yaml')
    if os.path.exists(meta):
        with open(meta) as f:
            data = yaml.safe_load(f)
        info = data['rosbag2_bagfile_information']
        if sorted(info.get('relative_file_paths', [])) != dbs:
            info['relative_file_paths'] = dbs
        with open(os.path.join(dst, 'metadata.yaml'), 'w') as f:
            yaml.safe_dump(data, f, sort_keys=False)


def evaluate(bag_dir, goal_tol, start_tol):
    plans, odoms, cmd_vels, odom_frame = kpi.read_bag_sqlite(
        bag_dir, '/plan', '/odom_opti', '/cmd_vel')
    if odom_frame == 'odom':
        odoms = kpi.odom_to_map(odoms, kpi.DEFAULT_MAP_ODOM_X, kpi.DEFAULT_MAP_ODOM_Y,
                                kpi.DEFAULT_MAP_ODOM_YAW)
    row = {'has_plan': bool(plans)}
    if not odoms:
        row.update(status='EXCLUDED', note='no odometry')
        return row, None
    (_, sx, sy), (_, fx, fy) = odoms[0], odoms[-1]
    dist = math.hypot(fx - GOAL[0], fy - GOAL[1])
    row.update(start_x='{:.3f}'.format(sx), start_y='{:.3f}'.format(sy),
               final_x='{:.3f}'.format(fx), final_y='{:.3f}'.format(fy),
               final_dist_to_goal='{:.3f}'.format(dist))

    if not any(abs(v) > 0.01 for _, v, _ in cmd_vels):
        row.update(status='EXCLUDED', note='car never commanded to move')
        return row, None
    if math.hypot(sx - START[0], sy - START[1]) > start_tol:
        row.update(status='EXCLUDED', note='did not start at the start pose')
        return row, None
    if dist > goal_tol:
        row.update(status='NOT_REACHED', note='')
        return row, None

    lin, ang = kpi.compute_accel_rms(cmd_vels)
    values = {'mission_time': '{:.2f}'.format(kpi.compute_mission_time(cmd_vels)),
              'linear_accel_rms': '{:.4f}'.format(lin),
              'angular_accel_rms': '{:.4f}'.format(ang)}
    if plans:
        _, _, plan = plans[-1]
        rmse, cmax = kpi.compute_cte(odoms, plan)
        values.update(cte_rmse='{:.4f}'.format(rmse), cte_max='{:.4f}'.format(cmax),
                      path_efficiency='{:.4f}'.format(kpi.compute_path_efficiency(odoms, plan)))
        row.update(status='REACHED', note='')
    else:
        values.update(cte_rmse='', cte_max='', path_efficiency='')
        row.update(status='REACHED_NO_PLAN', note='/plan not recorded')
    return row, values


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('source', nargs='?', default=os.path.expanduser('~/Music'))
    p.add_argument('--out', default='benchmarks_hw')
    p.add_argument('--goal-tol', type=float, default=0.25, help='m, final pose to goal')
    p.add_argument('--start-tol', type=float, default=0.3, help='m, first pose to start')
    args = p.parse_args()

    if os.path.exists(args.out):
        sys.exit('{} already exists -- remove it first'.format(args.out))
    kpi_rows, run_rows = [], []

    for combo_dir in sorted(glob.glob(os.path.join(args.source, '*_*'))):
        if not os.path.isdir(combo_dir):
            continue
        label, _, planner = os.path.basename(combo_dir).partition('_')
        controller = CONTROLLER_ALIASES.get(label, label)
        bags = {os.path.dirname(db) for db in
                glob.glob(os.path.join(combo_dir, '**', '*.db3'), recursive=True)}
        if not bags:
            continue
        combo = '{}_{}'.format(controller, planner)
        print('*', combo, '<-', combo_dir)

        for run, src in enumerate(sorted(bags, key=bag_start_ns), start=1):
            dst = os.path.join(args.out, combo, 'rosbags', '{}_run{}'.format(combo, run))
            copy_bag(src, dst)
            row, values = evaluate(dst, args.goal_tol, args.start_tol)
            row.update(controller=controller, planner=planner, run=run,
                       source=os.path.relpath(src, args.source), bag=dst)
            run_rows.append(row)
            if values:
                values.update(controller=controller, planner=planner, run=run)
                kpi_rows.append(values)
            print('    run {}  {:<16} {}  ({})'.format(run, row['status'], row['note'],
                                                    row['source']))

    for name, fields, rows in (('kpi_results_hw.csv', KPI_FIELDS, kpi_rows),
                               ('runs_hw.csv', RUNS_FIELDS, run_rows)):
        with open(os.path.join(args.out, name), 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            w.writerows(rows)
    print('Wrote', os.path.join(args.out, 'kpi_results_hw.csv'), 'and runs_hw.csv')


if __name__ == '__main__':
    main()

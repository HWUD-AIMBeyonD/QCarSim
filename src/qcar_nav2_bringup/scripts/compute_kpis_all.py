#!/usr/bin/env python3
"""One KPI calculation for sim and hardware bags: the numbers to compare.

Reads every counted run of both benchmarks and computes the KPIs with the
same code and the same preprocessing, so differences come from the runs,
not from the scripts:

    runs counted   sim: status SUCCEEDED (runs_sim.csv)
                   hw:  status REACHED / REACHED_NO_PLAN (runs_hw.csv)
    pose           sim: /odom (map->odom is identity in the sim)
                   hw:  /odom_opti, moved into the map frame with the static
                        map->odom transform (as compute_kpis_hw.py does)
    reference      the last /plan in the bag (one plan per run)
    mission window first to last /cmd_vel with |v| > 0.01 (as mission time)

KPIs (math from compute_kpis_hw.py, the hardware script):
    cte_rmse, cte_max   distance from each pose to the plan polyline,
                        poses inside the mission window only (the hardware
                        recordings idle for 5-15 s at the start pose, the
                        sim ones ~2 s; idle samples would dilute the RMSE)
    mission_time        s, mission window length
    path_efficiency     driven length / plan length, both de-noised the same
                        way: poses resampled at 10 Hz (OptiTrack jitter) and
                        the plan thinned to >= 0.1 m spacing (Jazzy NavFn's
                        1-cell zigzag adds ~3% length that Dashing's lacks)
    linear/angular_accel_rms   differentiated /cmd_vel (both 20 Hz)
    avg_speed           m/s, driven length / mission time
    final_dist_to_goal  m, last pose to the last plan pose

Output (<out>, default benchmarks_compare/):
    kpi_results_all.csv   one row per run, column 'source' = sim | hw
    summary_all.csv       mean ± sample std per source x controller x planner
    sim_vs_hw.csv         per controller x KPI: sim vs hw, gap and gap %
    sim_vs_hw_<planner>.png      planners run on both
    sim_<planner>_vs_<planner>.png   the sim planners side by side

Usage:
    python3 compute_kpis_all.py [--sim benchmarks_sim] [--hw benchmarks_hw]
                                [--out benchmarks_compare]
"""

import argparse
import csv
import glob
import math
import os
import sqlite3
import sys
from collections import defaultdict

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import compute_kpis_hw as kpi  # noqa: E402
from summarize_benchmark_sim import (CONTROLLERS, CONTROLLER_LABELS, GRID, INK, INK_2,  # noqa: E402
                                     PLANNER_COLORS, PLANNER_LABELS, PLANNERS, SURFACE,
                                     read_csv)

SOURCES = {
    'sim': {'runs': 'runs_sim.csv', 'counted': ('SUCCEEDED',), 'odom': '/odom',
            'map_odom': (0.0, 0.0, 0.0)},
    'hw': {'runs': 'runs_hw.csv', 'counted': ('REACHED', 'REACHED_NO_PLAN'),
           'odom': '/odom_opti',
           'map_odom': (kpi.DEFAULT_MAP_ODOM_X, kpi.DEFAULT_MAP_ODOM_Y,
                        kpi.DEFAULT_MAP_ODOM_YAW)},
}
SOURCE_LABELS = {'sim': 'Sim', 'hw': 'Hardware'}
# Categorical slots 3-4 (slots 1-2 already mean Hybrid A* / NavFn), validated
# on the light surface; contrast < 3:1, so the CSVs are the table view.
SOURCE_COLORS = {'sim': '#1baf7a', 'hw': '#eda100'}

ODOM_RATE = 10.0    # Hz, resampling for driven length
PLAN_SPACING = 0.1  # m, plan thinning for plan length
MOVING = 0.01       # m/s, as compute_mission_time()

KPIS = [
    ('cte_rmse', 'CTE RMSE (m)'),
    ('cte_max', 'CTE max (m)'),
    ('mission_time', 'Mission time (s)'),
    ('path_efficiency', 'Path efficiency (driven / planned)'),
    ('linear_accel_rms', 'Linear accel RMS (m/s²)'),
    ('angular_accel_rms', 'Angular accel RMS (rad/s²)'),
    ('avg_speed', 'Average speed (m/s)'),
    ('final_dist_to_goal', 'Final distance to goal (m)'),
]
PLOT_KPIS = KPIS[:7]


def bag_topics(bag):
    """Topic names in a bag: from the .db3 itself, else metadata.yaml."""
    dbs = glob.glob(os.path.join(bag, '*.db3'))
    if dbs:
        conn = sqlite3.connect(dbs[0])
        try:
            return {name for (name,) in conn.execute('SELECT name FROM topics')}
        finally:
            conn.close()
    with open(os.path.join(bag, 'metadata.yaml')) as f:
        info = yaml.safe_load(f)['rosbag2_bagfile_information']
    return {t['topic_metadata']['name'] for t in info['topics_with_message_count']}


def read_bag(bag, odom_topic):
    # Dashing bags (.db3, metadata v1) are read straight from sqlite: Jazzy's
    # rosbag2 resolves their file paths differently and cannot open them.
    if glob.glob(os.path.join(bag, '*.db3')):
        return kpi.read_bag_sqlite(bag, '/plan', odom_topic, '/cmd_vel')
    return kpi.read_bag_rosbag2_py(bag, '/plan', odom_topic, '/cmd_vel')


def resampled_length(odoms, rate):
    t = np.array([o[0] for o in odoms], float) / 1e9
    if len(t) < 2:
        return 0.0
    ts = np.arange(t[0], t[-1], 1.0 / rate)
    x = np.interp(ts, t, [o[1] for o in odoms])
    y = np.interp(ts, t, [o[2] for o in odoms])
    return float(np.sum(np.hypot(np.diff(x), np.diff(y))))


def thinned_length(plan, spacing):
    pts = [plan[0]]
    for q in plan[1:]:
        if math.hypot(q[0] - pts[-1][0], q[1] - pts[-1][1]) >= spacing:
            pts.append(q)
    if pts[-1] != plan[-1]:
        pts.append(plan[-1])
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))


def compute(bag, src):
    cfg = SOURCES[src]
    odom_topic = cfg['odom'] if cfg['odom'] in bag_topics(bag) else '/odom'
    plans, odoms, cmd_vels, _ = read_bag(bag, odom_topic)
    odoms = kpi.odom_to_map(odoms, *cfg['map_odom'])

    moving = [t for t, v, _ in cmd_vels if abs(v) > MOVING]
    t0, t1 = moving[0], moving[-1]
    window = [o for o in odoms if t0 <= o[0] <= t1]
    mission_time = (t1 - t0) / 1e9
    driven = resampled_length(window, ODOM_RATE)
    lin, ang = kpi.compute_accel_rms(cmd_vels)
    row = {'mission_time': mission_time, 'linear_accel_rms': lin, 'angular_accel_rms': ang,
           'avg_speed': driven / mission_time if mission_time > 0 else float('nan')}
    if plans:
        _, _, plan = plans[-1]
        rmse, cmax = kpi.compute_cte(window, plan)
        row.update(cte_rmse=rmse, cte_max=cmax,
                   path_efficiency=driven / thinned_length(plan, PLAN_SPACING),
                   final_dist_to_goal=math.hypot(odoms[-1][1] - plan[-1][0],
                                                 odoms[-1][2] - plan[-1][1]))
    return row


def stats(values):
    if not values:
        return float('nan'), float('nan')
    return float(np.mean(values)), float(np.std(values, ddof=1)) if len(values) > 1 else 0.0


def plot(groups, title, path):
    """groups: [(label, color, {(controller): {kpi: [values]}}, show_runs)]"""
    plt.rcParams.update({'font.size': 9, 'text.color': INK, 'axes.labelcolor': INK_2,
                         'xtick.color': INK_2, 'ytick.color': INK_2})
    fig, axes = plt.subplots(2, 4, figsize=(15, 6.5), facecolor=SURFACE)
    x = np.arange(len(CONTROLLERS))
    width = 0.36
    for ax, (k, label) in zip(axes.flat, PLOT_KPIS):
        ax.set_facecolor(SURFACE)
        for i, (glabel, color, data, show_runs) in enumerate(groups):
            vals = [data[c][k] for c in CONTROLLERS]
            means = [stats(v)[0] for v in vals]
            stds = [stats(v)[1] if v else 0 for v in vals]
            offset = (i - (len(groups) - 1) / 2) * (width + 0.02)  # 2px-style surface gap
            ax.bar(x + offset, means, width, yerr=stds, color=color, label=glabel, capsize=3,
                   error_kw={'elinewidth': 1, 'ecolor': INK_2, 'capthick': 1})
            if show_runs:
                for xi, v in zip(x, vals):
                    ax.scatter(np.full(len(v), xi + offset), v, s=12, color=INK, zorder=3,
                               linewidths=0, label='{} run'.format(glabel) if xi == 0 else None)
        ax.set_title(label, fontsize=10, color=INK, loc='left')
        ax.set_xticks(x)
        ax.set_xticklabels([CONTROLLER_LABELS[c] for c in CONTROLLERS])
        ax.grid(axis='y', color=GRID, linewidth=0.8)
        ax.set_axisbelow(True)
        for side in ('top', 'right', 'left'):
            ax.spines[side].set_visible(False)
        ax.spines['bottom'].set_color(GRID)
        ax.tick_params(axis='y', length=0)
    axes.flat[-1].axis('off')
    handles, labels = axes.flat[0].get_legend_handles_labels()
    axes.flat[-1].legend(handles, labels, loc='center', frameon=False)
    fig.suptitle(title, x=0.01, ha='left', fontsize=12, color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(path, dpi=130, facecolor=SURFACE)
    plt.close(fig)
    print('Wrote', path)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--sim', default='benchmarks_sim')
    p.add_argument('--hw', default='benchmarks_hw')
    p.add_argument('--out', default='benchmarks_compare')
    args = p.parse_args()
    os.makedirs(args.out, exist_ok=True)
    roots = {'sim': args.sim, 'hw': args.hw}

    rows = []
    data = defaultdict(lambda: defaultdict(lambda: defaultdict(list)))  # [src][(c, p)][kpi]
    outcome = defaultdict(lambda: [0, 0])                               # (src, c, p) -> ok, total
    for src, cfg in SOURCES.items():
        for run in read_csv(os.path.join(roots[src], cfg['runs'])):
            combo = (run['controller'], run['planner'])
            if run['status'] == 'EXCLUDED':  # hardware: not an attempt
                continue
            outcome[(src,) + combo][1] += 1
            if run['status'] not in cfg['counted']:
                continue
            outcome[(src,) + combo][0] += 1
            # runs_*.csv store '<root>/<combo>/rosbags/...': re-root it so a
            # renamed or moved benchmark folder still works
            bag = os.path.join(roots[src], *run['bag'].split('/')[1:])
            values = compute(bag, src)
            for k, _ in KPIS:
                if k in values:
                    data[src][combo][k].append(values[k])
            rows.append(dict({'source': src, 'controller': combo[0], 'planner': combo[1],
                              'run': run['run']},
                             **{k: '{:.4f}'.format(values[k]) if k in values else ''
                                for k, _ in KPIS}))
            print('{:<4} {}_{} run {}'.format(src, combo[0], combo[1], run['run']))

    path = os.path.join(args.out, 'kpi_results_all.csv')
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['source', 'controller', 'planner', 'run']
                           + [k for k, _ in KPIS])
        w.writeheader()
        w.writerows(rows)
    print('Wrote', path)

    path = os.path.join(args.out, 'summary_all.csv')
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['source', 'controller', 'planner', 'reached'] + [label for _, label in KPIS])
        for src in SOURCES:
            for pl in PLANNERS:
                for c in CONTROLLERS:
                    ok, total = outcome[(src, c, pl)]
                    if not total:
                        continue
                    vals = data[src][(c, pl)]
                    n = max([len(v) for v in vals.values()] or [0])
                    cells = []
                    for k, _ in KPIS:
                        if not vals[k]:
                            cells.append('-')
                            continue
                        m, s = stats(vals[k])
                        cells.append('{:.3f} ± {:.3f}'.format(m, s)
                                     + (' (n={})'.format(len(vals[k])) if len(vals[k]) < n else ''))
                    w.writerow([SOURCE_LABELS[src], CONTROLLER_LABELS[c], PLANNER_LABELS[pl],
                                '{}/{}'.format(ok, total)] + cells)
    print('Wrote', path)

    shared = [pl for pl in PLANNERS if any(c[1] == pl for c in data['sim'])
              and any(c[1] == pl for c in data['hw'])]
    path = os.path.join(args.out, 'sim_vs_hw.csv')
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['controller', 'planner', 'kpi', 'sim_mean', 'sim_std', 'sim_n',
                    'hw_mean', 'hw_std', 'hw_n', 'gap_hw_minus_sim', 'gap_pct_of_sim'])
        for pl in shared:
            for c in CONTROLLERS:
                for k, label in KPIS:
                    sv, hv = data['sim'][(c, pl)][k], data['hw'][(c, pl)][k]
                    (sm, ss), (hm, hs) = stats(sv), stats(hv)
                    w.writerow([CONTROLLER_LABELS[c], PLANNER_LABELS[pl], label,
                                '{:.4f}'.format(sm), '{:.4f}'.format(ss), len(sv),
                                '{:.4f}'.format(hm), '{:.4f}'.format(hs), len(hv),
                                '{:+.4f}'.format(hm - sm),
                                '{:+.1f}'.format(100 * (hm - sm) / sm) if sm else ''])
    print('Wrote', path)

    for pl in shared:
        plot([(SOURCE_LABELS[src], SOURCE_COLORS[src],
               {c: data[src][(c, pl)] for c in CONTROLLERS}, src == 'hw')
              for src in ('sim', 'hw')],
             'Sim vs hardware, {} (mean ± sample std)'.format(PLANNER_LABELS[pl]),
             os.path.join(args.out, 'sim_vs_hw_{}.png'.format(pl)))
    sim_planners = [pl for pl in PLANNERS if any(c[1] == pl for c in data['sim'])]
    if len(sim_planners) > 1:
        plot([(PLANNER_LABELS[pl], PLANNER_COLORS[pl],
               {c: data['sim'][(c, pl)] for c in CONTROLLERS}, False)
              for pl in sim_planners],
             'Sim: {} (mean ± sample std)'.format(' vs '.join(PLANNER_LABELS[pl]
                                                              for pl in sim_planners)),
             os.path.join(args.out, 'sim_{}.png'.format('_vs_'.join(sim_planners))))


if __name__ == '__main__':
    main()

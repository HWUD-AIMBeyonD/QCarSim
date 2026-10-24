#!/usr/bin/env python3
"""Summarise a sim benchmark (output of run_all_sim.sh / run_benchmark_sim.py).

Reads <root>/kpi_results_<tag>.csv and <root>/runs_<tag>.csv and writes:
    <root>/summary_table.csv   mean ± sample std per controller/planner, success rate
    <root>/kpi_comparison.png  one panel per KPI, controllers x planners

Also used for the hardware results (compile_hw_results.py): --tag hw. Empty
KPI cells (hardware runs without a recorded /plan) are skipped, and a cell
built from fewer runs than the others in its row shows its n.

Usage:
    python3 summarize_benchmark_sim.py [benchmarks_sim] [--tag sim]
    python3 summarize_benchmark_sim.py benchmarks_hw --tag hw
"""

import argparse

import csv
import os
from collections import defaultdict

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

CONTROLLERS = ['rpp', 'stanley', 'vector']
CONTROLLER_LABELS = {'rpp': 'RPP', 'stanley': 'Stanley', 'vector': 'Vector Pursuit'}
PLANNERS = ['hybrid', 'navfn']
PLANNER_LABELS = {'hybrid': 'Hybrid A*', 'navfn': 'NavFn'}
# Categorical slots 1-2 of the default palette, validated on the light surface.
PLANNER_COLORS = {'hybrid': '#2a78d6', 'navfn': '#eb6834'}
SURFACE = '#fcfcfb'
INK = '#0b0b0b'
INK_2 = '#52514e'
GRID = '#e4e3df'

KPIS = [
    ('cte_rmse', 'CTE RMSE (m)'),
    ('cte_max', 'CTE max (m)'),
    ('mission_time', 'Mission time (s)'),
    ('path_efficiency', 'Path efficiency (driven / planned)'),
    ('linear_accel_rms', 'Linear accel RMS (m/s²)'),
    ('angular_accel_rms', 'Angular accel RMS (rad/s²)'),
]


def read_csv(path):
    if not os.path.exists(path):
        return []
    with open(path) as f:
        return list(csv.DictReader(f))


def std(values):
    return float(np.std(values, ddof=1)) if len(values) > 1 else 0.0


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('root', nargs='?', default='benchmarks_sim')
    parser.add_argument('--tag', default='sim', help='sim or hw (CSV name suffix)')
    args = parser.parse_args()
    root = args.root
    kpis = read_csv(os.path.join(root, 'kpi_results_{}.csv'.format(args.tag)))
    runs = read_csv(os.path.join(root, 'runs_{}.csv'.format(args.tag)))

    data = defaultdict(lambda: defaultdict(list))
    for row in kpis:
        for k, _ in KPIS:
            if row[k] != '':
                data[(row['controller'], row['planner'])][k].append(float(row[k]))

    outcome = defaultdict(lambda: [0, 0])  # combo -> [succeeded, total]
    for row in runs:
        if row['status'] == 'EXCLUDED':  # hardware: not a real attempt (see runs_hw.csv)
            continue
        c = outcome[(row['controller'], row['planner'])]
        c[1] += 1
        c[0] += row['status'] in ('SUCCEEDED', 'REACHED', 'REACHED_NO_PLAN')

    # --- table ---
    planners = [p for p in PLANNERS if any(c[1] == p for c in data)]
    table = os.path.join(root, 'summary_table.csv')
    with open(table, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['controller', 'planner', 'success', 'n_kpi'] + [label for _, label in KPIS])
        for p in planners:
            for c in CONTROLLERS:
                ok, total = outcome[(c, p)]
                vals = data[(c, p)]
                n = max(len(vals[k]) for k, _ in KPIS)
                cells = []
                for k, _ in KPIS:
                    if not vals[k]:
                        cells.append('-')
                        continue
                    cell = '{:.3f} ± {:.3f}'.format(np.mean(vals[k]), std(vals[k]))
                    if len(vals[k]) < n:
                        cell += ' (n={})'.format(len(vals[k]))
                    cells.append(cell)
                w.writerow([CONTROLLER_LABELS[c], PLANNER_LABELS[p], '{}/{}'.format(ok, total), n]
                           + cells)
    print('Wrote', table)

    # --- figure ---
    plt.rcParams.update({'font.size': 9, 'text.color': INK, 'axes.labelcolor': INK_2,
                         'xtick.color': INK_2, 'ytick.color': INK_2})
    fig, axes = plt.subplots(2, 3, figsize=(12, 6.5), facecolor=SURFACE)
    x = np.arange(len(CONTROLLERS))
    width = 0.36
    for ax, (k, label) in zip(axes.flat, KPIS):
        ax.set_facecolor(SURFACE)
        for i, p in enumerate(planners):
            means = [np.mean(data[(c, p)][k]) if data[(c, p)][k] else np.nan for c in CONTROLLERS]
            stds = [std(data[(c, p)][k]) for c in CONTROLLERS]
            # small surface gap between paired bars
            offset = (i - (len(planners) - 1) / 2) * (width + 0.02)
            ax.bar(x + offset, means, width, yerr=stds, color=PLANNER_COLORS[p],
                   label=PLANNER_LABELS[p], capsize=3,
                   error_kw={'elinewidth': 1, 'ecolor': INK_2, 'capthick': 1})
        ax.set_title(label, fontsize=10, color=INK, loc='left')
        ax.set_xticks(x)
        ax.set_xticklabels([CONTROLLER_LABELS[c] for c in CONTROLLERS])
        ax.grid(axis='y', color=GRID, linewidth=0.8)
        ax.set_axisbelow(True)
        for side in ('top', 'right', 'left'):
            ax.spines[side].set_visible(False)
        ax.spines['bottom'].set_color(GRID)
        ax.tick_params(axis='y', length=0)
    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper right', ncol=2, frameon=False)
    n_runs = max([len(v['cte_rmse']) for v in data.values()] or [0])
    title = {'sim': 'Sim', 'hw': 'Hardware'}.get(args.tag, args.tag)
    fig.suptitle('{} benchmark: controller × planner (mean ± sample std, up to {} runs each)'.format(
                 title, n_runs),
                 x=0.01, ha='left', fontsize=12, color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    png = os.path.join(root, 'kpi_comparison.png')
    fig.savefig(png, dpi=130, facecolor=SURFACE)
    print('Wrote', png)


if __name__ == '__main__':
    main()

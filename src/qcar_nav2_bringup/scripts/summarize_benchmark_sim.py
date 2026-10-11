#!/usr/bin/env python3
"""Summarise a sim benchmark (output of run_all_sim.sh / run_benchmark_sim.py).

Reads <root>/kpi_results_sim.csv and <root>/runs_sim.csv and writes:
    <root>/summary_table.csv   mean ± std per controller/planner, success rate
    <root>/kpi_comparison.png  one panel per KPI, controllers x planners

Usage:
    python3 summarize_benchmark_sim.py [benchmarks_sim]
"""

import csv
import os
import sys
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


def main():
    root = sys.argv[1] if len(sys.argv) > 1 else 'benchmarks_sim'
    kpis = read_csv(os.path.join(root, 'kpi_results_sim.csv'))
    runs = read_csv(os.path.join(root, 'runs_sim.csv'))

    data = defaultdict(lambda: defaultdict(list))
    for row in kpis:
        for k, _ in KPIS:
            data[(row['controller'], row['planner'])][k].append(float(row[k]))

    outcome = defaultdict(lambda: [0, 0])  # combo -> [succeeded, total]
    for row in runs:
        c = outcome[(row['controller'], row['planner'])]
        c[1] += 1
        c[0] += row['status'] == 'SUCCEEDED'

    # --- table ---
    table = os.path.join(root, 'summary_table.csv')
    with open(table, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['controller', 'planner', 'success', 'n_kpi'] + [label for _, label in KPIS])
        for p in PLANNERS:
            for c in CONTROLLERS:
                ok, total = outcome[(c, p)]
                vals = data[(c, p)]
                n = len(vals['cte_rmse'])
                cells = ['{:.3f} ± {:.3f}'.format(np.mean(vals[k]), np.std(vals[k])) if n else '-'
                         for k, _ in KPIS]
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
        for i, p in enumerate(PLANNERS):
            means = [np.mean(data[(c, p)][k]) if data[(c, p)][k] else np.nan for c in CONTROLLERS]
            stds = [np.std(data[(c, p)][k]) if data[(c, p)][k] else 0 for c in CONTROLLERS]
            offset = (i - 0.5) * (width + 0.02)  # small surface gap between paired bars
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
    fig.suptitle('Sim benchmark: controller × planner (mean ± std, up to {} runs each)'.format(n_runs),
                 x=0.01, ha='left', fontsize=12, color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    png = os.path.join(root, 'kpi_comparison.png')
    fig.savefig(png, dpi=130, facecolor=SURFACE)
    print('Wrote', png)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Generate benchmark charts from KPI CSV data.

Usage:
    python3 plot_kpis.py kpi_results.csv [--output-dir plots/]
"""

import argparse
import csv
import math
import os
import sys
from collections import defaultdict

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

# Color scheme: controller -> base color
CONTROLLER_COLORS = {
    "rpp": "#1f77b4",      # blue
    "stanley": "#ff7f0e",   # orange
    "vpc": "#2ca02c",       # green
    "dwb": "#d62728",       # red
}

# Planner -> hatch pattern (solid for smac, hatched for navfn)
PLANNER_HATCHES = {
    "smac": "",
    "navfn": "///",
}

KPI_LABELS = {
    "cte_rmse": "CTE RMSE (m)",
    "cte_max": "CTE Max (m)",
    "mission_time": "Mission Time (s)",
    "linear_accel_rms": "Linear Accel RMS (m/s\u00b2)",
    "angular_accel_rms": "Angular Accel RMS (rad/s\u00b2)",
    "path_efficiency": "Path Efficiency",
}


def load_csv(csv_path):
    """Load KPI CSV and return list of dicts."""
    rows = []
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            row["run"] = int(row["run"])
            for k in ["cte_rmse", "cte_max", "mission_time",
                       "linear_accel_rms", "angular_accel_rms", "path_efficiency"]:
                row[k] = float(row[k])
            rows.append(row)
    return rows


def group_by_controller_planner(rows):
    """Group rows by (controller, planner). Returns dict of lists."""
    groups = defaultdict(list)
    for r in rows:
        groups[(r["controller"], r["planner"])].append(r)
    return groups


def get_color(controller):
    return CONTROLLER_COLORS.get(controller, "#888888")


def get_hatch(planner):
    return PLANNER_HATCHES.get(planner, "xx")


def make_label(controller, planner):
    return f"{controller} / {planner}"


def bar_chart(groups, kpi, title, ylabel, output_dir):
    """Grouped bar chart for a single KPI."""
    fig, ax = plt.subplots(figsize=(8, 5))

    labels = sorted(groups.keys())
    means = [np.mean([r[kpi] for r in groups[k]]) for k in labels]
    stds = [np.std([r[kpi] for r in groups[k]]) for k in labels]

    x = np.arange(len(labels))
    bars = ax.bar(x, means, yerr=stds, capsize=4, width=0.5,
                  color=[get_color(c) for c, p in labels],
                  hatch=[get_hatch(p) for c, p in labels],
                  edgecolor="black", linewidth=0.8)

    ax.set_xticks(x)
    ax.set_xticklabels([make_label(c, p) for c, p in labels], rotation=30, ha="right")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, f"{kpi}_bar.png"), dpi=150)
    plt.close(fig)


def smoothness_chart(groups, output_dir):
    """Side-by-side subplots for linear and angular accel RMS."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    labels = sorted(groups.keys())
    x = np.arange(len(labels))
    xlabels = [make_label(c, p) for c, p in labels]

    for ax, kpi, title in [
        (ax1, "linear_accel_rms", "Linear Accel RMS (m/s\u00b2)"),
        (ax2, "angular_accel_rms", "Angular Accel RMS (rad/s\u00b2)"),
    ]:
        means = [np.mean([r[kpi] for r in groups[k]]) for k in labels]
        stds = [np.std([r[kpi] for r in groups[k]]) for k in labels]
        ax.bar(x, means, yerr=stds, capsize=4, width=0.5,
               color=[get_color(c) for c, p in labels],
               hatch=[get_hatch(p) for c, p in labels],
               edgecolor="black", linewidth=0.8)
        ax.set_xticks(x)
        ax.set_xticklabels(xlabels, rotation=30, ha="right")
        ax.set_ylabel(title)
        ax.set_title(title)
        ax.grid(axis="y", alpha=0.3)

    fig.suptitle("Smoothness Comparison", fontsize=14)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "smoothness_bar.png"), dpi=150)
    plt.close(fig)


def box_plots(groups, output_dir):
    """Box plots for each KPI, one figure per KPI."""
    labels = sorted(groups.keys())
    xlabels = [make_label(c, p) for c, p in labels]
    colors = [get_color(c) for c, p in labels]

    for kpi, ylabel in KPI_LABELS.items():
        fig, ax = plt.subplots(figsize=(8, 5))
        data = [[r[kpi] for r in groups[k]] for k in labels]

        bp = ax.boxplot(data, patch_artist=True, labels=xlabels)
        for patch, color in zip(bp["boxes"], colors):
            patch.set_facecolor(color)
            patch.set_alpha(0.7)

        ax.set_ylabel(ylabel)
        ax.set_title(f"{ylabel} Distribution")
        ax.grid(axis="y", alpha=0.3)
        plt.xticks(rotation=30, ha="right")
        fig.tight_layout()
        fig.savefig(os.path.join(output_dir, f"{kpi}_box.png"), dpi=150)
        plt.close(fig)


def radar_chart(groups, output_dir):
    """Radar/spider chart with normalized KPIs (lower-is-better inverted)."""
    kpis = ["cte_rmse", "cte_max", "mission_time",
            "linear_accel_rms", "angular_accel_rms", "path_efficiency"]
    kpi_display = ["CTE\nRMSE", "CTE\nMax", "Mission\nTime",
                   "Lin Accel\nRMS", "Ang Accel\nRMS", "Path\nEfficiency"]

    labels = sorted(groups.keys())

    # Compute means per group
    raw = {}
    for k in labels:
        raw[k] = [np.mean([r[kpi] for r in groups[k]]) for kpi in kpis]

    # Normalize each KPI to [0, 1] across all groups
    # For lower-is-better KPIs (all except path_efficiency), invert so higher = better
    all_vals = np.array([raw[k] for k in labels])  # shape (n_groups, n_kpis)
    mins = all_vals.min(axis=0)
    maxs = all_vals.max(axis=0)
    ranges = maxs - mins
    ranges[ranges < 1e-9] = 1.0  # avoid division by zero

    normalized = {}
    for k in labels:
        normed = []
        for i, kpi in enumerate(kpis):
            val = (raw[k][i] - mins[i]) / ranges[i]
            # Invert lower-is-better metrics so that outer ring = better
            if kpi != "path_efficiency":
                val = 1.0 - val
            normed.append(val)
        normalized[k] = normed

    # Plot
    n_kpis = len(kpis)
    angles = np.linspace(0, 2 * np.pi, n_kpis, endpoint=False).tolist()
    angles += angles[:1]  # close the polygon

    fig, ax = plt.subplots(figsize=(8, 8), subplot_kw=dict(polar=True))

    for k in labels:
        controller, planner = k
        vals = normalized[k] + normalized[k][:1]
        style = "--" if planner == "navfn" else "-"
        ax.plot(angles, vals, style, color=get_color(controller), linewidth=2,
                label=make_label(controller, planner))
        ax.fill(angles, vals, alpha=0.1, color=get_color(controller))

    ax.set_xticks(angles[:-1])
    ax.set_xticklabels(kpi_display)
    ax.set_ylim(0, 1.05)
    ax.set_title("Normalized KPI Comparison\n(outer = better)", pad=20)
    ax.legend(loc="upper right", bbox_to_anchor=(1.3, 1.1))
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "radar_chart.png"), dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Plot navigation benchmark KPIs")
    parser.add_argument("csv_file", help="Path to kpi_results.csv")
    parser.add_argument("--output-dir", default="plots", help="Directory for output PNGs")
    args = parser.parse_args()

    if not os.path.exists(args.csv_file):
        print(f"Error: CSV file '{args.csv_file}' not found")
        sys.exit(1)

    os.makedirs(args.output_dir, exist_ok=True)

    rows = load_csv(args.csv_file)
    if not rows:
        print("Error: CSV file is empty")
        sys.exit(1)

    groups = group_by_controller_planner(rows)
    print(f"Loaded {len(rows)} rows, {len(groups)} controller/planner combos")

    # 1. CTE RMSE bar chart
    bar_chart(groups, "cte_rmse", "Cross-Track Error (RMSE)", "CTE RMSE (m)", args.output_dir)

    # 2. CTE Max bar chart
    bar_chart(groups, "cte_max", "Cross-Track Error (Max)", "CTE Max (m)", args.output_dir)

    # 3. Mission time bar chart
    bar_chart(groups, "mission_time", "Mission Time", "Time (s)", args.output_dir)

    # 4. Smoothness side-by-side
    smoothness_chart(groups, args.output_dir)

    # 5. Path efficiency bar chart
    bar_chart(groups, "path_efficiency", "Path Efficiency", "Ratio (actual / planned)", args.output_dir)

    # 6. Box plots for each KPI
    box_plots(groups, args.output_dir)

    # 7. Radar chart
    radar_chart(groups, args.output_dir)

    print(f"Charts saved to {args.output_dir}/")


if __name__ == "__main__":
    main()

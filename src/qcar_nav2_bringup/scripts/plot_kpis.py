import os
import csv
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict

# Configurations
CONTROLLERS = ["rpp", "stanley", "dwb", "vpc"]
PLANNERS = ["smac", "navfn"]

PLANNER_COLORS = {
    "smac": "#1f77b4",  # Blue
    "navfn": "#ff7f0e"  # Orange
}

KPI_CONFIG = {
    "cte_rmse": "CTE RMSE (m)",
    "cte_max": "CTE Max (m)",
    "mission_time": "Mission Time (s)",
    "linear_accel_rms": "Linear Accel RMS (m/s\u00b2)",
    "angular_accel_rms": "Angular Accel RMS (rad/s\u00b2)",
    "path_efficiency": "Path Efficiency"
}

def load_data(filepath):
    data = defaultdict(lambda: defaultdict(list))
    with open(filepath, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            c = row["controller"]
            p = row["planner"]
            for kpi in KPI_CONFIG.keys():
                data[kpi][(c, p)].append(float(row[kpi]))
    return data

def main():
    csv_file = "kpi_results.csv"
    if not os.path.exists(csv_file):
        print(f"Error: {csv_file} not found.")
        return

    data = load_data(csv_file)
    output_dir = "planner_plots"
    os.makedirs(output_dir, exist_ok=True)

    # 1. Generate Grouped Bar Charts
    for kpi, ylabel in KPI_CONFIG.items():
        fig, ax = plt.subplots(figsize=(8, 5))
        x = np.arange(len(CONTROLLERS))
        width = 0.35
        
        # Calculate means and stds for Smac
        smac_means = [np.mean(data[kpi][(c, "smac")]) for c in CONTROLLERS]
        smac_stds = [np.std(data[kpi][(c, "smac")]) for c in CONTROLLERS]
        
        # Calculate means and stds for NavFn
        navfn_means = [np.mean(data[kpi][(c, "navfn")]) for c in CONTROLLERS]
        navfn_stds = [np.std(data[kpi][(c, "navfn")]) for c in CONTROLLERS]
        
        # Plotting
        ax.bar(x - width/2, smac_means, width, yerr=smac_stds, 
               label="Smac Planner", color=PLANNER_COLORS["smac"], edgecolor="black", capsize=4)
        ax.bar(x + width/2, navfn_means, width, yerr=navfn_stds, 
               label="NavFn Planner", color=PLANNER_COLORS["navfn"], edgecolor="black", capsize=4)
        
        ax.set_ylabel(ylabel, fontweight='bold')
        ax.set_title(f"Planner Comparison: {ylabel}", pad=15)
        ax.set_xticks(x)
        ax.set_xticklabels([c.upper() for c in CONTROLLERS])
        ax.legend()
        ax.grid(axis="y", linestyle="--", alpha=0.6)
        
        fig.tight_layout()
        fig.savefig(f"{output_dir}/{kpi}_grouped_bar.png", dpi=150)
        plt.close(fig)

    # 2. Generate Summary Table (CSV)
    summary_file = f"{output_dir}/kpi_summary_table.csv"
    with open(summary_file, "w", newline="") as f:
        writer = csv.writer(f)
        header = ["Controller", "Planner"] + [KPI_CONFIG[k] for k in KPI_CONFIG.keys()]
        writer.writerow(header)
        
        for c in CONTROLLERS:
            for p in PLANNERS:
                row = [c.upper(), p.upper()]
                for kpi in KPI_CONFIG.keys():
                    mean_val = np.mean(data[kpi][(c, p)])
                    std_val = np.std(data[kpi][(c, p)])
                    row.append(f"{mean_val:.2f} ± {std_val:.2f}")
                writer.writerow(row)

    print(f"Generated {len(KPI_CONFIG)} grouped bar charts and 1 summary table in {output_dir}/")

if __name__ == "__main__":
    main()
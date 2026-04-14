#!/usr/bin/env bash
#
# Automated benchmark runner for QCar Nav2 controllers.
#
# Usage:
#   ./run_benchmark.sh [--runs N] [--csv FILE] [--bags-dir DIR]
#
# Prerequisites:
#   - Gazebo simulation running: ros2 launch qcar_gazebo simulation.yaml
#   - Nav2 stack running:        ros2 launch qcar_nav2_bringup navigation.yaml
#   - Edit CONTROLLERS and PLANNERS arrays below for your test matrix
#
# What it does per trial:
#   1. Starts rosbag recording (/plan, /odom, /cmd_vel)
#   2. Sends a nav goal via ros2 action
#   3. Waits for navigation to complete (or timeout)
#   4. Stops recording
#   5. Runs compute_kpis.py on the bag
#

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---------- Configuration ----------
RUNS=3
CSV_FILE="kpi_results.csv"
BAGS_DIR="rosbags"

# Controllers to test (must match nav2_params.yaml plugin names)
CONTROLLERS=("rpp" "stanley" "vpc" "dwb")

# Planners to test
PLANNERS=("smac")

# Navigation goal (x, y, yaw) in map frame
GOAL_X=2.0
GOAL_Y=0.0
GOAL_YAW=0.0

# Timeout in seconds for each navigation trial
NAV_TIMEOUT=120

# Topics to record
RECORD_TOPICS="/plan /odom /cmd_vel"

# ---------- Argument parsing ----------
while [[ $# -gt 0 ]]; do
    case $1 in
        --runs)    RUNS="$2"; shift 2 ;;
        --csv)     CSV_FILE="$2"; shift 2 ;;
        --bags-dir) BAGS_DIR="$2"; shift 2 ;;
        -h|--help)
            echo "Usage: $0 [--runs N] [--csv FILE] [--bags-dir DIR]"
            exit 0
            ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

mkdir -p "$BAGS_DIR"

echo "============================================"
echo " QCar Nav2 Benchmark"
echo "  Runs per combo:  $RUNS"
echo "  Controllers:     ${CONTROLLERS[*]}"
echo "  Planners:        ${PLANNERS[*]}"
echo "  Goal:            ($GOAL_X, $GOAL_Y, yaw=$GOAL_YAW)"
echo "  Bags directory:  $BAGS_DIR"
echo "  CSV output:      $CSV_FILE"
echo "============================================"
echo ""

send_nav_goal() {
    # Send NavigateToPose action goal
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
        "{pose: {header: {frame_id: 'map'}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: $(python3 -c "import math; print(math.sin($GOAL_YAW/2))"), w: $(python3 -c "import math; print(math.cos($GOAL_YAW/2))")}}}}" \
        --feedback 2>&1 &
    NAV_PID=$!
}

wait_for_nav() {
    local timeout=$1
    local elapsed=0
    while kill -0 "$NAV_PID" 2>/dev/null; do
        sleep 1
        elapsed=$((elapsed + 1))
        if [ "$elapsed" -ge "$timeout" ]; then
            echo "  [WARN] Navigation timed out after ${timeout}s"
            kill "$NAV_PID" 2>/dev/null || true
            wait "$NAV_PID" 2>/dev/null || true
            return 1
        fi
    done
    wait "$NAV_PID" 2>/dev/null || true
    return 0
}

reset_robot_pose() {
    # Teleport robot back to start via Gazebo service
    gz service -s /world/qcar_strack/set_pose \
        --reqtype gz.msgs.Pose \
        --reptype gz.msgs.Boolean \
        --timeout 5000 \
        --req "name: 'qcar', position: {x: -1.0, y: 0.0, z: 0.05}, orientation: {w: 1.0}" \
        2>/dev/null || echo "  [WARN] Could not reset robot pose"
    sleep 2
}

# ---------- Main loop ----------
for controller in "${CONTROLLERS[@]}"; do
    for planner in "${PLANNERS[@]}"; do
        for run in $(seq 1 "$RUNS"); do
            combo="${controller}_${planner}_run${run}"
            bag_path="${BAGS_DIR}/${combo}"
            echo "--- $combo ---"

            # Reset robot to start
            echo "  Resetting robot pose..."
            reset_robot_pose

            # Start recording
            echo "  Recording bag: $bag_path"
            ros2 bag record -o "$bag_path" $RECORD_TOPICS &
            BAG_PID=$!
            sleep 1  # let recorder initialize

            # Send goal
            echo "  Sending nav goal..."
            send_nav_goal

            # Wait for completion
            if wait_for_nav "$NAV_TIMEOUT"; then
                echo "  Navigation completed"
            fi

            # Stop recording
            sleep 1
            kill "$BAG_PID" 2>/dev/null || true
            wait "$BAG_PID" 2>/dev/null || true
            echo "  Bag saved"

            # Compute KPIs
            echo "  Computing KPIs..."
            python3 "$SCRIPT_DIR/compute_kpis.py" "$bag_path" \
                --controller "$controller" \
                --planner "$planner" \
                --run "$run" \
                --csv "$CSV_FILE"

            echo ""
        done
    done
done

echo "============================================"
echo " Benchmark complete!"
echo " Results: $CSV_FILE"
echo "============================================"

# Generate plots if matplotlib is available
if python3 -c "import matplotlib" 2>/dev/null; then
    echo "Generating plots..."
    python3 "$SCRIPT_DIR/plot_kpis.py" "$CSV_FILE" --output-dir plots/
else
    echo "Install matplotlib to generate plots: pip3 install matplotlib"
fi

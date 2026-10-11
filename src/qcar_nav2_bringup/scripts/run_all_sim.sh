#!/bin/bash
# Run the full sim benchmark: every controller with every planner.
#
#   bash run_all_sim.sh [runs_per_combo] [out_root]      (defaults: 5, benchmarks_sim)
#   COMBOS="vector:hybrid" bash run_all_sim.sh             (only some combos)
#
# Starts the Gazebo sim once, then for each combo brings up
# navigation.launch.py, runs run_benchmark_sim.py and tears the stack down.
# Order: rpp, stanley, vector with hybrid, then the same with navfn.
# Logs go to <out_root>/logs/.

RUNS="${1:-5}"
WS="$(cd "$(dirname "$0")/../../.." && pwd)"
cd "$WS"
OUT="${2:-benchmarks_sim}"
LOGS="$OUT/logs"
mkdir -p "$LOGS"

source /opt/ros/jazzy/setup.bash
source install/setup.bash

COMBOS="${COMBOS:-rpp:hybrid stanley:hybrid vector:hybrid rpp:navfn stanley:navfn vector:navfn}"
NAV_PROCS='controller_server|hybrid_planner_server|planner_server|bt_navigator|lifecycle_manager|direct_map_publisher'

stop_group() {
    local pid=$1
    kill -INT -- -"$pid" 2>/dev/null
    for _ in $(seq 1 20); do kill -0 "$pid" 2>/dev/null || return; sleep 0.5; done
    kill -TERM -- -"$pid" 2>/dev/null
    sleep 2
    kill -KILL -- -"$pid" 2>/dev/null
}

wait_nav_gone() {
    for _ in $(seq 1 20); do
        pgrep -f "$NAV_PROCS" > /dev/null || return
        sleep 0.5
    done
    pkill -KILL -f "$NAV_PROCS"
}

echo "* Starting simulation"
setsid ros2 launch qcar_gazebo simulation.yaml rviz:=False > "$LOGS/sim.log" 2>&1 &
SIM_PID=$!
trap 'echo "Interrupted"; stop_group $SIM_PID; exit 130' INT TERM

for _ in $(seq 1 60); do
    timeout 3 ros2 topic echo --once /odom nav_msgs/msg/Odometry > /dev/null 2>&1 && break
    sleep 1
done
echo "  sim up"

for combo in $COMBOS; do
    ctrl=${combo%%:*}
    plan=${combo##*:}
    log="$LOGS/nav_${ctrl}_${plan}.log"
    echo "* $ctrl + $plan"
    # bt_navigator's lifecycle reply is occasionally lost in DDS and the
    # lifecycle manager then waits forever; relaunch the stack if so.
    up=false
    for attempt in 1 2 3; do
        setsid ros2 launch qcar_nav2_bringup navigation.launch.py \
            controller_type:=$ctrl planner_type:=$plan > "$log.$attempt" 2>&1 &
        NAV_PID=$!
        for _ in $(seq 1 60); do
            # A costmap can also fail to activate (TF timeout) while the
            # lifecycle manager still reports success: then the planner sees
            # no map and rejects every goal. Treat that as a failed bring-up.
            if grep -qE 'Failed to bring up|process has died|Failed to activate' "$log.$attempt"; then break; fi
            if grep -q 'Managed nodes are active' "$log.$attempt" && \
               grep -q 'Controller Server Ready' "$log.$attempt" && \
               { [ "$plan" != hybrid ] || grep -q 'Hybrid Planner Server Fully Ready' "$log.$attempt"; }; then
                sleep 2
                grep -q 'Failed to activate' "$log.$attempt" || up=true
                break
            fi
            sleep 1
        done
        [ "$up" = true ] && break
        echo "  nav stack did not come up (attempt $attempt) -- see $log.$attempt"
        stop_group $NAV_PID
        wait_nav_gone
    done
    if [ "$up" = true ]; then
        sleep 3  # let the planner's costmap pick up /map
        python3 src/qcar_nav2_bringup/scripts/run_benchmark_sim.py \
            --controller "$ctrl" --planner "$plan" --runs "$RUNS" --out-root "$OUT"
    else
        echo "  giving up on $ctrl + $plan"
    fi

    stop_group $NAV_PID
    wait_nav_gone
done

echo "* Stopping simulation"
stop_group $SIM_PID
echo "Done. Results: $OUT/kpi_results_sim.csv, $OUT/runs_sim.csv"

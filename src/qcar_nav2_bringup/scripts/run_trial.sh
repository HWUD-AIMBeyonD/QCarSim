#!/bin/bash
# One repeatable navigation trial on the hardware-map world.
#
#   bash run_trial.sh <controller> <planner> <run> [goal_x goal_y goal_yaw_deg]
#
# Needs simulation.yaml and navigation.launch.py (with the same controller and
# planner) already running -- <controller>/<planner> here are only labels.
# Resets the car to the spawn pose, records a bag to
# rosbags/hw_parity/<controller>_<planner>_run<run>, sends one NavigateToPose
# goal and waits for the result.

CONTROLLER="$1"
PLANNER="$2"
RUN="$3"
GX="${4:-3.0}"
GY="${5:-0.1}"
GYAW_DEG="${6:-0}"

if [ -z "$CONTROLLER" ] || [ -z "$PLANNER" ] || [ -z "$RUN" ]; then
    echo "usage: $0 <rpp|stanley|vector> <hybrid|navfn> <run> [goal_x goal_y goal_yaw_deg]"
    exit 2
fi

WS="$(cd "$(dirname "$0")/../../.." && pwd)"
BAG="$WS/rosbags/hw_parity/${CONTROLLER}_${PLANNER}_run${RUN}"
QZ=$(python3 -c "import math; print(math.sin(math.radians($GYAW_DEG)/2))")
QW=$(python3 -c "import math; print(math.cos(math.radians($GYAW_DEG)/2))")

if [ -e "$BAG" ]; then
    echo "$BAG already exists; pick another run number"
    exit 1
fi
mkdir -p "$(dirname "$BAG")"

echo "* Resetting car to spawn pose"
gz service -s /world/qcar_hwmap/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
    --timeout 3000 --req 'name: "qcar", position: {x: 0, y: 0, z: 0}, orientation: {w: 1}' > /dev/null
sleep 2  # let the emulator's 1 s command timeout and the costmaps settle

echo "* Recording $BAG"
ros2 bag record -o "$BAG" /odom /plan /cmd_vel /cmd_vel_hw /scan > /dev/null 2>&1 &
BAG_PID=$!
sleep 2

echo "* Goal ($GX, $GY, ${GYAW_DEG} deg) with $CONTROLLER + $PLANNER"
START=$(date +%s.%N)
RESULT=$(timeout 300 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: $GX, y: $GY}, orientation: {z: $QZ, w: $QW}}}}" 2>&1 \
    | grep 'finished with status')
END=$(date +%s.%N)

sleep 1
kill -TERM $BAG_PID  # SIGINT is ignored by background jobs of a non-interactive shell
wait $BAG_PID 2>/dev/null

echo "  ${RESULT:-no result (timeout)}   wall time $(python3 -c "print(round($END-$START,1))") s"
timeout 5 ros2 topic echo --once /odom nav_msgs/msg/Odometry --field pose.pose.position | head -2 | tr '\n' ' '
echo

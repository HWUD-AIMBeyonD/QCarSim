# QCar Simulation — ROS 2 Jazzy + Gazebo Harmonic

A complete simulation environment for the Quanser QCar (1/10 scale Ackermann-steered vehicle) with autonomous navigation. Built for a Final Year Project comparing path-tracking controllers on a custom S-track course.

The simulation includes four navigation controllers (RPP, Stanley, Vector Pursuit, DWB), two global planners (SmacPlannerHybrid, NavFn), and a benchmarking pipeline for KPI-based controller evaluation. A pre-built map and tuned navigation parameters are included — you can have the QCar driving autonomously within minutes of cloning.

## Prerequisites

- **OS:** Ubuntu 24.04 (Noble)
- **ROS 2:** Jazzy Jalisco (`ros-jazzy-desktop`)
- **Gazebo:** Harmonic (`ros-jazzy-ros-gz`)

### Install Dependencies

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-smac-planner ros-jazzy-slam-toolbox \
  ros-jazzy-cartographer-ros ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-laser-filters ros-jazzy-topic-tools

pip install matplotlib numpy --break-system-packages
```

## Build

```bash
cd ~/Documents/qcarSim
source /opt/ros/jazzy/setup.bash
rosdep install -i --from-path src --rosdistro jazzy -y
colcon build --symlink-install
source install/setup.bash
```

## Quick Start — Autonomous Navigation

### Terminal 1 — Start simulation

```bash
ros2 launch qcar_gazebo simulation.yaml
```

Wait until Gazebo is fully loaded and the QCar is visible on the S-track.

### Terminal 2 — Start navigation

```bash
ros2 launch qcar_nav2_bringup navigation.yaml
```

Wait until the lifecycle manager reports all nodes active. RViz will open with the map loaded and the QCar localized at the start of the track.

### Send a navigation goal

In RViz, click **2D Goal Pose** and click on the map where you want the QCar to drive to. The QCar will plan a path and follow it autonomously.

Alternatively, you can send a goal from the terminal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 5.2, z: 0.0},
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

The Gazebo Teleop panel (WASD keys) can be used for manual driving at any time.

## Switching Controllers

Edit `src/qcar_nav2_bringup/config/nav2_params.yaml`. Under `controller_server`, uncomment the controller you want under `FollowPath:` and comment the others. RPP is active by default.

| Controller | Plugin |
|---|---|
| Regulated Pure Pursuit (default) | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` |
| Stanley | `stanley_controller::StanleyController` |
| Vector Pursuit | `vector_pursuit_controller::VectorPursuitController` |
| DWB | `dwb_core::DWBLocalPlanner` |

After switching, rebuild and relaunch:

```bash
colcon build --symlink-install --packages-select qcar_nav2_bringup
source install/setup.bash
```

## Switching Global Planners

In the same `nav2_params.yaml`, change the plugin under `planner_server > GridBased:`. SmacPlannerHybrid is active by default.

| Planner | Plugin |
|---|---|
| SmacPlannerHybrid (default) | `nav2_smac_planner::SmacPlannerHybrid` |
| NavFn | `nav2_navfn_planner::NavfnPlanner` |

## KPI Benchmarking

### Record a navigation run

```bash
# Start recording in the background
ros2 bag record /odom /cmd_vel /plan /received_global_plan -o rosbags/rpp_smac_run1 &

# Send a goal via RViz or terminal, wait for completion, then stop recording:
kill %1
```

### Compute KPIs from a recorded run

```bash
python3 src/qcar_nav2_bringup/scripts/compute_kpis.py \
  rosbags/rpp_smac_run1/rpp_smac_run1.mcap \
  --controller rpp --planner smac --run 1
```

Results are appended to `kpi_results.csv`. KPIs computed: CTE RMSE, CTE max, mission time, linear/angular acceleration RMS, and path efficiency.

### Generate comparison charts

After recording multiple runs across different controllers and planners:

```bash
python3 src/qcar_nav2_bringup/scripts/plot_kpis.py kpi_results.csv
```

## Creating a New World

If you want to test on a different track layout:

1. Create a new SDF world file in `src/qcar_gazebo/worlds/`. Use `qcar_strack.sdf` as a template — it contains the required Gazebo plugins (Physics, SceneBroadcaster, Sensors, Contact) and the GUI configuration with Teleop.

2. Update `src/qcar_gazebo/launch/simulation.yaml` to point to your new world:
```yaml
  - node:
      pkg: ros_gz_sim
      exec: gzserver
      name: gzserver
      param:
        - name: world_sdf_file
          value: $(find-pkg-share qcar_gazebo)/worlds/your_new_world.sdf
```

3. Build a map of the new world using SLAM (see below), then either update the default map path in `navigation.yaml` or pass it at launch time:
```bash
ros2 launch qcar_nav2_bringup navigation.yaml map:=/path/to/your_map.yaml
```

## SLAM Mapping

The included S-track map was generated from the known world geometry. If you need to build a map for a new world, two SLAM backends are available:

### Option A: slam_toolbox

```bash
# Terminal 1 — Simulation
ros2 launch qcar_gazebo simulation.yaml

# Terminal 2 — SLAM
ros2 launch qcar_nav2_bringup slam.yaml

# Terminal 3 — Drive around using Gazebo Teleop (WASD) or:
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 4 — Save map when done
bash src/qcar_nav2_bringup/scripts/save_map.sh my_map
```

### Option B: Google Cartographer

```bash
# Terminal 1 — Simulation
ros2 launch qcar_gazebo simulation.yaml

# Terminal 2 — Cartographer
ros2 launch qcar_nav2_bringup cartographer.yaml

# Terminal 3 — Drive around

# Terminal 4 — Save map when done
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
bash src/qcar_nav2_bringup/scripts/save_map.sh my_cartographer_map
```

## Package Structure

| Package | Description |
|---|---|
| `qcar_description` | URDF/Xacro model, meshes, sensor definitions, RViz config |
| `qcar_gazebo` | S-track world, Gazebo launch, bridge configs, laser filter |
| `qcar_nav2_bringup` | Nav2 parameters, maps, SLAM configs, KPI scripts, launch files |
| `stanley_controller` | Stanley path-tracking controller (Nav2 plugin) |
| `vector_pursuit_controller` | Vector Pursuit controller (Nav2 plugin) |

## Architecture

- **Drive:** `gz::sim::systems::AckermannSteering` — subscribes to `/cmd_vel`, actuates steering and drive joints
- **Odometry:** `gz::sim::systems::OdometryPublisher` — publishes ground truth pose from the physics engine on `/odom` and the `odom → base_link` transform
- **Localization:** Static `map → odom` identity transform (ground truth odometry makes AMCL unnecessary)
- **Map:** Pre-built occupancy grid generated from the S-track geometry
- **Navigation:** Nav2 with configurable global planner and local controller

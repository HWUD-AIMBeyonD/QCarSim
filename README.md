# QCar Gazebo Simulation — ROS 2 Jazzy

A Gazebo Harmonic simulation of the Quanser QCar (1/10 scale Ackermann-steered vehicle) running a full autonomous navigation stack on ROS 2 Jazzy. Built by adapting the [Husarion ROSbot simulation infrastructure](https://github.com/husarion/rosbot_ros) and replacing the robot model, drive system, and navigation configuration for Ackermann kinematics.

This workspace reproduces the hardware QCar navigation pipeline (originally ROS 2 Dashing on Jetson TX2) in simulation, enabling repeatable experiments and KPI-based evaluation of navigation controllers without physical hardware access.

---

## Prerequisites

- **OS:** Ubuntu 24.04 (Noble)
- **ROS 2:** Jazzy Jalisco (desktop install)
- **Gazebo:** Harmonic (gz-sim 8, installed via `ros-jazzy-ros-gz`)
- **Nav2 packages:**
  ```bash
  sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox ros-jazzy-nav2-smac-planner \
    ros-jazzy-nav2-lifecycle-manager ros-jazzy-teleop-twist-keyboard
  ```

---

## Build

```bash
cd ~/Documents/qcarSim
source /opt/ros/jazzy/setup.bash

# Pull Husarion external dependencies
vcs import src < src/rosbot_ros/rosbot/rosbot_simulation.repos

# Install ROS dependencies
export PIP_BREAK_SYSTEM_PACKAGES=1
rosdep update --rosdistro jazzy
rosdep install -i --from-path src --rosdistro jazzy -y

# Build
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## Usage

### Workflow A: SLAM Mapping

Generate an occupancy grid map of the simulation environment.

```bash
# Terminal 1 — Launch simulation
ros2 launch qcar_gazebo simulation.yaml

# Terminal 2 — Launch SLAM
ros2 launch qcar_nav2_bringup slam.yaml

# Terminal 3 — Drive around to build the map
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 4 — Save the map when done
ros2 launch qcar_nav2_bringup save_map.yaml map_name:=my_map
```

### Workflow B: Autonomous Navigation

Navigate autonomously on a saved map using AMCL localization and the Regulated Pure Pursuit controller.

```bash
# Terminal 1 — Launch simulation
ros2 launch qcar_gazebo simulation.yaml

# Terminal 2 — Launch Nav2 with a saved map
ros2 launch qcar_nav2_bringup navigation.yaml map:=/home/g16/Documents/qcarSim/my_map.yaml

# In RViz:
#   1. Click "2D Pose Estimate" to set initial pose
#   2. Click "2D Goal Pose" to send navigation goals
```

### Set Initial Pose via CLI

```bash
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    },
    covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.07]
  }
}" --once
```

### Send Navigation Goal via CLI

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}"
```

---

## Package Structure

```
src/
├── qcar_description/        Robot model (URDF, meshes, Gazebo plugins)
├── qcar_gazebo/             Simulation launch, Gazebo-ROS bridges
├── qcar_controller/         Drive controller interface, EKF config
├── qcar_nav2_bringup/       SLAM, Nav2 navigation, map saving
└── rosbot_ros/              Husarion infrastructure (worlds, utilities)
```

### qcar_description — Robot Model

Defines the QCar's physical structure, sensors, and Gazebo simulation plugins.

```
qcar_description/
├── meshes/                        Visual and collision meshes (.stl + .dae)
│   ├── QCarBody, QCarWheel, QCarLidar, QCarSteeringHubL, QCarSteeringHubR
├── models/realsense/              RealSense D435 camera mesh
├── urdf/
│   ├── qcar.urdf.xacro           Top-level: includes body, wheels, and Gazebo files
│   ├── qcar_body.urdf.xacro      Chassis, IMU, LiDAR, RealSense, 4x CSI cameras
│   ├── qcar_wheels.urdf.xacro    Ackermann steering hubs + wheel macros with friction
│   └── qcar_gz.urdf.xacro        Gazebo Harmonic plugins:
│                                    - AckermannSteering (drive + odometry)
│                                    - JointStatePublisher (6 joints)
│                                    - gpu_lidar (RPLiDAR A2: 720 samples, 10Hz, 12m)
│                                    - imu (50Hz with Gaussian noise)
│                                    - camera + depth_camera (RealSense D435)
│                                    - camera x4 (CSI front/back/left/right)
├── config/qcar/basic.yaml        Component configuration
├── rviz/qcar.rviz                RViz display configuration
└── launch/
    ├── qcar.yaml                  Loads URDF via xacro → robot_state_publisher
    └── rviz.yaml                  Launches RViz with QCar config
```

**Key parameters:** Wheelbase 0.256m, track width 0.112m, wheel radius 0.033m, steering limits ±0.5236 rad (±30°), body mass ~2.0 kg.

### qcar_gazebo — Simulation Environment

Launches Gazebo with the Husarion world, spawns the QCar, and bridges Gazebo topics to ROS.

```
qcar_gazebo/
├── config/
│   ├── gz_bridge.yaml             Clock bridge (Gazebo → ROS)
│   └── qcar_bridge.yaml          Per-robot topic bridges:
│                                    cmd_vel (ROS→GZ), odom (GZ→ROS),
│                                    scan, imu/data_raw, joint_states, tf
└── launch/
    ├── simulation.yaml            Top-level: Gazebo world + clock bridge + spawn
    └── spawn_qcar.yaml           Spawns QCar, starts GZ bridge, EKF,
                                   laser filter, RViz
```

### qcar_controller — Drive Control and Sensor Fusion

Provides the controller launch interface and EKF configuration. The actual drive control is handled by Gazebo's native `AckermannSteering` plugin (no ros2_control needed).

```
qcar_controller/
├── config/
│   ├── qcar/controllers.yaml          Minimal controller config
│   └── qcar_localization/ekf.yaml     EKF fusing /odom (x, y, yaw, vx, vyaw)
│                                       and /imu/data_raw (angular_vel_z, accel_x/y)
└── launch/
    └── controller.yaml                Loads QCar URDF with sim parameters
```

### qcar_nav2_bringup — Navigation Stack

SLAM mapping, autonomous navigation with AMCL + SmacPlannerHybrid + Regulated Pure Pursuit.

```
qcar_nav2_bringup/
├── config/
│   ├── nav2_params.yaml               Full Nav2 configuration:
│   │                                    AMCL (500-2000 particles, likelihood field)
│   │                                    SmacPlannerHybrid (Reeds-Shepp curves)
│   │                                    RegulatedPurePursuitController
│   │                                    Costmaps (0.05m resolution, robot_radius 0.12m)
│   │                                    Behaviors (backup, drive_on_heading, wait — no spin)
│   └── slam_online_async.yaml         slam_toolbox params (base_frame=base_link, 12m range)
├── launch/
│   ├── slam.yaml                      SLAM with lifecycle manager (autostart)
│   ├── navigation.yaml                Full Nav2 stack with map loading
│   └── save_map.yaml                  Map saving utility
└── scripts/
    └── save_map.sh                    Quick CLI for map saving
```

**Ackermann-specific Nav2 tuning:**
- `use_rotate_to_heading: false` — Ackermann cars cannot rotate in place
- No `spin` recovery behavior — replaced with backup, drive_on_heading, wait
- SmacPlannerHybrid with `REEDS_SHEPP` motion model and `minimum_turning_radius: 0.35`
- Progress checker: `required_movement_radius: 0.15`, `movement_time_allowance: 30.0`

---

## What Was Adopted from Husarion ROSbot Simulation

The [Husarion rosbot_ros](https://github.com/husarion/rosbot_ros) repository (Jazzy branch) provided the simulation infrastructure foundation. The following components are used directly or adapted:

| Component | Source Package | How It's Used |
|---|---|---|
| Gazebo world files | `husarion_gz_worlds` | The `husarion_world` SDF environment is used as the test world |
| Gazebo launch pattern | `rosbot_gazebo` | The `simulation.yaml` → `spawn_robot.yaml` pattern was replicated for QCar |
| YAML launch file conventions | All rosbot packages | The `set_env`, `let`, `include` YAML launch syntax was adopted throughout |
| Laser filter | `rosbot_utils` | `scan_to_scan_filter_chain` filters laser points inside the robot footprint |
| ros_gz_bridge pattern | `rosbot_gazebo/config` | The bridge YAML config format and namespace placeholder pattern were replicated |
| URDF/launch interface | `rosbot_description` | The `qcar.yaml` launch file mirrors `rosbot.yaml` exactly (xacro → robot_state_publisher) |
| Component description bridges | `husarion_components_description` | Gazebo component sensor bridges |

**What was replaced (not reused):**
- ROSbot URDF → QCar URDF with Ackermann kinematics
- `diff_drive_controller` (ros2_control) → `gz::sim::systems::AckermannSteering` (native Gazebo plugin)
- `rosbot_hardware_interfaces` → not needed in sim (Gazebo plugin handles drive directly)
- `rosbot_localization` EKF config → custom `ekf.yaml` matching QCar odom/IMU topic names
- All Nav2 parameters → re-tuned for Ackermann constraints (no spin, Reeds-Shepp planner, RPP controller)

---

## Debugging

### Check Topics

```bash
# Core topics that should be active during navigation
ros2 topic hz /scan              # Should be ~10 Hz
ros2 topic hz /odom              # Should be ~50 Hz
ros2 topic hz /imu/data_raw      # Should be ~50 Hz
ros2 topic hz /odometry/filtered # Should be ~50 Hz (from EKF)
ros2 topic hz /cmd_vel           # Active during goal execution

# Check if map is published (uses Transient Local QoS)
ros2 topic echo /map --qos-durability transient_local --once | head -5
```

### Check TF Tree

```bash
# Generate TF tree PDF
ros2 run tf2_tools view_frames

# Verify specific transforms
ros2 run tf2_ros tf2_echo map odom        # From AMCL
ros2 run tf2_ros tf2_echo odom base_link  # From EKF/Gazebo
```

**Expected TF chain:** `map → odom → base_link → body_link → (sensors, wheels)`

### Check Node Health

```bash
ros2 node list
ros2 node info /slam_toolbox
ros2 node info /controller_server

# Check lifecycle states
ros2 lifecycle get /slam_toolbox
ros2 lifecycle get /controller_server
```

### Common Issues

| Symptom | Cause | Fix |
|---|---|---|
| No meshes in Gazebo | `GZ_SIM_RESOURCE_PATH` missing qcar_description | Verify `set_env` in `simulation.yaml` uses `$(env GZ_SIM_RESOURCE_PATH '')` to preserve existing paths |
| `/map` not visible in RViz | QoS mismatch | Set Map display Durability to `Transient Local` in RViz |
| "Waiting for transform map→base_link" | AMCL not initialized | Set initial pose via RViz "2D Pose Estimate" or `/initialpose` topic |
| `cmd_vel` stays at zero | `use_rotate_to_heading: true` | Set `use_rotate_to_heading: false` in nav2_params.yaml RPP section |
| "Failed to make progress" | Progress checker too strict | Lower `required_movement_radius` (0.15), increase `movement_time_allowance` (30s) |
| Sharp turns in planned path | Using NavfnPlanner | Switch to `SmacPlannerHybrid` with `REEDS_SHEPP` motion model |
| SLAM node not subscribing to /scan | Lifecycle not activated | Ensure `nav2_lifecycle_manager` with `autostart: true` is in the SLAM launch |
| Husarion world fails to load | Resource path overwritten | Use `$(env GZ_SIM_RESOURCE_PATH '')` not `$GZ_SIM_RESOURCE_PATH` in set_env |
| World file not found (.sdf.sdf) | Double extension | Pass world name without `.sdf` extension to gz_sim launch |

### Monitor cmd_vel During Navigation

```bash
# Watch velocity commands in real time
ros2 topic echo /cmd_vel

# Check if velocity smoother is active
ros2 topic echo /cmd_vel_smoothed
```

---

## TF Tree

```
map
 └── odom                          (published by AMCL during navigation,
      └── base_link                 or slam_toolbox during mapping)
           └── body_link
                ├── imu_link
                ├── rplidar_link
                │    └── laser
                ├── realsense_link
                │    └── realsense_optical_frame
                ├── csi_front_link → csi_front_optical_frame
                ├── csi_back_link  → csi_back_optical_frame
                ├── csi_left_link  → csi_left_optical_frame
                ├── csi_right_link → csi_right_optical_frame
                ├── fl_steering_hub_link
                │    └── fl_wheel_link
                ├── fr_steering_hub_link
                │    └── fr_wheel_link
                ├── rl_wheel_link
                └── rr_wheel_link
```

---

## ROS Topics Reference

| Topic | Type | Source | Direction |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 controller → Gazebo | ROS → GZ |
| `/odom` | `nav_msgs/Odometry` | Gazebo Ackermann plugin | GZ → ROS |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF (robot_localization) | ROS internal |
| `/scan` | `sensor_msgs/LaserScan` | Gazebo RPLiDAR plugin | GZ → ROS |
| `/scan_filtered` | `sensor_msgs/LaserScan` | Laser filter node | ROS internal |
| `/imu/data_raw` | `sensor_msgs/Imu` | Gazebo IMU plugin | GZ → ROS |
| `/map` | `nav_msgs/OccupancyGrid` | map_server (Transient Local QoS) | ROS internal |
| `/joint_states` | `sensor_msgs/JointState` | Gazebo JointStatePublisher | GZ → ROS |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | AMCL | ROS internal |
| `/plan` | `nav_msgs/Path` | SmacPlannerHybrid | ROS internal |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | RViz / CLI | ROS internal |

---

## Software Versions

| Component | Version |
|---|---|
| Ubuntu | 24.04 Noble |
| ROS 2 | Jazzy Jalisco |
| Gazebo | Harmonic (gz-sim 8) |
| Nav2 | Jazzy (SmacPlannerHybrid + RPP) |
| slam_toolbox | Jazzy release |
| robot_localization | Jazzy release |

---

## License

QCar-specific packages (`qcar_description`, `qcar_gazebo`, `qcar_controller`, `qcar_nav2_bringup`) are licensed under Apache License 2.0. Husarion ROSbot packages retain their original Apache 2.0 license. Quanser QCar mesh files are used for research/education purposes.
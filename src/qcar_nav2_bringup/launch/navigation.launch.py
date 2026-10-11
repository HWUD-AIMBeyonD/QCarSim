#!/usr/bin/env python3
"""Sim counterpart of the hardware nav bringup (localization_optitrack +
nav2_online), running the same planner/controller code as the real QCar.

    direct_map_publisher   -> /map                    (verbatim hardware node)
    planner                -> planner_type:=hybrid : hybrid_planner_server (/ComputePathToPose)
                              planner_type:=navfn  : Jazzy planner_server + NavfnPlanner
                                                     (/compute_path_to_pose)
    controller_server      -> /FollowPath -> /cmd_vel  (rpp | stanley | vector)
    bt_navigator           -> /navigate_to_pose        (plan once, then follow)

map -> odom and base_link -> base come from spawn_qcar.yaml; /cmd_vel goes
through qcar_hw_emulator before it reaches Gazebo.

Usage:
    ros2 launch qcar_nav2_bringup navigation.launch.py controller_type:=stanley planner_type:=navfn
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SERVERS = {
    'rpp': ('rpp_controller', 'rpp_controller_server'),
    'stanley': ('stanley_controller', 'stanley_controller_server'),
    'vector': ('vector_pursuit_controller', 'vector_pursuit_controller_server'),
}

PLANNERS = ('hybrid', 'navfn')


def launch_setup(context):
    pkg_share = get_package_share_directory('qcar_nav2_bringup')
    controller_type = LaunchConfiguration('controller_type').perform(context)
    if controller_type not in SERVERS:
        raise RuntimeError('controller_type must be one of {}, got {!r}'.format(
            sorted(SERVERS), controller_type))
    planner_type = LaunchConfiguration('planner_type').perform(context)
    if planner_type not in PLANNERS:
        raise RuntimeError('planner_type must be one of {}, got {!r}'.format(
            PLANNERS, planner_type))

    params = os.path.join(pkg_share, 'config', 'hw', 'nav2_params_{}_{}.yaml'.format(
        controller_type, planner_type))
    bt_xml = os.path.join(pkg_share, 'config', 'hw', 'bt_navigation_{}.xml'.format(planner_type))
    map_yaml = LaunchConfiguration('map')
    sim_time = {'use_sim_time': True}
    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    ctrl_pkg, ctrl_exec = SERVERS[controller_type]

    if planner_type == 'hybrid':
        # Hardware code; not lifecycle-managed, as on the car.
        planner = Node(
            package='custom_hybrid_planner', executable='hybrid_planner_server',
            name='hybrid_planner_server', output='screen',
            parameters=[params, sim_time], remappings=tf_remaps)
        managed = ['bt_navigator']
    else:
        # Stock Jazzy NavFn (lifecycle node).
        planner = Node(
            package='nav2_planner', executable='planner_server',
            name='planner_server', output='screen',
            parameters=[params, sim_time], remappings=tf_remaps)
        managed = ['planner_server', 'bt_navigator']

    return [
        Node(
            package='qcar_nav2_bringup', executable='direct_map_publisher',
            name='map_server', output='screen',
            parameters=[{'yaml_filename': map_yaml, 'frame_id': 'map'}, sim_time]),

        planner,

        Node(
            package=ctrl_pkg, executable=ctrl_exec,
            name='controller_server', output='screen',
            parameters=[params, sim_time], remappings=tf_remaps),

        Node(
            package='nav2_bt_navigator', executable='bt_navigator',
            name='bt_navigator', output='screen',
            parameters=[params, sim_time, {'default_nav_to_pose_bt_xml': bt_xml}],
            remappings=tf_remaps),

        # As on hardware, the custom planner and controller servers are NOT
        # lifecycle-managed.
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[sim_time, {'autostart': True, 'node_names': managed}]),
    ]


def generate_launch_description():
    # Clean grid built from the hardware new_map (closed walls, unknown kept
    # unknown) by qcar_gazebo/scripts/generate_world_from_map.py --grid-out.
    # Pass map:=.../new_map.yaml to use the raw hardware map instead.
    default_map = os.path.join(
        get_package_share_directory('qcar_nav2_bringup'), 'maps', 'new_map_grid.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'controller_type', default_value='rpp',
            description='Which hardware controller to run: rpp, stanley or vector'),
        DeclareLaunchArgument(
            'planner_type', default_value='hybrid',
            description='Global planner: hybrid (hardware code) or navfn (stock Jazzy)'),
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='Map YAML (default: grid built from the hardware new_map)'),
        OpaqueFunction(function=launch_setup),
    ])

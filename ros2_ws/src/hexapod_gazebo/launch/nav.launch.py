"""
nav.launch.py

Point-to-point navigation in Gazebo on a saved map:

    ros2 launch hexapod_gazebo nav.launch.py                          # ~/maps/sim_map.yaml
    ros2 launch hexapod_gazebo nav.launch.py map:=/path/to/map.yaml
    ros2 launch hexapod_gazebo nav.launch.py slam:=true               # map and navigate at once
    ros2 launch hexapod_gazebo nav.launch.py slam:=true world:=big_world   # same, 24 x 16 m building

slam:=true needs no saved map and no "2D Pose Estimate": slam_toolbox builds
the map live and Nav2 plans on it as it grows.

Then in RViz: "2D Pose Estimate" if the robot isn't where it should be,
"2D Goal Pose" to pick a goal. The plan appears in hexcon (and as
/hexapod/preview_path in RViz); type `go` to start, `cancel` to drop it.
approve:=false = start at once.

What starts:
    gazebo.launch.py (robot, servos, gait, lidar)   - without its own RViz / SLAM
    Nav2 (planner, controller, navigator; map_server + AMCL on a saved map)
                     - Nav2 defaults + config/nav2_overrides.yaml, light behaviour tree
    plan_approval    - shows the plan, waits for `go`
    cmd_vel_bridge   - Nav2's /cmd_vel -> w / s / a / d / x for the gait
    RViz             - Nav2's own view (map, costmaps, path, goal tool, robot)
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

REPLACE_WHOLE = {'FollowPath', 'GridBased'}   # swap the default controller and planner entirely


def merge(base, over):
    for k, v in over.items():
        if k in REPLACE_WHOLE or not (isinstance(v, dict) and isinstance(base.get(k), dict)):
            base[k] = v
        else:
            merge(base[k], v)
    return base


def nav2(context):
    """Nav2 nodes started directly (same set as the real robot): planner,
    controller, navigator (+ map_server and AMCL on a saved map)."""
    nav2_share = get_package_share_directory('nav2_bringup')
    gz_share = get_package_share_directory('hexapod_gazebo')
    bringup = get_package_share_directory('hexapod_bringup')

    with open(os.path.join(nav2_share, 'params', 'nav2_params.yaml')) as fh:
        params = yaml.safe_load(fh)
    with open(os.path.join(gz_share, 'config', 'nav2_overrides.yaml')) as fh:
        merge(params, yaml.safe_load(fh))
    params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = \
        os.path.join(bringup, 'config', 'nav_lite_bt.xml')
    params_file = os.path.join(tempfile.gettempdir(), 'hexapod_nav2_params.yaml')
    with open(params_file, 'w') as fh:
        yaml.safe_dump(params, fh, sort_keys=False)

    sim = {'use_sim_time': True}

    def nav_node(pkg, exe, extra=None, remappings=None):
        return Node(package=pkg, executable=exe, name=exe, output='screen',
                    parameters=[params_file, sim] + ([extra] if extra else []),
                    remappings=remappings or [])

    def manager(name, nodes):
        return Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                    name=name, output='screen',
                    parameters=[{'autostart': True, 'node_names': nodes,
                                 'bond_timeout': 10.0, 'use_sim_time': True}])

    actions = []
    if LaunchConfiguration('slam').perform(context).lower() not in ('true', '1'):
        map_file = os.path.expanduser(LaunchConfiguration('map').perform(context))
        if not os.path.isfile(map_file):
            raise RuntimeError(f'map not found: {map_file} (make one with gazebo.launch.py slam:=true,'
                               ' or use nav.launch.py slam:=true)')
        actions += [
            nav_node('nav2_map_server', 'map_server', extra={'yaml_filename': map_file}),
            nav_node('nav2_amcl', 'amcl'),
            manager('lifecycle_manager_localization', ['map_server', 'amcl']),
        ]
    # (slam:=true: slam_toolbox is started by gazebo.launch.py)

    actions += [
        nav_node('nav2_planner', 'planner_server'),
        nav_node('nav2_controller', 'controller_server'),
        # a plain /goal_pose click must not start it: plan_approval does that after `go`
        nav_node('nav2_bt_navigator', 'bt_navigator', remappings=[('goal_pose', 'goal_pose_direct'),
                                         ('navigate_to_pose', 'navigate_to_pose_approved')]),
        manager('lifecycle_manager_navigation', ['planner_server', 'controller_server', 'bt_navigator']),
    ]
    return actions


def generate_launch_description():
    gz_share = get_package_share_directory('hexapod_gazebo')
    nav2_share = get_package_share_directory('nav2_bringup')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='~/maps/sim_map.yaml'),
        DeclareLaunchArgument('approve', default_value='true',
                              description='true = show the plan and wait for `go` in hexcon'),
        DeclareLaunchArgument('world', default_value='hexapod_world',
                              description='hexapod_world (6 x 4 m) or big_world (24 x 16 m)'),
        DeclareLaunchArgument('slam', default_value='false',
                              description='true = build the map live instead of loading one'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gz_share, 'launch', 'gazebo.launch.py')),
            launch_arguments={'rviz': 'false', 'slam': LaunchConfiguration('slam'),
                              'world': LaunchConfiguration('world')}.items(),
        ),

        # Nav2 once Gazebo and the robot are up
        TimerAction(period=8.0, actions=[OpaqueFunction(function=nav2)]),

        Node(
            package='hexapod_bringup',
            executable='cmd_vel_bridge',
            name='cmd_vel_bridge',
            output='screen',
            parameters=[{'use_sim_time': True,
                         'linear_deadband': 0.01,
                         'angular_deadband': 0.05,
                         'timeout': 1.0}],
        ),

        # goal -> plan preview -> `go` in hexcon -> navigate
        Node(
            package='hexapod_bringup',
            executable='plan_approval',
            name='plan_approval',
            output='screen',
            parameters=[{'use_sim_time': True,
                         'auto_go': PythonExpression(["'", LaunchConfiguration('approve'),
                                                      "'.lower() not in ('true', '1')"])}],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(nav2_share, 'rviz', 'nav2_default_view.rviz')],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])

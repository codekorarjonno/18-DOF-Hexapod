"""
nav.launch.py  -  run on the Raspberry Pi

Nav2 point-to-point navigation on the REAL robot, with the robot model:

    ros2 launch hexapod_bringup nav.launch.py                         # ~/maps/my_room.yaml
    ros2 launch hexapod_bringup nav.launch.py map:=~/maps/other.yaml
    ros2 launch hexapod_bringup nav.launch.py slam:=true              # map and navigate at once

Light mode: only planner + controller + navigator (and map_server + AMCL on a
saved map) so the Pi stays cool and the USB-powered ESP32 keeps its power.

What starts:
    wifi bridge + robot model   (hexapod.launch.py)  -> /joint_states, TF of legs and laser
    RPLIDAR C1                  -> /scan            frame: laser
    rf2o_laser_odometry         -> odom -> base_footprint
    slam:=false  map_server + AMCL on the saved map, planner, controller, navigator
    slam:=true   slam_toolbox (live map), planner, controller, navigator
    cmd_vel_bridge  (nav mode)  -> nw / ns / na / nd / x
                                   = wave gait only, stand before every change of direction

Replaces hexapod_mapping/navigate.launch.py (no fixed base_link -> laser any more,
frames are base_footprint). Don't run it together with `hexapod`, `hexmap` or the
old navigate launch: only one bridge may talk to the ESP32.

Plan approval (approve:=true, default): click "2D Goal Pose" in RViz, the plan
appears in hexcon (and as /hexapod/preview_path in RViz); type `go` to start,
`cancel` to drop it. approve:=false = start at once like before.

Watch and send goals from the laptop:
    rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# the RPLIDAR C1's permanent name on the Pi
LIDAR_PORT = ('/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_'
              '2a69e82660c7ed11999c1b2f71c9e8b5-if00-port0')

REPLACE_WHOLE = {'FollowPath', 'GridBased'}   # swap the default controller and planner entirely


def merge(base, over):
    for k, v in over.items():
        if k in REPLACE_WHOLE or not (isinstance(v, dict) and isinstance(base.get(k), dict)):
            base[k] = v
        else:
            merge(base[k], v)
    return base


def is_true(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ('true', '1')


def nav2(context):
    """Light Nav2: only the nodes the hexapod needs (low CPU on the Pi)."""
    nav2_share = get_package_share_directory('nav2_bringup')
    me = get_package_share_directory('hexapod_bringup')

    # Nav2 defaults + the hexapod's changes
    with open(os.path.join(nav2_share, 'params', 'nav2_params.yaml')) as fh:
        params = yaml.safe_load(fh)
    with open(os.path.join(me, 'config', 'nav2_overrides.yaml')) as fh:
        merge(params, yaml.safe_load(fh))
    params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = \
        os.path.join(me, 'config', 'nav_lite_bt.xml')
    params_file = os.path.join(tempfile.gettempdir(), 'hexapod_real_nav2_params.yaml')
    with open(params_file, 'w') as fh:
        yaml.safe_dump(params, fh, sort_keys=False)

    def nav_node(pkg, exe, name=None, extra=None):
        return Node(package=pkg, executable=exe, name=name or exe, output='screen',
                    parameters=[params_file] + ([extra] if extra else []))

    def manager(name, nodes):
        return Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                    name=name, output='screen',
                    parameters=[{'autostart': True, 'node_names': nodes,
                                 'bond_timeout': 10.0}])    # Pi: give nodes time to start

    actions = []

    if is_true(context, 'slam'):
        # live map from slam_toolbox
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
            launch_arguments={'slam_params_file': os.path.join(me, 'config', 'slam.yaml'),
                              'use_sim_time': 'false'}.items()))
    else:
        # saved map + AMCL
        map_file = os.path.expanduser(LaunchConfiguration('map').perform(context))
        if not os.path.isfile(map_file):
            raise RuntimeError(f'map not found: {map_file}  (save one first, or use slam:=true)')
        actions += [
            nav_node('nav2_map_server', 'map_server', extra={'yaml_filename': map_file}),
            nav_node('nav2_amcl', 'amcl'),
            manager('lifecycle_manager_localization', ['map_server', 'amcl']),
        ]

    # planner + controller + navigator only (controller publishes /cmd_vel directly)
    actions.append(TimerAction(period=4.0, actions=[
        nav_node('nav2_planner', 'planner_server'),
        nav_node('nav2_controller', 'controller_server'),
        # bt_navigator must not start on a plain /goal_pose click: plan_approval does that
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             output='screen', parameters=[params_file],
             remappings=[('goal_pose', 'goal_pose_direct'),
                         ('navigate_to_pose', 'navigate_to_pose_approved')]),
        manager('lifecycle_manager_navigation',
                ['planner_server', 'controller_server', 'bt_navigator']),
    ]))
    return actions


def generate_launch_description():
    me = get_package_share_directory('hexapod_bringup')
    walk = LaunchConfiguration('walk')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='~/maps/my_room.yaml'),
        DeclareLaunchArgument('slam', default_value='false',
                              description='true = build the map live instead of loading one'),
        DeclareLaunchArgument('approve', default_value='true',
                              description='true = show the plan and wait for `go` in hexcon'),
        DeclareLaunchArgument('walk', default_value='true',
                              description='false = Nav2 plans but the robot does not move'),
        DeclareLaunchArgument('laser_yaw', default_value='0.0',
                              description='lidar mounting turn in radians'),
        DeclareLaunchArgument('host', default_value='auto',
                              description='ESP32 address for the bridge'),
        DeclareLaunchArgument('lidar_port', default_value=LIDAR_PORT),

        # wifi bridge + robot_state_publisher (the robot model)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(me, 'launch', 'hexapod.launch.py')),
            launch_arguments={'host': LaunchConfiguration('host'),
                              'laser_yaw': LaunchConfiguration('laser_yaw'),
                              'twin': 'true'}.items(),
        ),

        # RPLIDAR C1
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': LaunchConfiguration('lidar_port'),
                'serial_baudrate': 460800,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
        ),

        # odometry from the laser scans
        TimerAction(period=3.0, actions=[
            Node(
                package='rf2o_laser_odometry',
                executable='rf2o_laser_odometry_node',
                name='rf2o_laser_odometry',
                output='screen',
                parameters=[{
                    'laser_scan_topic': '/scan',
                    'odom_topic': '/odom',
                    'publish_tf': True,
                    'base_frame_id': 'base_footprint',
                    'odom_frame_id': 'odom',
                    'init_pose_from_topic': '',
                    'freq': 10.0,
                }],
            ),
        ]),

        # Nav2 (and SLAM if slam:=true) once lidar and odometry run
        TimerAction(period=6.0, actions=[OpaqueFunction(function=nav2)]),

        # goal -> plan preview -> `go` in hexcon -> navigate
        Node(
            package='hexapod_bringup',
            executable='plan_approval',
            name='plan_approval',
            output='screen',
            parameters=[{'auto_go': PythonExpression(["'", LaunchConfiguration('approve'),
                                                      "'.lower() not in ('true', '1')"])}],
        ),

        # Nav2 /cmd_vel -> nw / ns / na / nd / x  (wave gait, stand before turns)
        Node(
            package='hexapod_bringup',
            executable='cmd_vel_bridge',
            name='cmd_vel_bridge',
            output='screen',
            parameters=[{'nav_mode': True,
                         'linear_deadband': 0.01,
                         'angular_deadband': 0.05,
                         'min_hold': 1.0,
                         'timeout': 1.0}],
            condition=IfCondition(walk),
        ),
    ])

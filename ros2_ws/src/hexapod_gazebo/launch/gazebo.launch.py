"""
gazebo.launch.py

The hexapod in Gazebo (Harmonic), walking with the same gait as the ESP32.

    ros2 launch hexapod_gazebo gazebo.launch.py              # Gazebo + RViz
    ros2 launch hexapod_gazebo gazebo.launch.py slam:=true   # + live mapping
    ros2 launch hexapod_gazebo gazebo.launch.py slam:=true world:=big_world   # 24 x 16 m building
    ros2 launch hexapod_gazebo gazebo.launch.py foot_mu:=0.3 # slipperier feet
    ros2 launch hexapod_gazebo gazebo.launch.py servo_effort:=1.1   # real MG996R torque
    ros2 run hexapod_bringup console                         # second terminal: w, x, g, t ...

What starts:
    Gazebo with worlds/<world>.sdf (hexapod_world, or big_world)
    robot_state_publisher  (hexapod_gz.urdf.xacro, sim time)
    the robot, spawned standing
    joint_state_broadcaster + position_controller  (the 18 servos + gripper)
    ros_gz_bridge          (/clock /scan /imu /odom /tf)
    hexapod_sim            (the firmware gait, sending servo commands)
    RViz                   (optional)
    slam_toolbox           (optional, same settings as on the real robot)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gz_share = get_package_share_directory('hexapod_gazebo')
    desc_share = get_package_share_directory('hexapod_description')

    world = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('rviz')
    use_slam = LaunchConfiguration('slam')

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(gz_share, 'urdf', 'hexapod_gz.urdf.xacro'),
                 ' foot_mu:=', LaunchConfiguration('foot_mu'),
                 ' servo_effort:=', LaunchConfiguration('servo_effort')]),
        value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': ['-r -v 2 ', os.path.join(gz_share, 'worlds', ''), world, '.sdf'],
                          'on_exit_shutdown': 'true'}.items(),
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'hexapod',
                   '-x', '0', '-y', '0', '-z', '0.03'],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': os.path.join(gz_share, 'config', 'bridge.yaml'),
                     'use_sim_time': True}],
    )

    joint_states = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    servos = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['position_controller'],
        output='screen',
    )

    gait = Node(
        package='hexapod_bringup',
        executable='sim',
        name='hexapod_sim',
        output='screen',
        parameters=[{'gazebo': True, 'use_sim_time': True}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(gz_share, 'rviz', 'gazebo.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])),
        launch_arguments={
            'slam_params_file': PathJoinSubstitution(
                [FindPackageShare('hexapod_bringup'), 'config', 'slam.yaml']),
            'use_sim_time': 'true',
        }.items(),
        condition=IfCondition(use_slam),
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='hexapod_world',
                              description='world name in worlds/: hexapod_world (6 x 4 m) or big_world (24 x 16 m)'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('slam', default_value='false'),
        DeclareLaunchArgument('servo_effort', default_value='2.0',
                              description='servo torque limit in N m (MG996R is ~1.1)'),
        DeclareLaunchArgument('foot_mu', default_value='0.35',
                              description='foot friction: lower = feet slip more'),

        # lets Gazebo find package://hexapod_description/meshes/...
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(desc_share)),

        gazebo,
        rsp,
        bridge,
        spawn,

        # controllers only after the robot exists, the gait only after the servos
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[joint_states])),
        RegisterEventHandler(OnProcessExit(target_action=joint_states, on_exit=[servos])),
        RegisterEventHandler(OnProcessExit(target_action=servos, on_exit=[gait])),

        rviz,
        TimerAction(period=5.0, actions=[slam]),
    ])

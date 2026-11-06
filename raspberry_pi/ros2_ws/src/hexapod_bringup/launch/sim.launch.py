"""
sim.launch.py

Walk the hexapod in RViz with no hardware: the simulator runs the same
gait as the ESP32 firmware and drives the robot model.

    ros2 launch hexapod_bringup sim.launch.py
    ros2 run hexapod_bringup console          # in a second terminal

    ros2 launch hexapod_bringup sim.launch.py speedup:=2.0   # faster
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc = FindPackageShare('hexapod_description')
    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([desc, 'urdf', 'hexapod.urdf.xacro'])]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('speedup', default_value='1.0',
                              description='run the gait faster (2.0) or slower (0.5)'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        Node(
            package='hexapod_bringup',
            executable='sim',
            name='hexapod_sim',
            output='screen',
            parameters=[{'speedup': ParameterValue(LaunchConfiguration('speedup'), value_type=float)}],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([desc, 'rviz', 'display.rviz'])],
        ),
    ])

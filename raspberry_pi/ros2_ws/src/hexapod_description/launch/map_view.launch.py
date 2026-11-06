"""
map_view.launch.py  -  run on the laptop

RViz with the map, the laser scan and the robot model, while the Pi runs
    ros2 launch hexapod_bringup map.launch.py

    ros2 launch hexapod_description map_view.launch.py

The camera follows the robot (Target Frame: base_footprint).
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution(
                [FindPackageShare('hexapod_description'), 'rviz', 'map.rviz'])],
            output='screen',
        ),
    ])

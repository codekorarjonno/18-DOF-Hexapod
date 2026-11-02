"""
rviz.launch.py

RViz only - for watching the digital twin on the laptop while the Pi
runs hexapod_bringup (which publishes the model and the live joints):

    ros2 launch hexapod_description rviz.launch.py

Don't run display.launch.py at the same time: its slider window would
fight the real robot's joint angles.
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_file = PathJoinSubstitution(
        [FindPackageShare('hexapod_description'), 'rviz', 'display.rviz'])

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_file],
            output='screen',
        ),
    ])

"""
display.launch.py

Shows the hexapod URDF in RViz with sliders for every joint.

    ros2 launch hexapod_description display.launch.py                 # laptop: RViz + sliders
    ros2 launch hexapod_description display.launch.py use_meshes:=false     # boxes only
    ros2 launch hexapod_description display.launch.py gui:=false rviz:=false   # Pi: TF only

gui:=false uses the plain joint_state_publisher (all joints at 0, the
all-90 stance) so every TF frame still exists for slam_toolbox and Nav2.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('hexapod_description')

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    use_meshes = LaunchConfiguration('use_meshes')

    xacro_file = PathJoinSubstitution([pkg, 'urdf', 'hexapod.urdf.xacro'])
    rviz_file = PathJoinSubstitution([pkg, 'rviz', 'display.rviz'])

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_meshes:=', use_meshes]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='joint slider window'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='start RViz'),
        DeclareLaunchArgument('use_meshes', default_value='true',
                              description='false = plain boxes (faster on a slow PC)'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(gui),
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=UnlessCondition(gui),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_file],
            condition=IfCondition(rviz),
        ),
    ])

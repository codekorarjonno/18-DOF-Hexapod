"""
Brings up the wifi bridge, the digital twin (robot_state_publisher),
and optionally the cmd_vel translator.

    ros2 launch hexapod_bringup hexapod.launch.py
    ros2 launch hexapod_bringup hexapod.launch.py host:=192.168.0.42
    ros2 launch hexapod_bringup hexapod.launch.py cmd_vel:=true
    ros2 launch hexapod_bringup hexapod.launch.py twin:=false

With twin:=true (default) the bridge publishes /joint_states from the
board's joint reports and robot_state_publisher turns them into TF, so
RViz on the laptop shows the model moving with the real robot:

    ros2 launch hexapod_description rviz.launch.py      # on the laptop
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')
    use_cmd_vel = LaunchConfiguration('cmd_vel')
    twin = LaunchConfiguration('twin')

    xacro_file = PathJoinSubstitution(
        [FindPackageShare('hexapod_description'), 'urdf', 'hexapod.urdf.xacro'])
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' laser_yaw:=', LaunchConfiguration('laser_yaw')]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='auto'),
        DeclareLaunchArgument('port', default_value='8888'),
        DeclareLaunchArgument('cmd_vel', default_value='false'),
        DeclareLaunchArgument('twin', default_value='true',
                              description='publish the robot model and its live joint angles'),
        DeclareLaunchArgument('laser_yaw', default_value='0.0',
                              description='lidar mounting turn in radians (3.14159 = backwards)'),

        Node(
            package='hexapod_bringup',
            executable='serial_bridge',
            name='serial_bridge',
            output='screen',
            parameters=[{
                'host': host,
                'port': port,
                'echo_feedback': True,
                'joint_states': twin,
            }],
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
            condition=IfCondition(twin),
        ),

        Node(
            package='hexapod_bringup',
            executable='cmd_vel_bridge',
            name='cmd_vel_bridge',
            output='screen',
            condition=IfCondition(use_cmd_vel),
        ),
    ])

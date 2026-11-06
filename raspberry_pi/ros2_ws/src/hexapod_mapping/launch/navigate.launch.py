import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory as share

LIDAR_PORT = '/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_2a69e82660c7ed11999c1b2f71c9e8b5-if00-port0'
LIDAR_Z = '0.15'

def generate_launch_description():
    walk = LaunchConfiguration('walk')
    params = os.path.join(share('hexapod_mapping'), 'config', 'nav2_params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.expanduser('~/maps/my_room.yaml')),
        DeclareLaunchArgument('walk', default_value='true'),

        Node(package='sllidar_ros2', executable='sllidar_node', name='sllidar_node',
             parameters=[{'channel_type': 'serial', 'serial_port': LIDAR_PORT,
                          'serial_baudrate': 460800, 'frame_id': 'laser',
                          'inverted': False, 'angle_compensate': True,
                          'scan_mode': 'Standard'}], output='screen'),

        Node(package='tf2_ros', executable='static_transform_publisher', name='base_to_laser',
             arguments=['--z', LIDAR_Z, '--frame-id', 'base_link', '--child-frame-id', 'laser']),

        TimerAction(period=3.0, actions=[
            Node(package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
                 name='rf2o_laser_odometry', output='screen',
                 parameters=[{'laser_scan_topic': '/scan', 'odom_topic': '/odom',
                              'publish_tf': True, 'base_frame_id': 'base_link',
                              'odom_frame_id': 'odom', 'init_pose_from_topic': '',
                              'freq': 10.0}])]),

        TimerAction(period=5.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    share('nav2_bringup'), 'launch', 'bringup_launch.py')),
                launch_arguments={'map': LaunchConfiguration('map'),
                                  'params_file': params,
                                  'use_sim_time': 'false',
                                  'autostart': 'true'}.items())]),

        Node(package='hexapod_bringup', executable='serial_bridge', name='serial_bridge',
             parameters=[{'host': 'auto', 'port': 8888}], output='screen',
             condition=IfCondition(walk)),
        Node(package='hexapod_bringup', executable='cmd_vel_bridge', name='cmd_vel_bridge',
             parameters=[{'linear_deadband': 0.02}], output='screen',
             condition=IfCondition(walk)),
    ])

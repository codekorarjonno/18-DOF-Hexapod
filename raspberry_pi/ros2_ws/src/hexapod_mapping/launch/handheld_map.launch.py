from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

LIDAR_PORT = '/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_2a69e82660c7ed11999c1b2f71c9e8b5-if00-port0'

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='sllidar_ros2', executable='sllidar_node', name='sllidar_node',
            parameters=[{'channel_type': 'serial',
                         'serial_port': LIDAR_PORT,
                         'serial_baudrate': 460800,
                         'frame_id': 'laser',
                         'inverted': False,
                         'angle_compensate': True,
                         'scan_mode': 'Standard'}],
            output='screen'),

        # physical mounting offset — measure your actual lidar height on the soccerbot
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='base_to_laser',
             arguments=['--x','0','--y','0','--z','0.15',
                        '--roll','0','--pitch','0','--yaw','0',
                        '--frame-id','base_link','--child-frame-id','laser']),

        TimerAction(period=3.0, actions=[
            Node(package='rf2o_laser_odometry',
                 executable='rf2o_laser_odometry_node',
                 name='rf2o_laser_odometry',
                 output='screen',
                 parameters=[{
                     'laser_scan_topic': '/scan',
                     'odom_topic': '/odom',
                     'publish_tf': True,
                     'base_frame_id': 'base_link',
                     'odom_frame_id': 'odom',
                     'init_pose_from_topic': '',
                     'freq': 10.0}]),
        ]),

        TimerAction(period=5.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('slam_toolbox'),
                    'launch','online_async_launch.py')),
                launch_arguments={'slam_params_file': os.path.join(
                    get_package_share_directory('hexapod_mapping'),
                    'config','mapper_params.yaml')}.items()),
        ]),
    ])

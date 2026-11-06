"""
map.launch.py  -  run on the Raspberry Pi

Live mapping with the robot model:

    RPLIDAR C1 (sllidar_ros2)  ->  /scan                  frame: laser
    robot model                ->  base_footprint -> base_link -> legs, laser
    rf2o_laser_odometry        ->  odom -> base_footprint
    slam_toolbox               ->  map  -> odom,  /map
    wifi bridge                ->  /joint_states (legs move with the real robot)

    ros2 launch hexapod_bringup map.launch.py
    ros2 launch hexapod_bringup map.launch.py lidar_port:=/dev/ttyUSB0
    ros2 launch hexapod_bringup map.launch.py laser_yaw:=3.14159   # scan backwards

Replaces hexapod_mapping/handheld_map.launch.py (same lidar, rf2o and SLAM
settings) but the lidar position now comes from the robot model instead of
a fixed base_link -> laser transform. Don't run both at once.

Watch it on the laptop:  ros2 launch hexapod_description map_view.launch.py
Save the map:            ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# the RPLIDAR C1's permanent name on the Pi (does not change after a reboot)
LIDAR_PORT = ('/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_'
              '2a69e82660c7ed11999c1b2f71c9e8b5-if00-port0')


def generate_launch_description():
    bringup = FindPackageShare('hexapod_bringup')
    lidar_port = LaunchConfiguration('lidar_port')

    return LaunchDescription([
        DeclareLaunchArgument('lidar_port', default_value=LIDAR_PORT),
        DeclareLaunchArgument('laser_yaw', default_value='0.0',
                              description='lidar mounting turn in radians'),
        DeclareLaunchArgument('host', default_value='auto',
                              description='ESP32 address for the bridge'),

        # wifi bridge + robot_state_publisher (the robot model)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([bringup, 'launch', 'hexapod.launch.py'])),
            launch_arguments={
                'host': LaunchConfiguration('host'),
                'laser_yaw': LaunchConfiguration('laser_yaw'),
                'twin': 'true',
            }.items(),
        ),

        # RPLIDAR C1
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': lidar_port,
                'serial_baudrate': 460800,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
        ),

        # odometry from the laser scans (after the lidar has started)
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

        # SLAM after rf2o (its own launch file handles the lifecycle start-up)
        TimerAction(period=5.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch',
                                          'online_async_launch.py'])),
                launch_arguments={
                    'slam_params_file': PathJoinSubstitution([bringup, 'config', 'slam.yaml']),
                    'use_sim_time': 'false',
                }.items(),
            ),
        ]),
    ])

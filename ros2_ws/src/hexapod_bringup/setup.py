from setuptools import setup

package_name = 'hexapod_bringup'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/hexapod.launch.py', 'launch/sim.launch.py', 'launch/map.launch.py', 'launch/nav.launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml', 'config/slam.yaml', 'config/nav2_overrides.yaml', 'config/nav_lite_bt.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='ROS 2 wifi bridge, terminal console and digital twin for the ESP32 hexapod',
    license='MIT',
    entry_points={
        'console_scripts': [
            'serial_bridge = hexapod_bringup.serial_bridge:main',
            'console = hexapod_bringup.console:main',
            'cmd_vel_bridge = hexapod_bringup.cmd_vel_bridge:main',
            'sim = hexapod_bringup.sim_hexapod:main',
            'leg_check = hexapod_bringup.leg_check:main',
            'plan_approval = hexapod_bringup.plan_approval:main',
        ],
    },
)

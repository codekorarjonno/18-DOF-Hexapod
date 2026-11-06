from setuptools import find_packages, setup

package_name = 'hexapod_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ariyan',
    maintainer_email='ariyan@todo.todo',
    description='Keyboard to ESP32 bridge for the hexapod',
    license='MIT',
    entry_points={
        'console_scripts': [
            'teleop_key = hexapod_control.teleop_key:main',
            'esp32_bridge = hexapod_control.esp32_bridge:main',
            'camera_node = hexapod_control.camera_node:main',
        ],
    },
)

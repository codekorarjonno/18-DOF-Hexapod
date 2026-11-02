#!/usr/bin/env python3
"""
leg_check.py  -  Gazebo diagnostic

Compares what the gait commands each servo (/position_controller/commands)
with where the joint really is (/joint_states), and every 2 seconds prints
the worst error per leg. A leg whose servo can't follow (blocked, stuck on
the floor, too weak) shows up with a large error and a "<--" mark.

    ros2 run hexapod_bringup leg_check
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

NAMES = [f'leg{n}_{j}_joint' for n in range(6) for j in ('coxa', 'femur', 'tibia')]
LEG = ['0 front-R', '1 middle-R', '2 back-R', '3 front-L', '4 middle-L', '5 back-L']
WARN_DEG = 5.0


class LegCheck(Node):

    def __init__(self):
        super().__init__('leg_check')
        self.cmd = None
        self.worst = [0.0] * 18
        self.create_subscription(Float64MultiArray, 'position_controller/commands', self.on_cmd, 10)
        self.create_subscription(JointState, 'joint_states', self.on_state, 50)
        self.create_timer(2.0, self.report)
        self.get_logger().info('watching servo errors - walk the robot (w, a, d ...)')

    def on_cmd(self, msg):
        self.cmd = list(msg.data[:18])

    def on_state(self, msg):
        if self.cmd is None:
            return
        pos = dict(zip(msg.name, msg.position))
        for i, n in enumerate(NAMES):
            if n in pos:
                self.worst[i] = max(self.worst[i], abs(math.degrees(self.cmd[i] - pos[n])))

    def report(self):
        if self.cmd is None:
            print('no servo commands yet (is the gait node running?)', flush=True)
            return
        print('\nworst error in the last 2 s (degrees)      coxa  femur  tibia', flush=True)
        for n in range(6):
            e = self.worst[3 * n:3 * n + 3]
            flag = '  <-- not following' if max(e) > WARN_DEG else ''
            print(f'  leg {LEG[n]:<12}                     {e[0]:5.1f}  {e[1]:5.1f}  {e[2]:5.1f}{flag}', flush=True)
        self.worst = [0.0] * 18


def main():
    rclpy.init()
    node = LegCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

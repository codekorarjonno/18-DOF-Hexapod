import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

KEYS = {'w': 'F', 's': 'B', 'a': 'L', 'd': 'R', ' ': 'S'}


class TeleopKey(Node):
    def __init__(self):
        super().__init__('teleop_key')
        self.pub = self.create_publisher(String, '/hexapod/cmd', 10)

    def send(self, letter):
        msg = String()
        msg.data = letter
        self.pub.publish(msg)


def read_key(settings):
    tty.setraw(sys.stdin.fileno())
    ready, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if ready else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = TeleopKey()
    print('w forward, s back, a left, d right, space stop, q quit')
    try:
        while rclpy.ok():
            key = read_key(settings)
            if key == 'q':
                break
            if key in KEYS:
                node.send(KEYS[key])
                print(f'sent {KEYS[key]}')
            rclpy.spin_once(node, timeout_sec=0)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

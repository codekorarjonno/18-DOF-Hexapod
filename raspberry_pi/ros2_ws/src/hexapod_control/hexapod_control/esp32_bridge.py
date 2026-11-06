import time

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String


class Esp32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2.0)
        self.ser.reset_input_buffer()

        self.create_subscription(String, '/hexapod/cmd', self.on_cmd, 10)
        self.create_timer(0.05, self.drain)
        self.get_logger().info(f'open {port} @ {baud}')

    def on_cmd(self, msg):
        letter = msg.data.strip().upper()[:1]
        if not letter:
            return
        self.ser.write((letter + '\n').encode())
        self.get_logger().info(f'tx {letter}')

    def drain(self):
        while self.ser.in_waiting:
            line = self.ser.readline().decode(errors='ignore').strip()
            if line:
                self.get_logger().info(f'rx {line}')


def main():
    rclpy.init()
    node = Esp32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

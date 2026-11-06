import io
from threading import Condition

import rclpy
from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class StreamBuffer(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('rate', 15.0)
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        rate = self.get_parameter('rate').value

        self.pub = self.create_publisher(CompressedImage, '/hexapod/image/compressed', 5)
        self.buffer = StreamBuffer()
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_video_configuration(main={'size': (width, height)}))
        self.picam2.start_recording(JpegEncoder(), FileOutput(self.buffer))

        self.create_timer(1.0 / rate, self.publish_frame)
        self.get_logger().info(f'streaming {width}x{height} at {rate} fps')

    def publish_frame(self):
        with self.buffer.condition:
            frame = self.buffer.frame
        if frame is None:
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.format = 'jpeg'
        msg.data = bytes(frame)
        self.pub.publish(msg)

    def shutdown(self):
        self.picam2.stop_recording()


def main():
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

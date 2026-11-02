#!/usr/bin/env python3
"""
serial_bridge.py

Bridges ROS 2 to the hexapod's ESP32 over wifi (TCP).

    subscribes:  /hexapod/command   std_msgs/String
    publishes:   /hexapod/feedback  std_msgs/String
    publishes:   /hexapod/link      std_msgs/Bool   (latched)
    publishes:   /joint_states      sensor_msgs/JointState  (digital twin)

The firmware reports every commanded joint angle as a line
"J c0 f0 t0 ... c5 f5 t5 g" (degrees, 90 = neutral; g = gripper servo).
Those lines become /joint_states for robot_state_publisher, so the
RViz model moves with the real robot. They are not echoed as feedback.
If no report has arrived for a second (old firmware, robot off, link
down), the last known pose (at start: the all-90 stance) is repeated
twice a second so the model never loses its legs in RViz.

Parameters:
    host       ESP32 address, or "auto" to discover it (default)
    mdns_name  hostname the firmware advertises
    port       TCP port the firmware listens on (8888)
    scan       allow subnet scanning as a last resort (true)
    timeout    socket read timeout in seconds (0.2)
    retry      seconds between reconnection attempts (2.0)
    joint_states  publish /joint_states from the twin reports (true)

Discovery order when host is "auto": mDNS name, then the last address
that worked, then a scan of every subnet this machine is attached to.
The link is re-established automatically if the board reboots.
"""

import concurrent.futures
import ipaddress
import math
import os
import socket
import subprocess
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

CACHE_FILE = os.path.expanduser('~/.hexapod_ip')

# Same order as the firmware report: legs 0..5, coxa/femur/tibia
LEG_JOINTS = [f'leg{n}_{j}_joint' for n in range(6) for j in ('coxa', 'femur', 'tibia')]
GRIPPER_OPEN, GRIPPER_CLOSE = 150.0, 100.0          # firmware calibration
JAW_MAX = math.radians((GRIPPER_OPEN - GRIPPER_CLOSE) / 2)   # each jaw, URDF upper limit


class SerialBridge(Node):

    def __init__(self):
        super().__init__('serial_bridge')

        self.declare_parameter('host', 'auto')
        self.declare_parameter('mdns_name', 'hexapod-esp.local')
        self.declare_parameter('port', 8888)
        self.declare_parameter('scan', True)
        self.declare_parameter('timeout', 0.2)
        self.declare_parameter('retry', 2.0)
        self.declare_parameter('echo_feedback', True)
        self.declare_parameter('joint_states', True)

        self.host = self.get_parameter('host').value
        self.mdns_name = self.get_parameter('mdns_name').value
        self.port = int(self.get_parameter('port').value)
        self.allow_scan = bool(self.get_parameter('scan').value)
        self.timeout = float(self.get_parameter('timeout').value)
        self.retry = float(self.get_parameter('retry').value)
        self.echo = bool(self.get_parameter('echo_feedback').value)
        self.publish_joints = bool(self.get_parameter('joint_states').value)

        self.sock = None
        self.sock_lock = threading.Lock()
        self.running = True
        self.linked = False
        self.last_good = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.feedback_pub = self.create_publisher(String, 'hexapod/feedback', 10)
        self.link_pub = self.create_publisher(Bool, 'hexapod/link', latched)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.last_joints = [0.0] * len(LEG_JOINTS) + [0.0]     # stance, gripper open
        self.last_twin = 0.0
        if self.publish_joints:
            self.create_timer(0.5, self.hold_pose)
        self.set_linked(False)

        self.create_subscription(String, 'hexapod/command', self.on_command, 10)

        self.worker = threading.Thread(target=self.link_loop, daemon=True)
        self.worker.start()

        self.get_logger().info('bridge started, publish to /hexapod/command')

    def set_linked(self, state: bool):
        if state == self.linked:
            return
        self.linked = state
        try:
            self.link_pub.publish(Bool(data=state))
        except Exception:
            pass

        if state:
            self.log('info', f'link up ({self.host})')
        else:
            self.log('warn', 'link down, retrying')
            try:
                self.feedback_pub.publish(String(data='[link lost]'))
            except Exception:
                pass

    def probe(self, address: str, timeout: float = 0.4) -> bool:
        try:
            with socket.create_connection((address, self.port), timeout=timeout):
                return True
        except (OSError, socket.gaierror):
            return False

    def cached(self):
        if self.last_good:
            return self.last_good
        try:
            with open(CACHE_FILE) as fh:
                return fh.read().strip() or None
        except OSError:
            return None

    def remember(self, address: str):
        self.last_good = address
        try:
            with open(CACHE_FILE, 'w') as fh:
                fh.write(address)
        except OSError:
            pass

    def local_targets(self):
        try:
            out = subprocess.run(
                ['ip', '-4', '-o', 'addr', 'show', 'scope', 'global'],
                capture_output=True, text=True, timeout=5.0).stdout
        except (OSError, subprocess.SubprocessError):
            return []

        targets = []
        for line in out.splitlines():
            for token in line.split():
                if token.count('.') == 3 and '/' in token:
                    try:
                        net = ipaddress.ip_interface(token).network
                    except ValueError:
                        continue
                    if net.num_addresses <= 1024:
                        targets.extend(str(h) for h in net.hosts())
        return targets

    def scan(self):
        targets = self.local_targets()
        if not targets:
            return None

        self.log('info', f'scanning {len(targets)} addresses on port {self.port}')

        with concurrent.futures.ThreadPoolExecutor(max_workers=64) as pool:
            for address, ok in zip(targets, pool.map(self.probe, targets)):
                if ok:
                    return address
        return None

    def discover(self, quick: bool = False):
        if self.host and self.host != 'auto':
            return self.host if self.probe(self.host, 2.0) else None

        last = self.cached()
        if last and self.probe(last, 1.0):
            return last

        if self.probe(self.mdns_name, 2.0):
            return self.mdns_name

        if quick or not self.allow_scan:
            return None

        return self.scan()

    def open_link(self, quick: bool = False) -> bool:
        address = self.discover(quick)
        if address is None:
            return False

        try:
            sock = socket.create_connection((address, self.port), timeout=5.0)
        except (OSError, socket.gaierror):
            return False

        sock.settimeout(self.timeout)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        with self.sock_lock:
            self.sock = sock

        self.host = address
        if address != self.mdns_name:
            self.remember(address)
        return True

    def close_link(self):
        with self.sock_lock:
            sock, self.sock = self.sock, None
        if sock is not None:
            try:
                sock.close()
            except Exception:
                pass

    def link_loop(self):
        attempt = 0
        while self.running:
            if self.sock is None:
                if self.open_link(quick=(attempt > 0 and attempt % 5 != 0)):
                    attempt = 0
                    self.set_linked(True)
                else:
                    attempt += 1
                    if attempt == 1:
                        self.log('error', 'no hexapod found, still trying')
                    threading.Event().wait(self.retry)
                    continue

            self.read_until_lost()
            self.close_link()
            self.set_linked(False)
            threading.Event().wait(self.retry)

    def on_twin(self, line: str):
        """'J c0 f0 t0 ... c5 f5 t5 g' -> /joint_states"""
        if not self.publish_joints or not rclpy.ok():
            return
        try:
            vals = [float(v) for v in line.split()[1:]]
        except ValueError:
            return
        if len(vals) != len(LEG_JOINTS) + 1:
            return

        legs = [math.radians(v - 90.0) for v in vals[:-1]]
        jaw = math.radians((GRIPPER_OPEN - vals[-1]) / 2.0)
        self.last_joints = legs + [min(max(jaw, 0.0), JAW_MAX)]
        self.last_twin = self.get_clock().now().nanoseconds / 1e9
        self.send_joints()

    def send_joints(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = LEG_JOINTS + ['gripper_joint']
        msg.position = list(self.last_joints)
        try:
            self.joint_pub.publish(msg)
        except Exception:
            pass

    def hold_pose(self):
        if self.get_clock().now().nanoseconds / 1e9 - self.last_twin > 1.0:
            self.send_joints()

    def read_until_lost(self):
        buf = b''
        while self.running and self.sock is not None:
            try:
                chunk = self.sock.recv(512)
            except socket.timeout:
                continue
            except OSError:
                return

            if not chunk:
                return

            buf += chunk
            while b'\n' in buf:
                raw, buf = buf.split(b'\n', 1)
                line = raw.decode('utf-8', errors='replace').strip()
                if not line:
                    continue

                if line.startswith('J '):
                    self.on_twin(line)
                    continue

                if not self.running or not rclpy.ok():
                    print(f'[serial_bridge] <- {line}', flush=True)
                    continue

                try:
                    self.feedback_pub.publish(String(data=line))
                except Exception:
                    print(f'[serial_bridge] <- {line}', flush=True)
                    continue

                if self.echo:
                    self.log('info', f'<- {line}')

    def log(self, level: str, text: str):
        try:
            getattr(self.get_logger(), level)(text)
        except Exception:
            print(f'[serial_bridge] {text}', flush=True)

    def send(self, text: str) -> bool:
        text = text.strip()
        if not text:
            return False

        with self.sock_lock:
            sock = self.sock
            if sock is None:
                self.log('warn', f'link down, dropped: {text}')
                return False
            try:
                sock.sendall((text + '\n').encode('ascii', errors='ignore'))
            except OSError as exc:
                self.log('error', f'send failed: {exc}')
                return False

        self.log('info', f'-> {text}')
        return True

    def on_command(self, msg: String):
        if len(msg.data.strip()) > 48:
            self.get_logger().warn('command longer than 48 chars, ignored')
            return
        self.send(msg.data)

    def shutdown(self):
        if self.sock is not None:
            try:
                self.send('x')
                threading.Event().wait(0.5)
            except Exception:
                pass

        self.running = False

        if self.worker.is_alive():
            self.worker.join(timeout=2.0)

        self.close_link()
        print('[serial_bridge] closed', flush=True)


def main():
    rclpy.init()
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

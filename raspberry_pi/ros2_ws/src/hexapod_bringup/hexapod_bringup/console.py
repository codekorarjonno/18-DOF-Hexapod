#!/usr/bin/env python3
"""
console.py

Interactive terminal for the hexapod. Type the same commands you used
in the Arduino Serial Monitor; they are published to /hexapod/command
and the bridge forwards them to the ESP32.

The prompt shows link state: "hexapod>" when connected, "hexapod [no
link]>" when the bridge has lost the board. Commands typed while the
link is down are refused locally rather than silently dropped.

    ros2 run hexapod_bringup console
"""

import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

LOCAL_HELP = """
  local commands
    quit / exit      leave this console (robot keeps its last state)
    help             this text

  navigation (Nav2 shows the plan first, nothing moves until you approve)
    go               start the planned path
    cancel           drop the plan, or stop a running navigation

  forwarded to the ESP32 (type ? for the board's own help)
    w s a d          forward / backward / turn left / turn right
    x                stop and settle
    g  r  grip       grab / release / toggle gripper
    1 / 1 d          single gait cycle
    h c p t          stance / center / print pose / leg test
    swing 20         and turn, lift, tuck, stance, speed
    40 5 120         raw board channel write
    twin on / off    joint reports for the RViz twin
    navgait wave     gait Nav2 uses: wave (default) or tripod
    navtsteps 3      tripod in Nav2: rest stand every 3 cycles (0 = off)
    fdrift 2         straight-line fix forward (bdrift = backward)
"""


class Console(Node):

    def __init__(self):
        super().__init__('console')
        self.linked = False

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(String, 'hexapod/command', 10)
        self.approve_pub = self.create_publisher(String, 'hexapod/approve', 10)
        self.create_subscription(String, 'hexapod/feedback', self.on_feedback, 10)
        self.create_subscription(Bool, 'hexapod/link', self.on_link, latched)

    def prompt(self):
        return 'hexapod> ' if self.linked else 'hexapod [no link]> '

    def redraw(self, text: str):
        sys.stdout.write('\r\033[K' + text + '\n' + self.prompt())
        sys.stdout.flush()

    def on_feedback(self, msg: String):
        self.redraw(msg.data)

    def on_link(self, msg: Bool):
        if msg.data == self.linked:
            return
        self.linked = msg.data
        self.redraw('*** link up ***' if msg.data else '*** link down ***')

    def send(self, text: str):
        self.pub.publish(String(data=text))

    def approve(self, word: str):
        self.approve_pub.publish(String(data=word))


def main():
    rclpy.init()
    node = Console()

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print('hexapod console - type help for local commands, ? for board help')
    print('Ctrl-D or "quit" to exit')

    try:
        while rclpy.ok():
            try:
                line = input(node.prompt()).strip()
            except EOFError:
                print()
                break

            if not line:
                continue

            low = line.lower()
            if low in ('quit', 'exit'):
                break
            if low == 'help':
                print(LOCAL_HELP)
                continue
            if low in ('go', 'cancel'):           # Nav2 plan approval (not for the board)
                node.approve(low)
                continue

            if not node.linked:
                print('  link is down, command not sent')
                continue

            node.send(line)
    except KeyboardInterrupt:
        print()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

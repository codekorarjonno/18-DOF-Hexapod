#!/usr/bin/env python3
"""
cmd_vel_bridge.py

Translates geometry_msgs/Twist on /cmd_vel (from Nav2, or teleop) into
the direction commands the gait understands.

Navigation safety (nav_mode, on by default): it sends nw / ns / na / nd
instead of w / s / a / d. The firmware (and the simulator) then use the
navigation gait (`navgait wave` default, or `navgait tripod`, typed in the
console), stand before every change of direction, and with tripod also take
a rest stand every 3 cycles (`navtsteps`). Manual commands are unaffected.

The gait has no notion of speed, only direction, so linear.x and
angular.z are reduced to forward / backward / turn left / turn right /
stop. The gait cannot walk in a curve, so when Nav2 asks for a sharp
curve (|angular| > max_curvature * |linear|, e.g. the goal is behind the
robot) it turns in place first; a gentle curve is walked straight.

Walking backward (allow_reverse, on by default): Nav2's controllers only
drive forward, so if the planned path leaves BEHIND the robot (more than
reverse_enter_deg off its nose) the bridge walks it backward (s) along the
path instead of turning around, correcting with small turns when the rear
is more than turn_deg off the path. It stops (x) if the lidar sees anything
closer than rear_stop_m behind. It reads Nav2's /plan and TF for this.

Fewer commands, to spare the servos (every change of direction costs a
full stand):
  - a new direction is only sent after Nav2 has asked for it steadily for
    `min_hold` seconds,
  - once sent, a direction is kept for at least `min_run` seconds,
  - after a stop, the robot rests at least `pause` seconds before moving.
Stop itself is always sent immediately (obstacle, goal reached).

Parameters:
    nav_mode           send navigation-safe commands (true)
    linear_deadband    (0.05)
    angular_deadband   (0.10)
    max_curvature      1/m; sharper requested curves become a turn in place (1.5)
    allow_reverse      walk backward when the path is behind the robot (true)
    reverse_enter_deg  path this far off the nose -> walk backward (100)
    reverse_exit_deg   back to forward below this (70)
    turn_deg           backward: turn if the rear is this far off the path (35)
    rear_stop_m        backward: stop if something is this close behind (0.45)
    min_hold           seconds a new direction must persist (1.0)
    min_run            seconds a direction is kept once sent (3.0)
    pause              seconds of rest after a stop before moving again (2.0)
    timeout            seconds without a Twist before stopping (0.6)
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class CmdVelBridge(Node):

    def __init__(self):
        super().__init__('cmd_vel_bridge')

        self.declare_parameter('nav_mode', True)
        self.declare_parameter('linear_deadband', 0.05)
        self.declare_parameter('angular_deadband', 0.10)
        self.declare_parameter('max_curvature', 1.5)
        self.declare_parameter('allow_reverse', True)
        self.declare_parameter('reverse_enter_deg', 100.0)
        self.declare_parameter('reverse_exit_deg', 70.0)
        self.declare_parameter('turn_deg', 35.0)
        self.declare_parameter('rear_stop_m', 0.45)
        self.declare_parameter('min_hold', 1.0)
        self.declare_parameter('min_run', 3.0)
        self.declare_parameter('pause', 2.0)
        self.declare_parameter('timeout', 0.6)

        self.nav = bool(self.get_parameter('nav_mode').value)
        self.lin_db = float(self.get_parameter('linear_deadband').value)
        self.ang_db = float(self.get_parameter('angular_deadband').value)
        self.max_curv = float(self.get_parameter('max_curvature').value)
        self.allow_reverse = bool(self.get_parameter('allow_reverse').value)
        self.rev_enter = math.radians(float(self.get_parameter('reverse_enter_deg').value))
        self.rev_exit = math.radians(float(self.get_parameter('reverse_exit_deg').value))
        self.turn_thr = math.radians(float(self.get_parameter('turn_deg').value))
        self.rear_stop = float(self.get_parameter('rear_stop_m').value)
        self.min_hold = float(self.get_parameter('min_hold').value)
        self.min_run = float(self.get_parameter('min_run').value)
        self.pause = float(self.get_parameter('pause').value)
        self.timeout = float(self.get_parameter('timeout').value)

        self.pub = self.create_publisher(String, 'hexapod/command', 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_twist, 10)

        # for walking backward: Nav2's path, robot pose, what is behind
        self.path = None
        self.reversing = False
        self.rev_turning = False
        self.rear_blocked = False
        if self.allow_reverse:
            self.tf = Buffer()
            TransformListener(self.tf, self)
            self.create_subscription(Path, 'plan', self.on_path, 10)
            self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)

        self.last_letter = 'x'
        self.last_stamp = self.get_clock().now()
        self.candidate = None
        self.candidate_since = None
        self.sent_at = -1e9              # when the current letter was sent

        self.create_timer(0.1, self.check_timeout)
        self.get_logger().info(
            'cmd_vel bridge up' + (' (navigation safety: nav gait, stand before turns)' if self.nav else ''))

    # ------------------------------------------------------------ walking backward
    def on_path(self, msg: Path):
        self.path = msg if msg.poses else None

    def on_scan(self, msg: LaserScan):
        """Anything within rear_stop_m in the 90 deg sector behind the robot?"""
        try:
            t = self.tf.lookup_transform('base_footprint', msg.header.frame_id, Time())
            off = yaw_of(t.transform.rotation)
        except Exception:                                   # noqa: BLE001
            off = 0.0
        blocked = False
        a = msg.angle_min
        for r in msg.ranges:
            if 0.15 < r < self.rear_stop and abs(wrap(a + off)) > math.radians(135):
                blocked = True
                break
            a += msg.angle_increment
        self.rear_blocked = blocked

    def path_angle(self):
        """Angle (robot frame) to a point ~0.3 m further along Nav2's path, or None."""
        if self.path is None:
            return None
        try:
            t = self.tf.lookup_transform(self.path.header.frame_id or 'map', 'base_footprint', Time())
        except Exception:                                   # noqa: BLE001
            return None
        rx, ry = t.transform.translation.x, t.transform.translation.y
        ryaw = yaw_of(t.transform.rotation)
        pts = [(ps.pose.position.x, ps.pose.position.y) for ps in self.path.poses]
        near = min(range(len(pts)), key=lambda i: (pts[i][0] - rx) ** 2 + (pts[i][1] - ry) ** 2)
        tx, ty = pts[-1]
        for x, y in pts[near:]:
            if math.hypot(x - rx, y - ry) >= 0.3:
                tx, ty = x, y
                break
        if math.hypot(tx - rx, ty - ry) < 0.05:
            return None
        return wrap(math.atan2(ty - ry, tx - rx) - ryaw)

    def reverse_letter(self):
        """'s' / 'a' / 'd' / 'x' while the path is behind, None to follow Nav2 normally."""
        ang = self.path_angle()
        if ang is None:
            self.reversing = False
            return None
        if not self.reversing and abs(ang) > self.rev_enter:
            self.reversing = True
            self.get_logger().info('path is behind: walking backward')
        elif self.reversing and abs(ang) < self.rev_exit:
            self.reversing = False
            self.get_logger().info('path is in front again: walking forward')
        if not self.reversing:
            return None
        err = wrap(ang - math.pi)                 # how far the REAR is off the path
        if abs(err) > (math.radians(9) if self.rev_turning else self.turn_thr):
            self.rev_turning = True               # turn until the rear is within ~9 deg
            return 'a' if err > 0.0 else 'd'
        self.rev_turning = False
        if self.rear_blocked:
            return 'x'                            # something close behind: wait
        return 's'

    # ------------------------------------------------------------ Twist -> letter
    def letter_for(self, twist: Twist) -> str:
        v, w = twist.linear.x, twist.angular.z
        moving = abs(v) > self.lin_db
        turning = abs(w) > self.ang_db
        if not moving and not turning:
            self.reversing = False                # Nav2 stopped (goal reached / blocked)
            return 'x'
        if self.allow_reverse:
            r = self.reverse_letter()
            if r is not None:
                return r
        # sharp curve (or pure rotation): turn in place, the gait can't walk an arc
        if turning and (not moving or abs(w) > self.max_curv * abs(v)):
            return 'a' if w > 0.0 else 'd'
        if moving:
            return 'w' if v > 0.0 else 's'
        return 'x'

    def now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def emit(self, letter: str):
        if letter == self.last_letter:
            return
        self.last_letter = letter
        self.sent_at = self.now()
        self.candidate = None
        text = letter if (letter == 'x' or not self.nav) else 'n' + letter
        self.pub.publish(String(data=text))
        self.get_logger().info(f'cmd_vel -> {text}')

    def request(self, letter: str):
        """Stop at once; a new direction only after it has been steady for min_hold."""
        if letter == 'x' or letter == self.last_letter:
            self.candidate = None
            self.emit(letter)
            return
        if letter != self.candidate:
            self.candidate, self.candidate_since = letter, self.now()
            return
        since_sent = self.now() - self.sent_at
        if self.now() - self.candidate_since < self.min_hold:
            return                                   # not steady yet
        if self.last_letter == 'x':
            if since_sent < self.pause:
                return                               # still resting after a stop
        elif since_sent < self.min_run:
            return                                   # keep the current direction a bit longer
        self.emit(letter)

    def on_twist(self, msg: Twist):
        self.last_stamp = self.get_clock().now()
        self.request(self.letter_for(msg))

    def check_timeout(self):
        if self.last_letter == 'x' and self.candidate is None:
            return
        age = (self.get_clock().now() - self.last_stamp).nanoseconds / 1e9
        if age > self.timeout:
            self.get_logger().warn('cmd_vel timeout, stopping')
            self.candidate = None
            self.emit('x')


def main():
    rclpy.init()
    node = CmdVelBridge()
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

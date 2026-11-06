#!/usr/bin/env python3
"""
sim_hexapod.py

A stand-in for the real robot: runs the same gait code as
Hexapod_esp_v1.ino and publishes the joint angles, so the RViz model
walks without any hardware.

    ros2 launch hexapod_bringup sim.launch.py     # RViz + this node
    ros2 run hexapod_bringup console              # type w, x, g, t, ...

With gazebo:=true (used by hexapod_gazebo) it sends the angles to the
Gazebo servo controllers on /position_controller/commands instead, and
Gazebo's physics decides how the robot actually moves.

Talks on the same topics as the real bridge, so the console (and
cmd_vel_bridge) work unchanged:

    subscribes:  /hexapod/command   std_msgs/String
    publishes:   /hexapod/feedback  std_msgs/String
    publishes:   /hexapod/link      std_msgs/Bool (latched, always true)
    publishes:   /joint_states      sensor_msgs/JointState

Only the legs and gripper move; the body stays in place (no odometry).

Navigation safety mode (nw / ns / na / nd from cmd_vel_bridge), same as the
firmware: the navigation gait (`navgait wave` default, or `navgait tripod`),
a full stand before every change of direction, and a rest stand every
`navtsteps` tripod cycles (3) / `navsteps` wave cycles (0 = off).
Also fdrift / bdrift straight-line correction, like the firmware.
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String

NUM_LEGS, COXA, FEMUR, TIBIA = 6, 0, 1, 2
NEUTRAL, ANGLE_MIN, ANGLE_MAX, INTERP_MS = 90, 25, 155, 20
GRIPPER_OPEN, GRIPPER_CLOSE = 150, 100

# (board, channel, dir, trim) per joint, as in the firmware LEGS table
LEGS = [
    [(0, 2, +1, 0), (0, 0, +1, 0), (0, 1, +1, -8)],
    [(0, 5, +1, 0), (0, 3, +1, 0), (0, 4, +1, 0)],
    [(0, 8, +1, 0), (0, 6, +1, 0), (0, 7, +1, 0)],
    [(1, 2, -1, 0), (1, 0, -1, 0), (1, 1, -1, 0)],
    [(1, 5, -1, 0), (1, 3, -1, 0), (1, 4, -1, 0)],
    [(1, 8, -1, 0), (1, 6, -1, 0), (1, 7, -1, 0)],
]
LEG_NAMES = ['0 front-R', '1 middle-R', '2 back-R', '3 front-L', '4 middle-L', '5 back-L']
TRIPOD_A, TRIPOD_B = (0, 4, 2), (3, 1, 5)
WAVE_ORDER, WAVE_ORDER_BACK = (2, 1, 0, 5, 4, 3), (0, 1, 2, 3, 4, 5)
FORWARD, BACKWARD, TURN_LEFT, TURN_RIGHT = range(4)
MODE_NAMES = ['forward', 'backward', 'turn left', 'turn right']
TRIPOD, WAVE = 0, 1
GAIT_NAMES = ['tripod', 'wave']
MODE_KEYS = {'w': FORWARD, 's': BACKWARD, 'a': TURN_LEFT, 'd': TURN_RIGHT}
JOINT_NAMES = [f'leg{n}_{j}_joint' for n in range(6) for j in ('coxa', 'femur', 'tibia')]


def clamp(a):
    return max(ANGLE_MIN, min(ANGLE_MAX, a))


class HexapodSim:
    """Python port of the firmware gait. Calls out(text) and show(angles, grip)."""

    def __init__(self, out, show, speedup=1.0, sleeper=time.sleep):
        self.out, self.show_cb, self.speedup = out, show, speedup
        self.sleeper = sleeper          # Gazebo passes a sim-time sleep
        self.params = dict(swing=12, turn=16, lift=20, tuck=12, stance=0, speed=200,
                           wlift=4, wpause=200, wswing=8, wturn=12,
                           navsteps=0, navtsteps=3, navhold=1000, fdrift=0, bdrift=0)
        self.nav_gait = WAVE
        self.current = [[NEUTRAL] * 3 for _ in range(NUM_LEGS)]
        self.target = [[NEUTRAL] * 3 for _ in range(NUM_LEGS)]
        self.shown = [[NEUTRAL] * 3 for _ in range(NUM_LEGS)]
        self.leg_swing = [0] * NUM_LEGS
        self.leg_up = [False] * NUM_LEGS
        self.wave_primed, self.wave_primed_mode, self.wave_step = False, -1, 0
        self.mode, self.gait = FORWARD, TRIPOD
        self.walking, self.pending = False, None
        self.nav_mode, self.nav_next, self.nav_cycles = False, FORWARD, 0
        self.grip_angle, self.grip_holding = GRIPPER_OPEN, False
        self.dirs = [[j[2] for j in leg] for leg in LEGS]
        self.lock = threading.Lock()
        self.inbox = []
        self.alive = True

    # ---------------------------------------------------------------- io
    def push_line(self, line):
        with self.lock:
            self.inbox.append(line)

    def poll(self):
        with self.lock:
            lines, self.inbox = self.inbox, []
        for ln in lines:
            self.handle_line(ln)

    def sleep_ms(self, ms):
        self.sleeper(ms / 1000.0 / self.speedup)

    def show(self):
        self.show_cb([a for leg in self.shown for a in leg], self.grip_angle)

    # ---------------------------------------------------------------- low level
    def write_joint(self, leg, joint, logical):
        _, _, _, trim = LEGS[leg][joint]
        d = self.dirs[leg][joint]
        physical = clamp(NEUTRAL + d * (logical - NEUTRAL) + trim)
        self.shown[leg][joint] = NEUTRAL + d * (physical - trim - NEUTRAL)

    def move_to_target(self, duration):
        steps = max(1, duration // INTERP_MS)
        start = [row[:] for row in self.current]
        for s in range(1, steps + 1):
            t = s / steps
            for l in range(NUM_LEGS):
                for j in range(3):
                    self.write_joint(l, j, start[l][j] + int((self.target[l][j] - start[l][j]) * t))
            self.show()
            self.sleep_ms(INTERP_MS)
            self.poll()
        self.current = [row[:] for row in self.target]

    def pause_ms(self, ms):
        t = 0
        while t < ms:
            self.sleep_ms(INTERP_MS)
            self.poll()
            t += INTERP_MS

    def apply_all(self):
        p = self.params
        lift = p['lift'] + (p['wlift'] if self.gait == WAVE else 0)
        for l in range(NUM_LEGS):
            self.target[l][COXA] = NEUTRAL + self.leg_swing[l]
            self.target[l][FEMUR] = NEUTRAL + p['stance'] + (lift if self.leg_up[l] else 0)
            self.target[l][TIBIA] = NEUTRAL + (p['tuck'] if self.leg_up[l] else 0)

    def reset_leg_state(self):
        self.leg_swing = [0] * NUM_LEGS
        self.leg_up = [False] * NUM_LEGS
        self.wave_primed = False

    def push_offset(self, leg):
        p = self.params
        sw = p['wswing'] if self.gait == WAVE else p['swing']
        tw = p['wturn'] if self.gait == WAVE else p['turn']
        right = leg < 3
        fd, bd = p['fdrift'], p['bdrift']          # straight-line correction
        return {FORWARD: -sw + (fd if right else -fd), BACKWARD: sw + (-bd if right else bd),
                TURN_RIGHT: tw if right else -tw, TURN_LEFT: -tw if right else tw}[self.mode]

    # ---------------------------------------------------------------- poses
    def pose_center(self):
        self.reset_leg_state()
        self.target = [[NEUTRAL] * 3 for _ in range(NUM_LEGS)]
        self.move_to_target(800)
        self.out('centered')

    def pose_stance(self):
        self.reset_leg_state()
        self.apply_all()
        self.move_to_target(600)
        self.out('stance')

    # ---------------------------------------------------------------- tripod
    def lift_group(self, g, up=True):
        for l in g:
            self.leg_up[l] = up
        self.apply_all()
        self.move_to_target(self.params['speed'])

    def stride(self, swing_g, push_g):
        for s, p in zip(swing_g, push_g):
            self.leg_swing[s] = -self.push_offset(s)
            self.leg_swing[p] = self.push_offset(p)
        self.apply_all()
        self.move_to_target(self.params['speed'] * 2)

    def tripod_cycle(self):
        self.wave_primed = False
        self.lift_group(TRIPOD_A)
        self.stride(TRIPOD_A, TRIPOD_B)
        self.lift_group(TRIPOD_A, False)
        self.lift_group(TRIPOD_B)
        self.stride(TRIPOD_B, TRIPOD_A)
        self.lift_group(TRIPOD_B, False)
        return True

    def settle_group(self, g):
        if not any(self.leg_swing[l] for l in g):
            return
        self.wave_primed = False
        self.lift_group(g)
        for l in g:
            self.leg_swing[l] = 0
        self.apply_all()
        self.move_to_target(self.params['speed'])
        self.lift_group(g, False)

    # ---------------------------------------------------------------- wave
    def wave_leg(self, i):
        return (WAVE_ORDER_BACK if self.wave_primed_mode == BACKWARD else WAVE_ORDER)[i]

    def wave_offset(self, leg, remaining):
        P = self.push_offset(leg)
        return P - int(2 * P * remaining / 5)        # C integer division truncates toward 0

    def reposition(self, l, swing):
        self.leg_up[l] = True
        self.apply_all()
        self.move_to_target(self.params['speed'])
        self.leg_swing[l] = swing
        self.apply_all()
        self.move_to_target(self.params['speed'])
        self.leg_up[l] = False
        self.apply_all()
        self.move_to_target(self.params['speed'])
        self.pause_ms(self.params['wpause'])

    def wave_prime(self):
        self.wave_primed, self.wave_primed_mode, self.wave_step = False, self.mode, 0
        for i in range(NUM_LEGS):
            if self.pending or self.mode != self.wave_primed_mode:
                return
            l = self.wave_leg(i)
            want = self.wave_offset(l, i)
            if self.leg_swing[l] != want or self.leg_up[l]:
                self.reposition(l, want)
        self.wave_primed = True

    def wave_phase(self):
        sl = self.wave_leg(self.wave_step)
        self.leg_up[sl] = True
        self.apply_all()
        self.move_to_target(self.params['speed'])
        for i in range(NUM_LEGS):
            self.leg_swing[self.wave_leg(i)] = self.wave_offset(
                self.wave_leg(i), (i - self.wave_step - 1) % NUM_LEGS)
        self.apply_all()
        self.move_to_target(self.params['speed'] * 2)
        self.leg_up[sl] = False
        self.apply_all()
        self.move_to_target(self.params['speed'])
        self.pause_ms(self.params['wpause'])
        self.wave_step = (self.wave_step + 1) % NUM_LEGS

    def wave_cycle(self):
        """True only if all six legs stepped."""
        if not self.wave_primed or self.wave_primed_mode != self.mode:
            self.wave_prime()
            if not self.wave_primed:
                return False
        for _ in range(NUM_LEGS):
            if self.pending or self.mode != self.wave_primed_mode:
                return False
            self.wave_phase()
        return True

    def settle_wave(self):
        for i in range(NUM_LEGS):
            l = self.wave_leg(i)
            if self.leg_swing[l] or self.leg_up[l]:
                self.reposition(l, 0)
        self.wave_primed = False

    # ---------------------------------------------------------------- top level
    def walk_cycle(self):
        return self.wave_cycle() if self.gait == WAVE else self.tripod_cycle()

    # ---------------------------------------------------------------- navigation safety
    def nav_command(self, m):
        was_nav = self.nav_mode
        self.nav_mode = True
        # same direction and gait: carry on (the gait is switched during the stand)
        if (was_nav and self.walking and m == self.mode and self.gait == self.nav_gait
                and not self.pending):
            return
        self.nav_next, self.walking, self.pending = m, True, 'S'
        self.out(f'nav: stand, then {MODE_NAMES[m]}')

    def nav_stand(self):
        self.settle_wave()
        self.settle_group(TRIPOD_A)
        self.settle_group(TRIPOD_B)
        self.pose_stance()
        self.pause_ms(self.params['navhold'])

    def set_gait(self, g):
        if g == self.gait:
            self.out(f'gait already {GAIT_NAMES[g]}')
            return
        self.gait, self.wave_primed = g, False
        if g == TRIPOD and any(self.leg_swing) and not self.pending:
            self.pending = 'n'
        self.out(f'gait: {GAIT_NAMES[g]}')

    def test_legs(self):
        self.pose_stance()
        for l in range(NUM_LEGS):
            self.out(f'  lifting leg {LEG_NAMES[l]}')
            self.leg_up[l] = True
            self.apply_all()
            self.move_to_target(400)
            self.sleep_ms(600)
            self.leg_up[l] = False
            self.apply_all()
            self.move_to_target(400)
            self.sleep_ms(400)
        self.out('leg test done')

    def grip(self, close):
        self.grip_angle = GRIPPER_CLOSE if close else GRIPPER_OPEN
        self.grip_holding = close
        self.out('gripper: grabbed' if close else 'gripper: released')
        self.show()

    def print_pose(self):
        self.out('  leg           coxa  femur  tibia')
        for l in range(NUM_LEGS):
            self.out(f'  {LEG_NAMES[l]:<12}' + ''.join(f'{a:>6}' for a in self.current[l]))
        self.out(f"  direction: {MODE_NAMES[self.mode]}  ({'walking' if self.walking else 'stopped'})")
        self.out(f'  gait:      {GAIT_NAMES[self.gait]}')
        self.out(f"  gripper:   {'closed' if self.grip_holding else 'open'} ({self.grip_angle} deg)")
        self.out(f"  nav mode:  {'on' if self.nav_mode else 'off'}   nav gait: {GAIT_NAMES[self.nav_gait]}")

    def print_params(self):
        self.out('  '.join(f'{k}={v}' for k, v in self.params.items()))

    def handle_line(self, line):
        line = line.strip()
        low = line.lower()
        if not line:
            return
        if len(line) == 1:
            c = low
            if c in MODE_KEYS:
                self.nav_mode = False                   # manual driving
                self.mode, self.walking, self.pending = MODE_KEYS[c], True, None
                self.out(f'walking {MODE_NAMES[self.mode]} ({GAIT_NAMES[self.gait]})')
                return
            if c == 'g':
                return self.grip(True)
            if c == 'r':
                return self.grip(False)
            if c in 'x1hct':
                self.walking = False
                self.nav_mode = False
                self.pending = c
                if c == 'x':
                    self.out('stopping')
                return
            if c == 'p':
                return self.print_pose()
            if c == '?':
                self.out('  simulator: w s a d x  g r grip  1 [dir]  h c p t  tripod wave')
                self.out('  swing turn lift tuck stance speed wlift wpause wswing wturn <value>')
                self.out('  navgait wave|tripod   navsteps / navtsteps / navhold / fdrift / bdrift <value>')
                return self.print_params()
        if low in ('nw', 'ns', 'na', 'nd'):
            return self.nav_command(MODE_KEYS[low[1]])
        if low in ('grab', 'release', 'grip'):
            return self.grip(low == 'grab' or (low == 'grip' and not self.grip_holding))
        if low in ('tripod', 'wave'):
            return self.set_gait(TRIPOD if low == 'tripod' else WAVE)
        if low in ('dump', 'twin on', 'twin off'):
            return self.out('(not used in the simulator)')
        tok = line.split()
        if len(tok) == 4 and tok[0].lower() == 'dir':
            try:
                leg, d = int(tok[1]), int(tok[3])
                jn = {'coxa': 0, 'c': 0, 'femur': 1, 'f': 1, 'tibia': 2, 't': 2}[tok[2].lower()]
                assert 0 <= leg < 6 and d in (1, -1)
            except (ValueError, KeyError, AssertionError):
                return self.out('? usage: dir <leg 0-5> <coxa|femur|tibia> <1|-1>')
            self.dirs[leg][jn] = d
            return self.out(f'leg {LEG_NAMES[leg]}  {tok[2]} dir = {d}')
        if len(tok) == 2:
            k = tok[0].lower()
            if k == 'navgait':
                g = {'tripod': TRIPOD, 'wave': WAVE}.get(tok[1].lower())
                if g is None:
                    return self.out('? navgait must be tripod or wave')
                self.nav_gait = g
                self.out(f'nav gait: {GAIT_NAMES[g]}')
                if self.nav_mode and self.walking and self.gait != g and not self.pending:
                    self.nav_next, self.pending = self.mode, 'S'   # stand, switch, carry on
                return
            if k == 'gait' and tok[1].lower() in ('tripod', 'wave'):
                return self.set_gait(TRIPOD if tok[1].lower() == 'tripod' else WAVE)
            if k == '1' and tok[1].lower() in MODE_KEYS:
                self.mode, self.walking, self.pending = MODE_KEYS[tok[1].lower()], False, '1'
                return
            if k in self.params:
                try:
                    v = int(tok[1])
                except ValueError:
                    return self.out('? value must be a number')
                if k == 'speed':
                    v = max(40, v)
                if k in ('navsteps', 'navtsteps'):
                    v = max(0, v)
                if k in ('fdrift', 'bdrift'):
                    v = max(-10, min(10, v))
                    self.wave_primed = False
                if k == 'navhold':
                    v = max(0, v)
                if k == 'wpause':
                    v = max(0, v)
                if k in ('wswing', 'wturn'):
                    self.wave_primed = False
                self.params[k] = v
                return self.print_params()
        if len(tok) == 3:
            b = {'40': 0, '0x40': 0, '0': 0, '41': 1, '0x41': 1, '1': 1}.get(tok[0].lower())
            try:
                ch = int(tok[1])
                val = int(tok[2])
            except ValueError:
                b = None
            if b is not None and 0 <= ch <= 8:
                self.walking = False
                for l in range(NUM_LEGS):
                    for j in range(3):
                        bb, cc, _, trim = LEGS[l][j]
                        if bb == b and cc == ch:
                            d = self.dirs[l][j]
                            old = NEUTRAL + d * (self.shown[l][j] - NEUTRAL) + trim
                            a = clamp(old + val if tok[2][0] in '+-' else val)
                            self.shown[l][j] = NEUTRAL + d * (a - trim - NEUTRAL)
                            self.show()
                            return self.out(f'0x{40 + b} ch{ch} -> {a} deg')
        self.out('? unknown - type ? for help')

    def run_action(self, a):
        if a == 'S':                                # navigation: stand, then go
            self.nav_stand()
            if self.gait != self.nav_gait:          # switch to the nav gait while standing
                self.gait, self.wave_primed = self.nav_gait, False
                self.out(f'nav: {GAIT_NAMES[self.gait]} gait')
            if self.pending:
                return
            self.mode, self.nav_cycles, self.walking = self.nav_next, 0, self.nav_mode
            return
        if a == 'x':
            if self.gait == WAVE:
                self.settle_wave()
            else:
                self.settle_group(TRIPOD_A)
                self.settle_group(TRIPOD_B)
            self.pose_stance()
        elif a == 'h':
            self.pose_stance()
        elif a == 'c':
            self.pose_center()
        elif a == 't':
            self.test_legs()
        elif a == 'n':
            self.settle_group(TRIPOD_A)
            self.settle_group(TRIPOD_B)
            self.out('legs settled for tripod')
        elif a == '1':
            self.out(f'one cycle {MODE_NAMES[self.mode]} ({GAIT_NAMES[self.gait]})')
            self.walk_cycle()
            self.out('cycle done')

    def loop(self):
        self.out('=== hexapod simulator (same gait as the ESP32) ===')
        self.pose_center()
        while self.alive:
            self.poll()
            if self.pending:
                a, self.pending = self.pending, None
                self.run_action(a)
            elif self.walking:
                full = self.walk_cycle()
                every = self.params['navtsteps' if self.gait == TRIPOD else 'navsteps']
                if self.nav_mode and full and every > 0 and not self.pending:
                    self.nav_cycles += 1
                    if self.nav_cycles >= every:
                        self.nav_cycles, self.nav_next, self.pending = 0, self.mode, 'S'
                        self.out('nav: rest stand')
            else:
                self.show()                        # keep the model fresh while idle
                self.sleep_ms(100)


class SimNode(Node):

    def __init__(self):
        super().__init__('hexapod_sim')
        self.declare_parameter('speedup', 1.0)
        self.declare_parameter('gazebo', False)
        self.gazebo = bool(self.get_parameter('gazebo').value)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.fb = self.create_publisher(String, 'hexapod/feedback', 10)
        self.link = self.create_publisher(Bool, 'hexapod/link', latched)
        if self.gazebo:
            self.cmd = self.create_publisher(Float64MultiArray, 'position_controller/commands', 10)
        else:
            self.js = self.create_publisher(JointState, 'joint_states', 10)
        self.create_subscription(String, 'hexapod/command', self.on_command, 10)
        self.link.publish(Bool(data=True))

        self.sim = HexapodSim(self.out, self.publish, float(self.get_parameter('speedup').value),
                              sleeper=self.sim_sleep if self.gazebo else time.sleep)
        self.worker = threading.Thread(target=self.sim.loop, daemon=True)
        self.worker.start()
        self.get_logger().info('simulator up - run: ros2 run hexapod_bringup console')

    def sim_sleep(self, seconds):
        """Wait in Gazebo time, so the gait keeps pace even when Gazebo runs
        slower than real time (otherwise steps overlap and legs drag)."""
        clock = self.get_clock()
        start = clock.now().nanoseconds
        if start == 0:                      # no /clock yet
            time.sleep(seconds)
            return
        end = start + int(seconds * 1e9)
        deadline = time.monotonic() + 10 * seconds + 1.0   # never hang if Gazebo pauses
        while clock.now().nanoseconds < end and time.monotonic() < deadline and rclpy.ok():
            time.sleep(0.002)

    def out(self, text):
        if rclpy.ok():
            self.fb.publish(String(data=text))
            self.get_logger().info(f'<- {text}')

    def publish(self, angles, grip):
        if not rclpy.ok():
            return
        jaw = math.radians((GRIPPER_OPEN - grip) / 2.0)
        position = [math.radians(a - NEUTRAL) for a in angles] + [jaw]
        if self.gazebo:
            self.cmd.publish(Float64MultiArray(data=position))
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES + ['gripper_joint']
        msg.position = position
        self.js.publish(msg)

    def on_command(self, msg):
        if len(msg.data.strip()) <= 48:
            self.get_logger().info(f'-> {msg.data.strip()}')
            self.sim.push_line(msg.data)


def main():
    rclpy.init()
    node = SimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sim.alive = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

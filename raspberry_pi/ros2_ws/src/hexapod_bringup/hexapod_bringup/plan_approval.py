#!/usr/bin/env python3
"""
plan_approval.py

Nav2 goal -> preview -> your OK -> go.

1. You click "2D Goal Pose" (topic /goal_pose) or "Nav2 Goal" in RViz.
2. This node asks Nav2's planner for the path, but does NOT move the robot.
3. It shows the plan:
     - in RViz: topic /hexapod/preview_path  (add a "Path" display)
     - in the console (hexcon): the steps, e.g.
           1. turn left 40 deg      (~3 turn cycles)
           2. walk forward 1.20 m   (~29 wave cycles)
4. Type `go` in hexcon to start, `cancel` to drop the plan (or stop a running one).

bt_navigator itself does not listen to /goal_pose any more and its action is
renamed to navigate_to_pose_approved (nav.launch.py remaps both). This node
serves navigate_to_pose instead, so RViz's "Nav2 Goal" also waits for `go`.

The step counts are estimates from the leg geometry; calibrate them with the
*_step_m / *_turn_deg parameters (walk 10 cycles, measure, divide by 10).

Parameters:
    goal_topic       topic RViz publishes goals on (/goal_pose)
    auto_go          false = wait for `go` (default); true = start at once
    gait             wave / tripod - which numbers to use; follows `navgait` typed in hexcon
    wave_step_m      body travel per wave cycle    (0.042 m)
    tripod_step_m    body travel per tripod cycle  (0.062 m)
    wave_turn_deg    body turn per wave cycle      (14 deg)
    tripod_turn_deg  body turn per tripod cycle    (19 deg)
    wave_cycle_s     duration of one wave cycle    (6.0 s)
    tripod_cycle_s   duration of one tripod cycle  (1.6 s)
    stand_s          one stand (before a turn etc.) (5.0 s)
    min_turn_deg     smaller heading changes are not listed as turns (20)
    simplify_m       how far the path may bend before it counts as a corner (0.08 m)
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def simplify(points, tol):
    """Ramer-Douglas-Peucker: keep only the corners of the path."""
    if len(points) < 3:
        return points
    (x0, y0), (x1, y1) = points[0], points[-1]
    dx, dy = x1 - x0, y1 - y0
    norm = math.hypot(dx, dy) or 1e-9
    best, idx = 0.0, 0
    for i, (x, y) in enumerate(points[1:-1], 1):
        d = abs(dy * x - dx * y + x1 * y0 - y1 * x0) / norm
        if d > best:
            best, idx = d, i
    if best <= tol:
        return [points[0], points[-1]]
    return simplify(points[:idx + 1], tol)[:-1] + simplify(points[idx:], tol)


class PlanApproval(Node):

    def __init__(self):
        super().__init__('plan_approval')
        p = self.declare_parameter
        p('goal_topic', '/goal_pose')
        p('auto_go', False)
        p('gait', 'wave')
        p('wave_step_m', 0.042)
        p('tripod_step_m', 0.062)
        p('wave_turn_deg', 14.0)
        p('tripod_turn_deg', 19.0)
        p('wave_cycle_s', 6.0)
        p('tripod_cycle_s', 1.6)
        p('stand_s', 5.0)
        p('min_turn_deg', 20.0)
        p('simplify_m', 0.08)
        self.gait = str(self.get_parameter('gait').value).lower()

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.say_pub = self.create_publisher(String, 'hexapod/feedback', 10)
        self.path_pub = self.create_publisher(Path, 'hexapod/preview_path', latched)
        self.create_subscription(PoseStamped, self.get_parameter('goal_topic').value,
                                 self.on_goal, 10)
        self.create_subscription(String, 'hexapod/approve', self.on_approve, 10)
        self.create_subscription(String, 'hexapod/command', self.on_command, 10)

        cb = ReentrantCallbackGroup()
        self.planner = ActionClient(self, ComputePathToPose, 'compute_path_to_pose',
                                    callback_group=cb)
        # the real navigator (renamed in nav.launch.py) ...
        self.navigator = ActionClient(self, NavigateToPose, 'navigate_to_pose_approved',
                                      callback_group=cb)
        # ... and the name RViz's "Nav2 Goal" button calls: goes through approval too
        ActionServer(self, NavigateToPose, 'navigate_to_pose',
                     execute_callback=self.exec_rviz_goal,
                     goal_callback=lambda _req: GoalResponse.ACCEPT,
                     cancel_callback=lambda _gh: CancelResponse.ACCEPT,
                     callback_group=cb)
        self.tf = Buffer()
        TransformListener(self.tf, self)

        self.pending = None          # goal waiting for `go`
        self.active = None           # running NavigateToPose goal handle
        self.job = 0                 # increases with every new goal
        self.running_job = None      # job the active navigation belongs to
        self.done = {}               # job -> final status
        self.planning_job = None     # job whose path is being computed
        self.last_fb = 0.0
        self.get_logger().info('plan approval up: click a goal, then type go / cancel in hexcon')

    # ------------------------------------------------------------------ helpers
    def say(self, text):
        for line in text.splitlines():
            self.say_pub.publish(String(data=line))
        self.get_logger().info(text)

    def val(self, name):
        return self.get_parameter(name).value

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    # ------------------------------------------------------------------ inputs
    def on_command(self, msg):
        tok = msg.data.strip().lower().split()
        if len(tok) == 2 and tok[0] == 'navgait' and tok[1] in ('wave', 'tripod'):
            self.gait = tok[1]

    def on_goal(self, goal: PoseStamped):
        if self.active is not None:
            self.say('plan: robot is already navigating - type cancel first')
            return
        if not self.planner.wait_for_server(timeout_sec=2.0):
            self.say('plan: Nav2 planner not ready yet, try again in a moment')
            return
        self.job += 1
        self.planning_job = self.job
        self.say('plan: planning...')
        req = ComputePathToPose.Goal()
        req.goal = goal
        req.use_start = False
        self.planner.send_goal_async(req).add_done_callback(
            lambda f, g=goal: self.on_plan_accepted(f, g))

    def on_plan_accepted(self, future, goal):
        handle = future.result()
        if not handle.accepted:
            self.planning_job = None
            self.say('plan: planner refused the goal')
            return
        handle.get_result_async().add_done_callback(lambda f, g=goal: self.on_plan(f, g))

    def on_plan(self, future, goal):
        path = future.result().result.path
        self.planning_job = None
        if not path.poses:
            self.pending = None
            self.say('plan: no path found (goal blocked or outside the map) - pick another spot')
            return
        self.pending = goal
        self.show_plan(path)
        if self.val('auto_go'):
            self.start()

    def on_approve(self, msg):
        word = msg.data.strip().lower()
        if word == 'go':
            if self.active is not None:
                self.say('plan: already going')
            elif self.pending is None:
                self.say('plan: nothing to approve - click a 2D Goal Pose first')
            else:
                self.start()
        elif word == 'cancel':
            if self.active is not None:
                self.active.cancel_goal_async()
                self.say('plan: stopping navigation')
            elif self.pending is not None:
                self.say('plan: dropped')
            else:
                self.say('plan: nothing to cancel')
            self.pending = None
            self.path_pub.publish(Path(header=self.empty_header()))

    def empty_header(self):
        h = Path().header
        h.frame_id = 'map'
        h.stamp = self.get_clock().now().to_msg()
        return h

    # ------------------------------------------------------------------ plan text
    def robot_pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)
        except Exception:                                     # noqa: BLE001
            return None

    def show_plan(self, path: Path):
        pts = [(ps.pose.position.x, ps.pose.position.y) for ps in path.poses]
        corners = simplify(pts, float(self.val('simplify_m')))

        # simplified path for RViz
        prev = Path(header=path.header)
        for x, y in corners:
            ps = PoseStamped(header=path.header)
            ps.pose.position.x, ps.pose.position.y = x, y
            ps.pose.orientation.w = 1.0
            prev.poses.append(ps)
        self.path_pub.publish(prev)

        rp = self.robot_pose()
        heading = rp[2] if rp else math.atan2(corners[-1][1] - corners[0][1],
                                              corners[-1][0] - corners[0][0])
        tripod = self.gait == 'tripod'
        step = float(self.val('tripod_step_m' if tripod else 'wave_step_m'))
        turn = float(self.val('tripod_turn_deg' if tripod else 'wave_turn_deg'))
        cyc_s = float(self.val('tripod_cycle_s' if tripod else 'wave_cycle_s'))
        stand_s = float(self.val('stand_s'))
        min_turn = math.radians(float(self.val('min_turn_deg')))

        # same rule as cmd_vel_bridge: a path behind the robot is walked backward
        enter, leave = math.radians(100.0), math.radians(70.0)
        backward = False
        steps = []                       # ('turn', deg) / ('walk', m) / ('back', m)
        for (x0, y0), (x1, y1) in zip(corners, corners[1:]):
            length = math.hypot(x1 - x0, y1 - y0)
            if length < 0.05:
                continue
            seg = math.atan2(y1 - y0, x1 - x0)
            d = wrap(seg - heading)
            backward = abs(d) > (leave if backward else enter)
            if backward:
                d = wrap(seg - (heading + math.pi))      # turn so the REAR faces the path
            if abs(d) > min_turn:
                steps.append(('turn', math.degrees(d)))
                heading = wrap(heading + d)
            kind = 'back' if backward else 'walk'
            if steps and steps[-1][0] == kind:
                steps[-1] = (kind, steps[-1][1] + length)
            else:
                steps.append((kind, length))

        total = sum(v for k, v in steps if k != 'turn')
        lines, secs, n = [], 0.0, 0
        for kind, v in steps:
            n += 1
            if kind == 'turn':
                c = max(1, math.ceil(abs(v) / turn))
                lines.append(f'  {n}. turn {"left" if v > 0 else "right"} {abs(v):.0f} deg'
                             f'   (~{c} turn cycles)')
                secs += stand_s + c * cyc_s + stand_s
            else:
                c = max(1, math.ceil(v / step))
                way = 'forward' if kind == 'walk' else 'BACKWARD'
                lines.append(f'  {n}. walk {way} {v:.2f} m   (~{c} {self.gait} cycles)')
                secs += c * cyc_s
                if tripod:
                    secs += (c // 3) * stand_s          # rest stand every 3 tripod cycles
        turns = sum(1 for k, _ in steps if k == 'turn')
        head = (f'PLAN ({self.gait}): {total:.2f} m, {turns} turn(s), about '
                f'{max(1, round(secs / 60))} min - stops within 30 cm of the goal')
        self.say('\n'.join([head] + lines + ['  type  go  to start,  cancel  to drop it']))

    # ------------------------------------------------------------------ execute
    def start(self):
        if not self.navigator.wait_for_server(timeout_sec=2.0):
            self.say('plan: Nav2 navigator not ready yet, type go again in a moment')
            return
        req = NavigateToPose.Goal()
        req.pose = self.pending
        self.running_job = self.job
        self.say('plan: GO')
        self.navigator.send_goal_async(req, feedback_callback=self.on_nav_feedback) \
            .add_done_callback(self.on_nav_accepted)

    def on_nav_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.done[self.running_job] = 6
            self.say('plan: Nav2 refused the goal')
            return
        self.active = handle
        handle.get_result_async().add_done_callback(self.on_nav_done)

    def on_nav_feedback(self, msg):
        now = self.now_s()
        if now - self.last_fb > 20.0:                          # not too chatty
            self.last_fb = now
            self.say(f'plan: {msg.feedback.distance_remaining:.2f} m to go')

    def on_nav_done(self, future):
        status = future.result().status
        self.done[self.running_job] = status
        self.active = None
        self.pending = None
        self.path_pub.publish(Path(header=self.empty_header()))
        text = {4: 'goal reached', 5: 'cancelled', 6: 'failed (stuck or blocked)'}.get(
            status, f'ended (status {status})')
        self.say(f'plan: {text}')


    # ------------------------------------------------------------------ RViz "Nav2 Goal"
    def exec_rviz_goal(self, gh):
        """A goal from RViz's Nav2 Goal button: plan, wait for `go`, then report back."""
        if self.active is not None:
            self.say('plan: robot is already navigating - type cancel first')
            gh.abort()
            return NavigateToPose.Result()
        self.on_goal(gh.request.pose)
        mine = self.job
        while rclpy.ok():
            time.sleep(0.2)
            if gh.is_cancel_requested:
                if self.running_job == mine or self.pending is not None:
                    self.on_approve(String(data='cancel'))
                gh.canceled()
                return NavigateToPose.Result()
            if mine in self.done:
                status = self.done.pop(mine)
                (gh.succeed if status == 4 else gh.canceled if status == 5 else gh.abort)()
                return NavigateToPose.Result()
            replaced = self.job != mine and self.running_job != mine
            dropped = (self.pending is None and self.running_job != mine
                       and self.planning_job != mine)
            if replaced or dropped:
                gh.abort()                      # new goal clicked, or `cancel` before `go`
                return NavigateToPose.Result()
        gh.abort()
        return NavigateToPose.Result()


def main():
    rclpy.init()
    node = PlanApproval()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

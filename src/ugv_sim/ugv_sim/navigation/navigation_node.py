"""
MEMBER 4 navigation node: Hybrid A* global planner + DWA local planner.

Built on top of the existing Member 1 interfaces (/cmd_vel, /odom,
/lidar/points) without modifying them.

    Sensors (/lidar/points, /odom)
      -> Costmap2D (obstacle representation)
      -> Hybrid A* (global path, replanned periodically / on blockage)
      -> DWA (local trajectory, ~10 Hz, collision-checked against the
         same costmap, scored against the global path)
      -> /cmd_vel
      -> UGV

Run interactively (needs a real keyboard-attached terminal, same pattern
as the retired goal_navigator.py):

    ros2 run ugv_sim navigation_node

State machine: IDLE -> PLANNING -> NAVIGATING -> (REPLANNING | RECOVERY |
ROCKING)* -> GOAL_REACHED | FAILED. No human /cmd_vel input is used once
navigation starts -- the human only presses the start key.
"""
import math
import os
import sys
import termios
import time
import tty

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from . import dwa
from . import hybrid_astar
from .collision_checker import is_state_valid
from .costmap import Costmap2D
from .goal_manager import GoalManager
from .recovery import RecoveryManager

CONTROL_HZ = 10.0
REPLAN_PERIOD = 6.0          # s, periodic full replan while navigating
PATH_CHECK_PERIOD = 0.5      # s, how often the remaining path is re-validated
# Minimum real gap between two ACTUAL replans, regardless of how often
# _path_blocked() returns True. Every replan starts from the vehicle's
# *current* heading, and the motion primitives have no sharp in-place
# turn (only wide arcs) -- replanning faster than the vehicle can
# converge onto its current plan's intended heading was measured to
# cascade: each new plan starts from an already-drifted heading, can't
# cheaply correct it, and just continues curving the same way, snowballing
# into continuous rotation with the vehicle never actually going anywhere
# (yaw climbed past 110 degrees, entirely away from the goal bearing,
# while position barely changed, even after a first attempt at rate
# limiting to 1.8s). Pushed much higher because DWA's own per-tick
# collision checking against the live costmap already guarantees safety
# regardless of how stale the global path is -- frequent replanning here
# only ever bought path *efficiency*, never safety, so there is no reason
# to replan faster than the vehicle can actually act on a plan.
MIN_REPLAN_INTERVAL = 4.5    # s
PATH_DEVIATION_REPLAN = 1.2  # m
ROTATE_OMEGA = 0.45

# ROCK recovery: alternating forward/reverse pulses, values measured
# directly to produce real progress escaping a Rubicon rock obstacle where
# constant-velocity driving could not (see recovery.py's module docstring).
ROCK_FORWARD_V = 0.3
ROCK_REVERSE_V = -0.25
ROCK_FORWARD_DURATION = 0.8   # s
ROCK_REVERSE_DURATION = 0.5   # s
ROCK_CYCLE = ROCK_FORWARD_DURATION + ROCK_REVERSE_DURATION


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wait_for_keypress(prompt):
    print(prompt, flush=True)
    # UGV_SIM_AUTOSTART=1 skips the interactive wait -- for automated
    # testing only (this repo's own test harnesses), where driving a raw
    # pty through termios.setraw() to simulate one keystroke has proven
    # unreliable after many repeated launches in one session (writes
    # stopped being delivered even after 30 retries, no code-side cause
    # found). Default interactive behavior (a human pressing a real key)
    # is unchanged.
    if os.environ.get('UGV_SIM_AUTOSTART') == '1':
        print('[UGV_SIM_AUTOSTART=1] skipping interactive wait', flush=True)
    else:
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
    print('\nKey pressed — starting autonomous navigation.\n', flush=True)


class NavigationNode(Node):

    def __init__(self):
        super().__init__('navigation_node')
        self.declare_parameter('goal_x', 8.0)
        self.declare_parameter('goal_y', 5.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('goal_pos_tolerance', 0.35)
        self.declare_parameter('goal_yaw_tolerance_deg', 90.0)

        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_yaw = self.get_parameter('goal_yaw').value
        pos_tol = self.get_parameter('goal_pos_tolerance').value
        yaw_tol = math.radians(self.get_parameter('goal_yaw_tolerance_deg').value)

        self.goal_mgr = GoalManager(
            self.goal_x, self.goal_y, self.goal_yaw, pos_tol, yaw_tol)
        self.costmap = Costmap2D()
        self.recovery = RecoveryManager()

        self.pose = None          # (x, y, theta)
        self.v_curr = 0.0
        self.omega_curr = 0.0
        self.recent_omegas = []

        self.global_path = None
        self.path_idx = 0
        self.state = 'IDLE'
        self.replan_count = 0
        self.recovery_count = 0
        self.plan_fail_streak = 0
        self.last_replan_time = 0.0
        self.last_path_check_time = 0.0
        self.last_telemetry_time = 0.0
        self.rock_start_time = 0.0
        self.start_time = None
        self.total_path_length = 0.0

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/planner/global_path', 1)
        self.local_traj_pub = self.create_publisher(Path, '/planner/local_trajectory', 1)

        self.create_subscription(Odometry, '/odom', self._odom_cb, 20)
        self.create_subscription(PointCloud2, '/lidar/points', self._lidar_cb, 5)

    # ---- subscriptions ----------------------------------------------

    def _odom_cb(self, msg):
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     yaw_of(msg.pose.pose.orientation))
        self.v_curr = msg.twist.twist.linear.x
        self.omega_curr = msg.twist.twist.angular.z

    def _lidar_cb(self, msg):
        if self.pose is None:
            return
        if self.state in ('ROCKING', 'RECOVERY'):
            # /odom is pure wheel dead-reckoning with no correction.
            # Measured directly on Rubicon terrain: during a rock/rotate
            # recovery cycle (heavy wheel-slip by design -- that's what
            # rocking IS), /odom diverged from true pose by over 0.35 m in
            # under 15 s, once even reporting forward progress while the
            # vehicle had actually moved backward. Costmap cells painted
            # from a pose that wrong land near the vehicle's own
            # (falsely-reported) position, which then reads as blocked on
            # the next plan -- triggering more recovery, more slip, more
            # drift: a self-reinforcing failure loop, confirmed directly
            # (min_clearance=0.00/feasible=0 events lined up with the
            # worst odom/truth divergence in logged runs). Suspending
            # costmap updates for the duration of a recovery maneuver, when
            # odometry is least trustworthy anyway, breaks that loop; normal
            # updates resume once back in NAVIGATING with a more reliable
            # (if still slowly drifting) pose estimate.
            return
        x, y, theta = self.pose
        now = time.time()
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        for px, py, pz in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=False):
            if not (math.isfinite(px) and math.isfinite(py) and math.isfinite(pz)):
                continue
            r = math.hypot(px, py)
            if r < 0.8 or r > 6.0:
                continue
            height = pz + 0.63  # lidar mount height above base_link/ground
            wx = x + px * cos_t - py * sin_t
            wy = y + px * sin_t + py * cos_t
            if 0.06 <= height <= 0.9:
                # Raised obstacle-height return: occupancy, as before.
                self.costmap.mark_occupied_world(wx, wy, now)
            elif -0.2 <= height < 0.06:
                # Near-ground return: feeds terrain roughness only (not
                # occupancy -- ground itself, even uneven ground, is not
                # an obstacle). Height spread within a cell over several
                # returns is the roughness signal; see costmap.py.
                self.costmap.mark_ground_sample_world(wx, wy, height, now)

    # ---- planning helpers --------------------------------------------

    def _publish_path(self, path):
        msg = Path()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        for (x, y, theta) in path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.z = math.sin(theta / 2.0)
            ps.pose.orientation.w = math.cos(theta / 2.0)
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    def _publish_local_traj(self, traj):
        msg = Path()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        for (x, y, theta) in traj:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.z = math.sin(theta / 2.0)
            ps.pose.orientation.w = math.cos(theta / 2.0)
            msg.poses.append(ps)
        self.local_traj_pub.publish(msg)

    def _path_length(self, path):
        return sum(math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
                   for i in range(len(path) - 1))

    def _plan(self):
        x, y, theta = self.pose
        if not is_state_valid(self.costmap, x, y, theta):
            # The vehicle's own current pose reads as colliding -- a map
            # error (self-hit leakage or /odom-drift-smeared stale cells,
            # see costmap.py), not a real one, since physics would already
            # have stopped it. Clear the immediate surroundings rather
            # than let planning fail against the vehicle's own position
            # forever. Measured directly: without this, the vehicle
            # stalled permanently near an obstacle it was never actually
            # touching (true clearance to the nearest real obstacle was
            # 1.3 m) because stale/self-hit cells had accumulated at its
            # own location.
            # Radius must exceed the footprint's own corner reach
            # (sqrt(0.52^2 + 0.42^2) = 0.67 m) plus the costmap's
            # inflation radius (0.15 m), or a cell just past the clear
            # radius can still inflate back into the footprint -- measured
            # directly: clearing at 0.75 m reported "changed=False" (no
            # cells within 0.75 m at all) while the vehicle's own pose
            # kept reading as colliding anyway, five checks in a row.
            cleared = self.costmap.clear_radius_world(x, y, 1.0)
            self.get_logger().warn(
                f'Own pose read as colliding at ({x:.2f},{y:.2f}) -- '
                f'cleared nearby cells (changed={cleared})'
            )
        t0 = time.time()
        path = hybrid_astar.plan(
            x, y, theta, self.goal_x, self.goal_y, self.goal_yaw,
            self.costmap, goal_pos_tol=self.goal_mgr.pos_tolerance)
        dt = time.time() - t0
        if path is None:
            self.get_logger().warn(f'Hybrid A*: no path found ({dt:.2f}s search)')
            return False
        self.global_path = path
        self.path_idx = 0
        self.total_path_length += self._path_length(path)
        self._publish_path(path)
        self.get_logger().info(
            f'Hybrid A*: path with {len(path)} waypoints, '
            f'length={self._path_length(path):.2f} m, search={dt:.2f}s'
        )
        return True

    def _path_blocked(self, lookahead_waypoints=5):
        # Only check a near-term lookahead window, not the whole
        # remaining path to goal. The costmap is built incrementally as
        # the vehicle explores, so far-future sections of the path are
        # still "unknown" rather than genuinely verified clear -- checking
        # them here just means they keep being re-flagged as new lidar
        # data about them arrives, triggering a replan before the vehicle
        # can ever finish committing to the current one. Measured
        # directly: with the whole path checked, the vehicle replanned 16
        # times in the first 10 seconds and never made real progress.
        if self.global_path is None:
            return True
        end = min(len(self.global_path), self.path_idx + lookahead_waypoints)
        for (x, y, theta) in self.global_path[self.path_idx:end:2]:
            if not is_state_valid(self.costmap, x, y, theta):
                return True
        return False

    # ---- main control loop --------------------------------------------

    def _telemetry(self, extra=''):
        x, y, theta = self.pose
        dist = self.goal_mgr.distance(x, y)
        self.get_logger().info(
            f'[{self.state}] x={x:.2f} y={y:.2f} yaw={math.degrees(theta):.1f} '
            f'dist_to_goal={dist:.2f} v={self.v_curr:.2f} w={self.omega_curr:.2f} '
            f'obstacles={self.costmap.occupied_count()} replans={self.replan_count} '
            f'recoveries={self.recovery_count}{extra}'
        )

    def run(self):
        wait_for_keypress(
            f'Press any key to start autonomous navigation to goal '
            f'({self.goal_x}, {self.goal_y}, yaw={self.goal_yaw})...'
        )
        self.state = 'PLANNING'
        period = 1.0 / CONTROL_HZ
        min_clearance_seen = None

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=period)
            if self.pose is None:
                continue
            now = time.time()
            x, y, theta = self.pose
            self.costmap.decay(now)

            if self.state == 'PLANNING':
                if self._plan():
                    self.plan_fail_streak = 0
                    self.start_time = now
                    self.recovery.reset_progress_clock(now, self.goal_mgr.distance(x, y))
                    self.last_replan_time = now
                    self.state = 'NAVIGATING'
                else:
                    self.cmd_pub.publish(Twist())
                    self.plan_fail_streak += 1
                    if self.plan_fail_streak == 3:
                        nearby = []
                        for gx, gy in np.argwhere(self.costmap.occupied):
                            ox, oy = self.costmap.grid_to_world(int(gx), int(gy))
                            d = math.hypot(ox - x, oy - y)
                            if d < 3.0:
                                nearby.append((round(d, 2), round(ox, 2), round(oy, 2)))
                        nearby.sort()
                        bearing_deg = math.degrees(
                            math.atan2(self.goal_y - y, self.goal_x - x))
                        self.get_logger().warn(
                            f'DIAG stuck at ({x:.2f},{y:.2f}) yaw={math.degrees(theta):.1f} '
                            f'goal_bearing={bearing_deg:.1f} nearby_occupied={nearby[:25]}'
                        )
                    if self.plan_fail_streak >= 5:
                        self.state = 'FAILED'
                        self._telemetry(' NAVIGATION FAILED (no initial path found)')
                    else:
                        time.sleep(0.3)
                continue

            if self.state in ('NAVIGATING',):
                if self.goal_mgr.reached(x, y, theta):
                    self.cmd_pub.publish(Twist())
                    self.state = 'GOAL_REACHED'
                    self._telemetry(' GOAL REACHED')
                    continue

                if (now - self.last_path_check_time > PATH_CHECK_PERIOD
                        and now - self.last_replan_time > MIN_REPLAN_INTERVAL):
                    self.last_path_check_time = now
                    if self._path_blocked() or now - self.last_replan_time > REPLAN_PERIOD:
                        self.state = 'REPLANNING'
                        continue

                dist = self.goal_mgr.distance(x, y)
                rec_status = self.recovery.check(now, dist)
                if rec_status == 'failed':
                    self.cmd_pub.publish(Twist())
                    self.state = 'FAILED'
                    self._telemetry(' NAVIGATION FAILED')
                    continue
                if rec_status in ('rotate', 'rotating'):
                    if rec_status == 'rotate':
                        self.recovery_count += 1
                    cmd = Twist()
                    cmd.angular.z = ROTATE_OMEGA
                    self.cmd_pub.publish(cmd)
                    self.state = 'RECOVERY'
                    continue
                if rec_status in ('rock', 'rocking'):
                    if rec_status == 'rock':
                        self.recovery_count += 1
                        self.rock_start_time = now
                    self.state = 'ROCKING'
                    continue
                if rec_status == 'stall_replan':
                    self.state = 'REPLANNING'
                    continue

                v, omega, self.path_idx, min_clear, feasible = dwa.compute_command(
                    (x, y, theta), self.v_curr, self.omega_curr,
                    self.global_path, self.path_idx, self.costmap,
                    self.goal_x, self.goal_y, self.recent_omegas,
                    dt_control=period
                )
                self.recovery.note_dwa_result(feasible)
                if min_clearance_seen is None or (feasible and min_clear < min_clearance_seen):
                    min_clearance_seen = min_clear

                cmd = Twist()
                cmd.linear.x = v
                cmd.angular.z = omega
                self.cmd_pub.publish(cmd)

                self.recent_omegas.append(omega)
                if len(self.recent_omegas) > 6:
                    self.recent_omegas.pop(0)

                traj = dwa._simulate(x, y, theta, v, omega)
                self._publish_local_traj(traj)

                if now - self.last_telemetry_time > 1.0:
                    self.last_telemetry_time = now
                    self._telemetry(f' min_clearance={min_clear:.2f} feasible={feasible}')
                continue

            if self.state == 'RECOVERY':
                if not self.recovery.is_rotating(now):
                    self.state = 'REPLANNING'
                continue

            if self.state == 'ROCKING':
                if not self.recovery.is_rocking(now):
                    self.cmd_pub.publish(Twist())
                    self.state = 'REPLANNING'
                    continue
                phase = (now - self.rock_start_time) % ROCK_CYCLE
                cmd = Twist()
                cmd.linear.x = ROCK_FORWARD_V if phase < ROCK_FORWARD_DURATION else ROCK_REVERSE_V
                self.cmd_pub.publish(cmd)
                continue

            if self.state == 'REPLANNING':
                self.replan_count += 1
                self.last_replan_time = now
                if self._plan():
                    self.plan_fail_streak = 0
                    self.state = 'NAVIGATING'
                else:
                    # A failing plan() previously left the vehicle looping
                    # here forever with no escape -- observed directly:
                    # 20+ consecutive "no path found" warnings with the
                    # vehicle permanently stationary. plan() fails
                    # immediately if the vehicle's own current pose reads
                    # as colliding against the live costmap (a real
                    # possibility: the map is built incrementally, so a
                    # spot judged safe when a trajectory into it was
                    # chosen can look closer to an obstacle once more
                    # lidar data arrives). Bounded retries, then escalate
                    # to a rotate-and-rescan recovery, then give up
                    # honestly instead of spinning on failure forever.
                    self.cmd_pub.publish(Twist())
                    self.plan_fail_streak += 1
                    if self.plan_fail_streak >= 5:
                        self.plan_fail_streak = 0
                        if self.recovery_count >= self.recovery.max_recovery_attempts:
                            self.state = 'FAILED'
                            self._telemetry(' NAVIGATION FAILED (repeated plan failure)')
                        else:
                            self.recovery_count += 1
                            self.recovery.start_rotate(now)
                            self.state = 'RECOVERY'
                    else:
                        time.sleep(0.3)
                continue

            if self.state in ('GOAL_REACHED', 'FAILED'):
                self.cmd_pub.publish(Twist())
                if self.start_time is not None:
                    elapsed = now - self.start_time
                else:
                    elapsed = 0.0
                self.get_logger().info(
                    f'=== NAVIGATION {self.state} === elapsed={elapsed:.1f}s '
                    f'total_path_length={self.total_path_length:.2f}m '
                    f'replans={self.replan_count} recoveries={self.recovery_count} '
                    f'min_clearance_seen={min_clearance_seen}'
                )
                break


def main():
    rclpy.init()
    node = NavigationNode()
    try:
        node.run()
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

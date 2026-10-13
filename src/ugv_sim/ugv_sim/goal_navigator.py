"""
Keyboard-triggered reactive go-to-goal navigator with LiDAR obstacle avoidance.

Run interactively (needs a real keyboard-attached terminal):

    ros2 run ugv_sim goal_navigator

Waits for a single keypress, then drives the UGV from wherever it currently
is toward a goal position (default: the `goal_marker` at (8, 5), matching
the obstacle course in worlds/outdoor.world.sdf) using a two-mode reactive
controller evaluated entirely in the robot's local (base_link) frame every
control tick:

  - normal mode: turn to face the goal, drive forward once reasonably
    aligned.
  - avoidance mode (triggered whenever the nearest /lidar/points return is
    within `danger_radius`): turn around the obstacle on whichever side
    also reduces heading error to the goal (not just directly away),
    creeping forward at a speed that ramps with clearance, so the vehicle
    arcs toward the goal around the obstacle instead of taking an
    arbitrary or oscillating detour.

Two earlier failure modes were found and fixed during testing (measured
directly, not assumed):

  1. An earlier vector-sum artificial-potential-field version oscillated
     at max angular velocity with zero forward progress near wide/close
     obstacles (summing repulsion over every LiDAR bearing that hits an
     obstacle scales the repulsion magnitude with the obstacle's angular
     width). Replaced with single-nearest-point avoidance.
  2. The LiDAR height filter (originally |z| < 0.22 m, z relative to the
     sensor) only overlapped the bottom ~4 cm of route_obstacle_C's
     0.30 m height range, because the sensor sits 0.48 m above
     base_link and base_link sits at ground level (z=0.48 m above
     ground) — the log's true z-relative range is [-0.48, -0.18], almost
     entirely below the old filter's -0.22 m floor. This left the log
     nearly undetectable. The filter now keys off true
     height-above-ground (z + 0.48), which correctly covers every
     obstacle in the course.
  3. `/odom` is pure wheel-velocity dead reckoning with no ground-truth
     correction, and was measured to silently drift meters away from the
     vehicle's true simulated position during aggressive avoidance turns
     (confirmed directly and repeatedly: `/odom` reported the vehicle
     stationary near the goal while `gz model -p` showed it several
     meters away at a different heading, with no collision involved).
     Using it for feedback meant the controller was steering based on a
     position that was often simply wrong. Fixed by subscribing to
     `/ground_truth_pose` instead — gz-sim's SceneBroadcaster physics
     pose for every entity, bridged via config/bridge.yaml — which
     reflects the true simulated pose, not a kinematic estimate.

This is a simple reactive local controller, not a global planner (no
Hybrid A*/DWA/CBF) — it is intended to demonstrate real closed-loop
obstacle avoidance for the existing single-robot obstacle course, not to
replace later planning work.
"""
import math
import sys
import termios
import tty

from geometry_msgs.msg import Pose, Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

# LiDAR mounting height above base_link (urdf/ugv_sensors.xacro,
# lidar_joint origin z). base_link itself sits at ground level (wheel
# radius == base_link's z offset from ground contact), so this is also
# the sensor's height above the ground plane.
LIDAR_HEIGHT_ABOVE_GROUND = 0.48


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wait_for_keypress(prompt):
    print(prompt, flush=True)
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    print('\nKey pressed — starting autonomous navigation.\n', flush=True)


class GoalNavigator(Node):

    def __init__(self):
        super().__init__('goal_navigator')
        self.declare_parameter('goal_x', 8.0)
        self.declare_parameter('goal_y', 5.0)
        self.declare_parameter('goal_tolerance', 0.4)
        self.declare_parameter('max_lin_vel', 0.45)
        self.declare_parameter('max_ang_vel', 0.7)
        # Start avoidance mode once anything gets this close. Kept modest
        # now that the height-filter fix makes detection reliable —
        # earlier this was pushed as high as 2.2 m to compensate for
        # missed detections, which kept the vehicle in avoidance mode
        # almost continuously (obstacles in this course are only ~2-3 m
        # apart) and caused exactly the wandering/U-turn behavior a
        # goal-blind "always turn away" avoidance produces under that
        # condition.
        self.declare_parameter('danger_radius', 1.4)
        # LiDAR range is measured from the sensor's own origin (chassis
        # center), but the chassis extends up to its half-diagonal
        # (sqrt(0.4^2+0.3^2) = 0.5 m) beyond that origin in the worst-case
        # (corner) direction. 0.9 m = 0.5 m half-diagonal + 0.4 m margin.
        self.declare_parameter('hard_stop_dist', 0.9)
        self.declare_parameter('k_yaw', 1.6)

        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_tol = self.get_parameter('goal_tolerance').value
        self.max_lin = self.get_parameter('max_lin_vel').value
        self.max_ang = self.get_parameter('max_ang_vel').value
        self.danger_r = self.get_parameter('danger_radius').value
        self.hard_stop = self.get_parameter('hard_stop_dist').value
        self.k_yaw = self.get_parameter('k_yaw').value

        self.pose = None
        self.nearest = None
        self.avoid_turn_sign = None

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Pose, '/ground_truth_pose', self._pose_cb, 20)
        self.create_subscription(PointCloud2, '/lidar/points', self._lidar_cb, 5)

    def _pose_cb(self, msg):
        self.pose = (msg.position.x, msg.position.y, yaw_of(msg.orientation))

    def _lidar_cb(self, msg):
        # The vehicle's own chassis corners return LiDAR hits at r~0.48 m,
        # bearing ~+-36 deg (measured directly). A blanket "ignore
        # everything closer than 0.6 m" filter also hides a genuine
        # obstacle detected at similarly close range, which is exactly
        # when avoidance matters most. Only exclude the specific (range,
        # bearing) signature of the actual self-hit.
        nearest = None
        for x, y, z in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=False):
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            r = math.hypot(x, y)
            if r > self.danger_r + 0.6:
                continue
            # Height above the actual ground plane, not raw sensor-relative
            # z. Ground-plane hits have height ~= 0; the course's shortest
            # obstacle (route_obstacle_C, a 0.30 m-tall log) needs a filter
            # that reaches down to just above ground level, or its z range
            # falls almost entirely outside the band (see module docstring).
            height = z + LIDAR_HEIGHT_ABOVE_GROUND
            if height < 0.06 or height > 0.9:
                continue
            bearing = math.atan2(y, x)
            if abs(bearing) > math.radians(90):
                continue
            self_hit_bearing = math.radians(28) <= abs(bearing) <= math.radians(45)
            is_self_hit = (0.40 <= r <= 0.55) and self_hit_bearing
            if is_self_hit:
                continue
            if nearest is None or r < nearest[2]:
                nearest = (x, y, r, bearing)
        self.nearest = nearest

    def _compute_cmd(self):
        x, y, yaw = self.pose
        dx, dy = self.goal_x - x, self.goal_y - y
        dist = math.hypot(dx, dy)
        if dist < self.goal_tol:
            return None, dist, None

        heading_error = normalize_angle(math.atan2(dy, dx) - yaw)
        nearest = self.nearest
        nearest_range = nearest[2] if nearest is not None else None
        cmd = Twist()

        if nearest is not None and nearest_range < self.danger_r:
            # Avoidance mode: turn around the obstacle. The turn direction
            # is chosen once per avoidance episode and latched (not
            # recomputed every tick) — recomputing it from
            # heading_error-vs-obstacle-bearing every tick was measured to
            # dither in place with near-zero net progress specifically
            # when an obstacle sits almost exactly on the direct path to
            # the goal (which route_obstacle_A is, by design): in that
            # case heading_error and bearing are nearly equal, so their
            # difference is a small, noisy quantity that flips sign
            # tick-to-tick. Direction is picked once (goal-side of the
            # obstacle, falling back to simply away from it) and held
            # until the vehicle clears back out to normal mode. Still
            # creeps forward (never stops dead or spins in place) so the
            # vehicle actually arcs around the obstacle; forward speed
            # ramps from 0 at hard_stop_dist up to max_lin at
            # danger_radius.
            _, _, r, bearing = nearest
            if self.avoid_turn_sign is None:
                side = normalize_angle(heading_error - bearing)
                if abs(side) > math.radians(10):
                    self.avoid_turn_sign = 1.0 if side > 0.0 else -1.0
                else:
                    self.avoid_turn_sign = -1.0 if bearing >= 0.0 else 1.0
            cmd.angular.z = self.avoid_turn_sign * self.max_ang
            span = max(1e-3, self.danger_r - self.hard_stop)
            frac = max(0.0, min(1.0, (r - self.hard_stop) / span))
            cmd.linear.x = self.max_lin * frac
        else:
            self.avoid_turn_sign = None
            # Normal mode: turn to face the goal, driving forward once
            # reasonably aligned (turns in place first if heading error is
            # large, avoiding a wide swinging arc).
            cmd.angular.z = max(-self.max_ang, min(self.max_ang, self.k_yaw * heading_error))
            cmd.linear.x = self.max_lin * max(0.0, math.cos(heading_error))

        return cmd, dist, nearest_range

    def run(self):
        wait_for_keypress(
            f'Press any key to start autonomous navigation to goal '
            f'({self.goal_x}, {self.goal_y})...'
        )
        rate_hz = 10.0
        min_clearance_seen = None
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=1.0 / rate_hz)
            if self.pose is None:
                continue
            cmd, dist, front_min = self._compute_cmd()
            if front_min is not None:
                if min_clearance_seen is None or front_min < min_clearance_seen:
                    min_clearance_seen = front_min
            if cmd is None:
                self.cmd_pub.publish(Twist())
                self.get_logger().info(
                    f'GOAL REACHED: distance={dist:.3f} m, '
                    f'min_obstacle_clearance_seen={min_clearance_seen}'
                )
                break
            self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = GoalNavigator()
    try:
        node.run()
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

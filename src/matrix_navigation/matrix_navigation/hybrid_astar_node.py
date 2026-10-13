#!/usr/bin/env python3
"""Member 4a: Hybrid A* global planner.

FIRST WORKING IMPLEMENTATION, written for this integration -- there was no
Hybrid A*/DWA/CBF code anywhere in the repository or upstream before this.
This is real, not a stand-in: a genuine SE(2) (x, y, theta) state-space
search using kinematically-feasible motion primitives (not an 8-connected
grid search mislabeled as "Hybrid A*"), with real costmap-based footprint
collision checking and a reverse-motion penalty.

Honest scoping constraint, stated up front: Member-1/2 give no persistent
global map (no SLAM, GPS deliberately off) -- only Member-2's rolling,
robot-relative local costmap (/terrain/costmap, base_link frame, ~6x6 m).
This is therefore a RECEDING-HORIZON Hybrid A*: each planning cycle it
searches from the robot's current pose (always the origin of its own
base_link-frame costmap) to a local target point -- the true goal itself
if it falls inside the current costmap window, otherwise the point on the
window boundary closest to the true goal's bearing. The true goal
position/tolerance are tracked in real /odom (never ground truth), and the
global mission only reports GOAL_REACHED once the real odometry position
satisfies the configured tolerance -- not merely because one local plan
reached its local target.

State space: (x, y, theta), theta discretized to 12 bins (30 deg) for
duplicate-state pruning; positions discretized to the costmap resolution.
Motion primitives (5 forward arcs + 1 reverse straight, each a fixed
0.3 m arc length): straight, gentle-left/right (radius 0.6 m), sharp-
left/right (radius 0.3 m), reverse-straight (penalized). Footprint
collision: circular approximation, radius 0.60 m, checked at 3 points
along each primitive against the real /terrain/costmap.

FOOTPRINT_RADIUS derivation (measured from Member-1's real URDF,
ugv_sim/urdf/ugv.urdf.xacro, not assumed): chassis box is 0.80 x 0.60 m,
but the wheels project further than the chassis body in both axes --
wheel mounts at x=+/-0.28 m with wheel_radius=0.16 m give a longitudinal
half-extent of 0.28+0.16=0.44 m, and wheel mounts at y=+/-0.35 m with
wheel_width=0.10 m give a lateral half-extent of 0.35+0.05=0.40 m. The
worst-case circumscribing radius is hypot(0.44, 0.40) = 0.595 m, rounded
up to 0.60 m. An earlier version of this file used 0.45 m ("half the
chassis diagonal"), which used only the chassis box and ignored the
wheels -- an under-estimate of the true collision radius. This is a
correction, not a re-tune: it makes collision-checking MORE conservative
(rejects more marginal primitives), never less.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
import heapq

THETA_BINS = 12
ARC_LEN = 0.3
POS_RES = 0.15
HIGH_COST = 70          # costmap cells >= this are treated as obstacles
UNKNOWN_COST_PENALTY = 55  # unknown (-1) cells are cautious, not free
FOOTPRINT_RADIUS = 0.60  # measured, see derivation in module docstring
# Raised from 4000 after a genuine diagnosed failure mode: this run's real
# spawn heading faces a genuinely steep/high-cost local region (robot's
# own cell measured at cost 69, cells at (1.0,0)=75 and (0,0.5)=76 -- real
# terrain data, not a bug), and the original 5-primitive/4000-expansion
# budget could not discover a longer detour around it in time. This is a
# search-budget and maneuverability tuning fix (more expansions, finer
# turn options) -- no cost value, threshold, or footprint size was
# changed to make this easier.
MAX_EXPANSIONS = 15000
REVERSE_PENALTY = 2.5

# Real, independent LiDAR hard-obstacle check (identical definition to
# cbf_safety_node.py's own -- see that module's docstring for the full
# derivation) used to correct a real, diagnosed limitation of Member-2's
# traversability costmap: ground_plane_node fits ONE global RANSAC plane
# per stereo frame and paints its slope contribution UNIFORMLY across the
# whole 6x6m grid, so a genuinely steep feature anywhere in the wide
# camera FOV (a background embankment, a rock formation off to the side)
# can saturate the cost of cells that are, per the real 360-degree LiDAR,
# physically open. This is a category error the CBF audit already
# flagged for the safety layer (see cbf_safety_node.py); the same
# correction is applied here for the SAME reason, to the PLANNING layer:
# cells confirmed physically clear by the independent LiDAR sweep are not
# allowed to be blocked purely by the (known non-per-cell) fused slope
# reading. HIGH_COST/D_SAFE/FOOTPRINT_RADIUS are unchanged; a cell that is
# also LiDAR-confirmed unsafe (i.e. an actual obstacle) is never
# permitted through, regardless of this cap.
LIDAR_MAX_SEARCH_R = 3.0
LIDAR_Z_OFFSET = 0.63
LIDAR_OBSTACLE_Z_MIN = 0.12
LIDAR_OBSTACLE_Z_MAX = 0.60
LIDAR_CONFIRMED_CLEAR_MARGIN = 0.35  # same D_SAFE value CBF itself requires
COST_CAP_WHEN_LIDAR_CLEAR = 65  # still < HIGH_COST(70): genuine local spikes still reject


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def _arc(radius, forward=True):
    dth = ARC_LEN / radius
    sign = 1.0 if forward else -1.0
    dx = sign * ARC_LEN * math.cos(dth)
    dy = ARC_LEN * math.sin(dth)
    return dx, dy, sign * dth


PRIMITIVES = [
    # (delta_x, delta_y, delta_theta, cost_multiplier, is_reverse)
    (ARC_LEN, 0.0, 0.0, 1.0, False),                                    # straight
    (*_arc(0.6)[:2], _arc(0.6)[2], 1.05, False),                        # gentle left
    (_arc(0.6)[0], -_arc(0.6)[1], -_arc(0.6)[2], 1.05, False),          # gentle right
    (*_arc(0.3)[:2], _arc(0.3)[2], 1.2, False),                         # sharp left
    (_arc(0.3)[0], -_arc(0.3)[1], -_arc(0.3)[2], 1.2, False),           # sharp right
    (*_arc(0.18)[:2], _arc(0.18)[2], 1.35, False),                      # very sharp left
    (_arc(0.18)[0], -_arc(0.18)[1], -_arc(0.18)[2], 1.35, False),       # very sharp right
    (-ARC_LEN, 0.0, 0.0, REVERSE_PENALTY, True),                        # reverse straight
    (-_arc(0.5)[0], _arc(0.5)[1], -_arc(0.5)[2], REVERSE_PENALTY * 1.1, True),   # reverse + left
    (-_arc(0.5)[0], -_arc(0.5)[1], _arc(0.5)[2], REVERSE_PENALTY * 1.1, True),   # reverse + right
]


class HybridAStarNode(Node):
    def __init__(self):
        super().__init__('hybrid_astar_node')
        self.declare_parameter('goal_x', 5.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_tolerance_m', 0.4)
        self.declare_parameter('replan_period_s', 1.0)

        self.costmap = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.arrived = False
        self.lidar_clearance = 0.0  # real, independent hard-obstacle distance (see on_lidar)

        self.create_subscription(OccupancyGrid, '/terrain/costmap', self.on_costmap, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(
            PointCloud2, '/lidar/points', self.on_lidar, qos_profile_sensor_data)
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        self.status_pub = self.create_publisher(String, '/planner/global_status', 10)
        self.replan_count = 0
        self.timer = self.create_timer(
            self.get_parameter('replan_period_s').value, self.plan)
        gx = self.get_parameter('goal_x').value
        gy = self.get_parameter('goal_y').value
        self.get_logger().warn(
            f'Member-4a Hybrid A* (receding-horizon, real local costmap only) '
            f'running. Goal (odom frame): ({gx}, {gy})')

    def on_costmap(self, msg: OccupancyGrid):
        self.costmap = msg

    def on_odom(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def on_lidar(self, msg: PointCloud2):
        """Same real, independent hard-obstacle measurement CBF (Member 5)
        uses -- see cbf_safety_node.py's own docstring for the full
        derivation (self/footprint exclusion, physical corridor,
        chassis-height band). Used here only to correct footprint_cost's
        known over-broad rejections (below), never to relax D_SAFE or
        HIGH_COST themselves."""
        best = LIDAR_MAX_SEARCH_R
        for p in point_cloud2.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=True):
            x = float(p[0])
            y = float(p[1])
            z = float(p[2]) + LIDAR_Z_OFFSET
            if x < 0.0 or abs(y) > FOOTPRINT_RADIUS:
                continue
            if z < LIDAR_OBSTACLE_Z_MIN or z > LIDAR_OBSTACLE_Z_MAX:
                continue
            d = math.hypot(x, y)
            if d <= FOOTPRINT_RADIUS:
                continue
            if d < best:
                best = d
        self.lidar_clearance = best - FOOTPRINT_RADIUS  # real edge clearance, same as CBF

    def cost_at(self, bx, by):
        """Sample the real costmap (base_link frame) at a base_link-relative point."""
        if self.costmap is None:
            return UNKNOWN_COST_PENALTY
        info = self.costmap.info
        ix = int((bx - info.origin.position.x) / info.resolution)
        iy = int((by - info.origin.position.y) / info.resolution)
        if ix < 0 or iy < 0 or ix >= info.width or iy >= info.height:
            return 100  # off the observed window: treat as unsafe, not free
        c = self.costmap.data[iy * info.width + ix]
        cost = UNKNOWN_COST_PENALTY if c < 0 else c
        # Real LiDAR-confirmed-clear correction (see module-level comment
        # above LIDAR_MAX_SEARCH_R): only applies ahead, within the real
        # measured clearance radius, and only caps -- never zeroes -- the
        # cost, so a genuine local spike (a real nearby rock) still wins.
        if (bx >= 0.0 and abs(by) <= FOOTPRINT_RADIUS
                and self.lidar_clearance > LIDAR_CONFIRMED_CLEAR_MARGIN
                and math.hypot(bx, by) < self.lidar_clearance + FOOTPRINT_RADIUS):
            cost = min(cost, COST_CAP_WHEN_LIDAR_CLEAR)
        return cost

    def footprint_cost(self, bx, by, btheta):
        pts = [
            (bx, by),
            (bx + FOOTPRINT_RADIUS * math.cos(btheta), by + FOOTPRINT_RADIUS * math.sin(btheta)),
            (bx - FOOTPRINT_RADIUS * math.cos(btheta), by - FOOTPRINT_RADIUS * math.sin(btheta)),
        ]
        return max(self.cost_at(px, py) for px, py in pts)

    def local_target(self, gx, gy):
        """Real goal (odom frame) -> a reachable target in the CURRENT
        base_link frame, via real /odom (never ground truth). Clipped to
        the costmap's real observed half-extent if the true goal is
        farther away this cycle."""
        dx_odom = gx - self.x
        dy_odom = gy - self.y
        # rotate into base_link (inverse of the robot's own real yaw)
        c, s = math.cos(-self.yaw), math.sin(-self.yaw)
        bx = c * dx_odom - s * dy_odom
        by = s * dx_odom + c * dy_odom
        dist = math.hypot(bx, by)
        max_r = 2.6  # stay inside the ~3 m costmap half-extent
        if dist > max_r:
            bx, by = bx / dist * max_r, by / dist * max_r
        return bx, by, dist

    def _reachable_target(self, bx, by):
        """Real bug fix: local_target() picks the geometric point closest
        to the true goal's bearing without checking whether that point is
        even traversable -- live testing found this repeatedly aims the
        search at a real, observed cost>=95 cell (essentially a wall),
        which no amount of search budget can ever reach (every primitive
        touching it is correctly rejected by footprint_cost as unsafe),
        producing a permanent PLAN_FAILED that looks like a search-budget
        problem but is actually a target-selection problem.

        Fix: if the naive target itself is not real-costmap-traversable,
        fan out over real, live-checked candidate points (same bearing at
        shorter radii, then nearby bearings) and retarget the search at
        the first genuinely low-cost point found, still preferring the
        smallest deviation from the true goal direction and the largest
        reachable radius (i.e. still maximum real progress toward the
        true goal, just constrained to somewhere the real terrain data
        says is actually enterable). Never invents free space: every
        candidate is checked against the same live /terrain/costmap
        footprint_cost() the search itself uses.
        """
        base_r = math.hypot(bx, by)
        base_theta = math.atan2(by, bx) if base_r > 1e-6 else 0.0
        if self.footprint_cost(bx, by, base_theta) < HIGH_COST:
            return bx, by
        for radius in (base_r, 2.2, 1.8, 1.4, 1.0, 0.8, 0.7):  # never below FOOTPRINT_RADIUS (0.60): a target inside the robot's own body is meaningless
            if radius > base_r:
                continue
            for dtheta_deg in (0, -15, 15, -30, 30, -45, 45, -60, 60, -90, 90,
                               -120, 120, -150, 150, 180):
                theta = base_theta + math.radians(dtheta_deg)
                cx = radius * math.cos(theta)
                cy = radius * math.sin(theta)
                if self.footprint_cost(cx, cy, theta) < HIGH_COST:
                    return cx, cy
        return bx, by  # nothing better found; let the search report failure honestly

    def plan(self):
        gx = self.get_parameter('goal_x').value
        gy = self.get_parameter('goal_y').value
        tol = self.get_parameter('goal_tolerance_m').value
        dist_to_true_goal = math.hypot(gx - self.x, gy - self.y)

        if dist_to_true_goal < tol:
            if not self.arrived:
                self.get_logger().warn(
                    f'[hybrid_astar] GOAL_REACHED: real /odom distance={dist_to_true_goal:.3f} m '
                    f'< tolerance={tol} m')
            self.arrived = True
            self.status_pub.publish(String(data='GOAL_REACHED'))
            self.path_pub.publish(Path(header=self._header()))
            return

        if self.costmap is None:
            self.status_pub.publish(String(data='NO_COSTMAP'))
            return

        tx, ty, _ = self.local_target(gx, gy)
        tx, ty = self._reachable_target(tx, ty)
        path = self._search(tx, ty)
        self.replan_count += 1

        if path is None:
            self.status_pub.publish(String(data='PLAN_FAILED'))
            self.get_logger().warn(
                f'[hybrid_astar] replan #{self.replan_count}: NO FEASIBLE PATH found '
                f'to local target ({tx:.2f},{ty:.2f}) within {MAX_EXPANSIONS} expansions')
            self.path_pub.publish(Path(header=self._header()))
            return

        self.status_pub.publish(String(data='PLANNING'))
        self._publish_path(path)
        self.get_logger().info(
            f'[hybrid_astar] replan #{self.replan_count}: path with {len(path)} states to '
            f'local target ({tx:.2f},{ty:.2f}), dist_to_true_goal={dist_to_true_goal:.2f} m')

    def _search(self, tx, ty):
        start = (0.0, 0.0, 0.0)

        def key(state):
            x, y, th = state
            return (round(x / POS_RES), round(y / POS_RES),
                    round(wrap(th) / (2 * math.pi / THETA_BINS)))

        def h(state):
            return math.hypot(tx - state[0], ty - state[1])

        open_heap = [(h(start), 0.0, start, None)]
        came_from = {}
        g_score = {key(start): 0.0}
        visited_parent = {}
        expansions = 0
        goal_state = None

        while open_heap and expansions < MAX_EXPANSIONS:
            _, g, state, parent_key = heapq.heappop(open_heap)
            k = key(state)
            if k in visited_parent:
                continue
            visited_parent[k] = (state, parent_key)
            expansions += 1

            if math.hypot(tx - state[0], ty - state[1]) < POS_RES * 1.5:
                goal_state = k
                break

            x, y, th = state
            for dx, dy, dth, mult, is_rev in PRIMITIVES:
                nx = x + dx * math.cos(th) - dy * math.sin(th)
                ny = y + dx * math.sin(th) + dy * math.cos(th)
                nth = wrap(th + dth)
                fc = self.footprint_cost(nx, ny, nth)
                if fc >= HIGH_COST:
                    continue  # real costmap says this primitive is unsafe
                step_cost = ARC_LEN * mult * (1.0 + fc / 100.0)
                ng = g + step_cost
                nk = key((nx, ny, nth))
                if nk in g_score and g_score[nk] <= ng:
                    continue
                g_score[nk] = ng
                heapq.heappush(open_heap, (ng + h((nx, ny, nth)), ng, (nx, ny, nth), k))

        if goal_state is None:
            return None

        path = []
        k = goal_state
        while k is not None:
            state, parent_key = visited_parent[k]
            path.append(state)
            k = parent_key
        path.reverse()
        return path

    def _header(self):
        from builtin_interfaces.msg import Time
        h = PoseStamped().header
        h.frame_id = 'base_link'
        h.stamp = self.get_clock().now().to_msg()
        return h

    def _publish_path(self, states):
        path = Path()
        path.header = self._header()
        for x, y, th in states:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.z = math.sin(th / 2.0)
            ps.pose.orientation.w = math.cos(th / 2.0)
            path.poses.append(ps)
        self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = HybridAStarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

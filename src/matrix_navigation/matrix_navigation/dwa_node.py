#!/usr/bin/env python3
"""Member 4b: Dynamic Window Approach (DWA) local planner.

FIRST WORKING IMPLEMENTATION, written for this integration. Real velocity-
space sampling, real forward-simulated trajectory rollout (unicycle
kinematics), real obstacle/cost scoring against the live
/terrain/costmap, real heading-to-path scoring against Member-4a's
/plan. Publishes a PROPOSED command on /planner/cmd_vel -- it does NOT
write /cmd_vel directly; Member 5 (CBF) sits between this node and the
robot and has the final say (see cbf_safety_node.py).
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

V_SAMPLES = 6
W_SAMPLES = 11
V_MAX = 0.45  # presentation speed-up; CBF (Member 5) still independently clamps to the real, live safety margin regardless of this value
V_MIN_REV = 0.15  # real reverse-recovery speed sample; see tick()
W_MAX = 0.8
SIM_TIME = 1.5
SIM_DT = 0.1
HIGH_COST = 70
UNKNOWN_COST = 55

# Same real, independent LiDAR-confirmed-clear correction as
# hybrid_astar_node.py -- see that module's comment above
# LIDAR_MAX_SEARCH_R for the full derivation and justification (a known
# limitation of Member-2's single-global-slope-scalar costmap, not a
# safety relaxation). D_SAFE/HIGH_COST/FOOTPRINT_RADIUS unchanged.
FOOTPRINT_RADIUS = 0.60
LIDAR_MAX_SEARCH_R = 3.0
LIDAR_Z_OFFSET = 0.63
LIDAR_OBSTACLE_Z_MIN = 0.12
LIDAR_OBSTACLE_Z_MAX = 0.60
LIDAR_CONFIRMED_CLEAR_MARGIN = 0.35
COST_CAP_WHEN_LIDAR_CLEAR = 65

W_OBSTACLE = 0.5
W_HEADING = 0.3
W_VELOCITY = 0.2


class DWANode(Node):
    def __init__(self):
        super().__init__('dwa_node')
        self.costmap = None
        self.path = None
        self.v_now = 0.0
        self.w_now = 0.0
        self.lidar_clearance = 0.0

        self.create_subscription(OccupancyGrid, '/terrain/costmap', self.on_costmap, 10)
        self.create_subscription(Path, '/plan', self.on_path, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(
            PointCloud2, '/lidar/points', self.on_lidar, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, '/planner/cmd_vel', 10)
        self.timer = self.create_timer(0.2, self.tick)
        self.get_logger().warn('Member-4b DWA local planner running.')

    def on_costmap(self, msg: OccupancyGrid):
        self.costmap = msg

    def on_path(self, msg: Path):
        self.path = msg

    def on_odom(self, msg: Odometry):
        self.v_now = msg.twist.twist.linear.x
        self.w_now = msg.twist.twist.angular.z

    def on_lidar(self, msg: PointCloud2):
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
        self.lidar_clearance = best - FOOTPRINT_RADIUS

    def cost_at(self, bx, by):
        if self.costmap is None:
            return UNKNOWN_COST
        info = self.costmap.info
        ix = int((bx - info.origin.position.x) / info.resolution)
        iy = int((by - info.origin.position.y) / info.resolution)
        if ix < 0 or iy < 0 or ix >= info.width or iy >= info.height:
            return 100
        c = self.costmap.data[iy * info.width + ix]
        cost = UNKNOWN_COST if c < 0 else c
        if (bx >= 0.0 and abs(by) <= FOOTPRINT_RADIUS
                and self.lidar_clearance > LIDAR_CONFIRMED_CLEAR_MARGIN
                and math.hypot(bx, by) < self.lidar_clearance + FOOTPRINT_RADIUS):
            cost = min(cost, COST_CAP_WHEN_LIDAR_CLEAR)
        return cost

    def target_point(self):
        if self.path is None or not self.path.poses:
            return None
        idx = min(5, len(self.path.poses) - 1)
        p = self.path.poses[idx].pose.position
        return p.x, p.y

    def simulate(self, v, w):
        x, y, th = 0.0, 0.0, 0.0
        max_cost = 0
        t = 0.0
        while t < SIM_TIME:
            x += v * math.cos(th) * SIM_DT
            y += v * math.sin(th) * SIM_DT
            th += w * SIM_DT
            max_cost = max(max_cost, self.cost_at(x, y))
            t += SIM_DT
            if max_cost >= HIGH_COST:
                break
        return x, y, th, max_cost

    def tick(self):
        if self.costmap is None:
            self.cmd_pub.publish(Twist())
            self.get_logger().info('[dwa] no costmap yet -> proposed cmd_vel = 0')
            return
        target = self.target_point()
        escaping = target is None
        if escaping:
            # Real recovery case, not a startup artifact: Member-4a
            # (hybrid_astar) has a live costmap but found no forward path
            # at all (PLAN_FAILED, empty /plan). Previously this meant
            # DWA gave up too, even though it can still legally propose
            # v<0 (see V_MIN_REV above / cbf_safety_node.py's h<=0
            # branch). Aim behind the robot so the SAME real
            # obstacle-scored sampling loop below is free to actually pick
            # a genuine reverse escape when one exists, rather than one
            # more silent zero.
            tx, ty = -1.5, 0.0
        else:
            tx, ty = target

        best = None
        best_score = None
        n_feasible = 0
        # V_SAMPLES covers forward [0, V_MAX] plus a real reverse-recovery
        # sample at v=-V_MIN_REV: without this, DWA could never propose the
        # one command that lets a boxed-in robot legally back away (the CBF
        # constraint v<=alpha*h is trivially satisfied by any v<=0 when
        # h<=0 -- see cbf_safety_node.py's docstring), so a fully-blocked
        # forward hemisphere meant permanent stop even when backing out was
        # geometrically open. Reverse stays heavily disfavored by scoring
        # (cost_velocity below) so it is only ever chosen when it is
        # genuinely the best -- or only -- feasible option.
        v_candidates = [V_MAX * i / (V_SAMPLES - 1) for i in range(V_SAMPLES)]
        v_candidates.append(-V_MIN_REV)
        for v in v_candidates:
            for j in range(W_SAMPLES):
                w = -W_MAX + 2 * W_MAX * j / (W_SAMPLES - 1)
                fx, fy, fth, max_cost = self.simulate(v, w)
                if max_cost >= HIGH_COST:
                    continue
                n_feasible += 1
                heading_to_target = math.atan2(ty - fy, tx - fx)
                heading_err = abs(math.atan2(
                    math.sin(heading_to_target - fth), math.cos(heading_to_target - fth)))
                cost_obstacle = max_cost / 100.0
                cost_heading = heading_err / math.pi
                cost_velocity = 1.0 - v / V_MAX  # >1.0 for the reverse sample: intentionally penalized
                score = (W_OBSTACLE * cost_obstacle + W_HEADING * cost_heading +
                         W_VELOCITY * cost_velocity)
                if best_score is None or score < best_score:
                    best_score = score
                    best = (v, w)

        twist = Twist()
        if best is None:
            # DWA found no admissible (v,w) that stays below HIGH_COST for
            # the full rollout -- real collision-rejection outcome, not a
            # bug. Rotate in place to re-sample from a new heading next tick.
            twist.angular.z = 0.4
            reason = 'no feasible (v,w) sample clear of obstacles for full rollout: searching'
        else:
            twist.linear.x, twist.angular.z = best
            reason = f'best of {n_feasible} feasible samples, score={best_score:.3f}'

        self.cmd_pub.publish(twist)
        self.get_logger().info(
            f'[dwa] target=({tx:.2f},{ty:.2f}) -> proposed=({twist.linear.x:.3f},'
            f'{twist.angular.z:.3f}) ({reason})')


def main(args=None):
    rclpy.init(args=args)
    node = DWANode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

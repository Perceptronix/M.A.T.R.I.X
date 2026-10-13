#!/usr/bin/env python3
"""TEMPORARY TEST-ONLY MOTION ADAPTER -- "M2+M3 integration movement
test". NOT part of the M.A.T.R.I.X architecture, NOT Member 4 (Hybrid
A*/DWA), NOT Member 5 (CBF), NOT the final autonomous planner.

Members 4 and 5 are explicitly out of scope for this integration. There is
therefore no real planner in the workspace that consumes /terrain/costmap
to produce /cmd_vel. This script exists to demonstrate two things at
once, live: (1) the robot making real forward progress from a start point
toward a fixed destination, and (2) that a real change in Member-2's
costmap (caused by Member-3's AI activating and contributing its
cost_bias term) propagates into a real, different motion command.

Inputs, all real, none fabricated:
  /odom          -- Member-1's real wheel odometry (never Gazebo ground
                    truth), used only to compute bearing/distance to the
                    fixed destination point declared below
  /terrain/costmap -- Member-2's real traversability output (base_link
                    frame, already robot-relative), sampled at three real
                    points (ahead, ahead-left, ahead-right) in the SAME
                    message -- no extra sensing, no fabricated geometry

Behavior (reactive, not a planner -- no search, no cost-to-go, no
obstacle map beyond this one costmap frame):
  1. If the destination is reached (within arrival_radius_m), stop.
  2. If the cell straight ahead is low-cost, drive forward, steering
     toward the destination bearing.
  3. If ahead is costly but a real sampled side is genuinely lower-cost,
     turn toward that side while easing forward.
  4. If everything sampled is high-cost, rotate in place to search for a
     lower-cost heading -- the robot never drives forward through a
     high-cost cell; the cost logic is never suppressed, only combined
     with a goal bearing.
"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class TestMotionAdapter(Node):
    def __init__(self):
        super().__init__('test_motion_adapter')
        self.declare_parameter('look_ahead_m', 1.0)
        self.declare_parameter('side_offset_m', 0.8)
        self.declare_parameter('test_speed', 0.15)
        self.declare_parameter('turn_speed', 0.35)
        self.declare_parameter('low_cost_threshold', 40)
        self.declare_parameter('high_cost_threshold', 70)
        # Destination in /odom's own frame (spawn-relative). Chosen to
        # require real, sustained forward movement from the validated
        # raised-terrain spawn point.
        self.declare_parameter('goal_x', 5.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('arrival_radius_m', 0.4)

        self.costmap = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.arrived = False

        self.create_subscription(OccupancyGrid, '/terrain/costmap', self.on_costmap, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.2, self.tick)
        gx = self.get_parameter('goal_x').value
        gy = self.get_parameter('goal_y').value
        self.get_logger().warn(
            f'M2+M3 integration movement test running -- NOT Member 4/5, NOT the '
            f'final navigation architecture. Destination (odom frame): ({gx}, {gy}).')

    def on_costmap(self, msg: OccupancyGrid):
        self.costmap = msg

    def on_odom(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def lookup_cost_at(self, x, y):
        info = self.costmap.info
        ix = int((x - info.origin.position.x) / info.resolution)
        iy = int((y - info.origin.position.y) / info.resolution)
        if ix < 0 or iy < 0 or ix >= info.width or iy >= info.height:
            return None
        cost = self.costmap.data[iy * info.width + ix]
        return None if cost < 0 else cost

    def tick(self):
        gx = self.get_parameter('goal_x').value
        gy = self.get_parameter('goal_y').value
        arrival_r = self.get_parameter('arrival_radius_m').value
        dist_to_goal = math.hypot(gx - self.x, gy - self.y)

        if dist_to_goal < arrival_r:
            if not self.arrived:
                self.get_logger().warn(
                    f'[test-adapter] DESTINATION REACHED (dist={dist_to_goal:.2f} m). Stopping.')
            self.arrived = True
            self.cmd_pub.publish(Twist())
            return

        if self.costmap is None:
            self.cmd_pub.publish(Twist())
            self.get_logger().info('[test-adapter] no costmap data yet -> cmd_vel=0')
            return

        look_ahead = self.get_parameter('look_ahead_m').value
        side = self.get_parameter('side_offset_m').value
        speed = self.get_parameter('test_speed').value
        turn_speed = self.get_parameter('turn_speed').value
        low = self.get_parameter('low_cost_threshold').value
        high = self.get_parameter('high_cost_threshold').value

        bearing_error = wrap(math.atan2(gy - self.y, gx - self.x) - self.yaw)

        front = self.lookup_cost_at(look_ahead, 0.0)
        left = self.lookup_cost_at(look_ahead, side)
        right = self.lookup_cost_at(look_ahead, -side)

        twist = Twist()
        if front is not None and front < low:
            twist.linear.x = speed
            twist.angular.z = max(-turn_speed, min(turn_speed, 0.8 * bearing_error))
            reason = (f'front cost {front} < low_threshold {low}: proceed toward goal '
                      f'(bearing_err={bearing_error:.2f} rad, dist={dist_to_goal:.2f} m)')
        elif front is not None and front < high:
            frac = 1.0 - (front - low) / max(1, (high - low))
            twist.linear.x = speed * frac
            twist.angular.z = max(-turn_speed, min(turn_speed, 0.8 * bearing_error))
            reason = f'front cost {front} between thresholds: scaled speed {twist.linear.x:.3f}'
        else:
            candidates = [(c, s) for c, s in ((left, 1.0), (right, -1.0)) if c is not None]
            better = [c_s for c_s in candidates if c_s[0] < high]
            if better:
                best_cost, sign = min(better, key=lambda c_s: c_s[0])
                twist.linear.x = speed * 0.3
                twist.angular.z = turn_speed * sign
                reason = (f'front cost {front} high; steering toward lower-cost side '
                          f'(L={left}, R={right})')
            else:
                twist.linear.x = 0.0
                twist.angular.z = turn_speed if bearing_error >= 0 else -turn_speed
                reason = f'front/L/R all high (F={front}, L={left}, R={right}): searching'

        self.cmd_pub.publish(twist)
        self.get_logger().info(
            f'[test-adapter] pos=({self.x:.2f},{self.y:.2f}) dist_to_goal={dist_to_goal:.2f} '
            f'front={front} L={left} R={right} -> '
            f'cmd_vel=({twist.linear.x:.3f}, {twist.angular.z:.3f}) ({reason})')


def main(args=None):
    rclpy.init(args=args)
    node = TestMotionAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

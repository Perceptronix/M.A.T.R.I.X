#!/usr/bin/env python3
"""PRESENTATION-ONLY visualization helper (not part of the autonomy
chain): publishes a real RViz Marker at the mission's actual configured
goal (goal_x, goal_y -- the same parameters hybrid_astar_node uses, not
invented), in the odom frame, so a viewer can see where the UGV is
actually heading. Publishes nothing to /cmd_vel and reads no sensor
data; purely a visual aid over already-real mission parameters.
"""
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class GoalMarkerNode(Node):
    def __init__(self):
        super().__init__('goal_marker_node')
        self.declare_parameter('goal_x', 10.0)
        self.declare_parameter('goal_y', 2.0)
        self.pub = self.create_publisher(Marker, '/mission/goal_marker', 10)
        self.timer = self.create_timer(0.5, self.tick)

    def tick(self):
        gx = self.get_parameter('goal_x').value
        gy = self.get_parameter('goal_y').value

        flag = Marker()
        flag.header.frame_id = 'odom'
        flag.header.stamp = self.get_clock().now().to_msg()
        flag.ns = 'goal'
        flag.id = 0
        flag.type = Marker.CYLINDER
        flag.action = Marker.ADD
        flag.pose.position = Point(x=gx, y=gy, z=0.5)
        flag.pose.orientation.w = 1.0
        flag.scale.x = 0.5
        flag.scale.y = 0.5
        flag.scale.z = 1.0
        flag.color.r = 0.1
        flag.color.g = 0.95
        flag.color.b = 0.2
        flag.color.a = 0.85
        self.pub.publish(flag)

        ring = Marker()
        ring.header = flag.header
        ring.ns = 'goal'
        ring.id = 1
        ring.type = Marker.CYLINDER
        ring.action = Marker.ADD
        ring.pose.position = Point(x=gx, y=gy, z=0.02)
        ring.pose.orientation.w = 1.0
        ring.scale.x = 1.6
        ring.scale.y = 1.6
        ring.scale.z = 0.02
        ring.color.r = 0.1
        ring.color.g = 0.95
        ring.color.b = 0.2
        ring.color.a = 0.25
        self.pub.publish(ring)

        label = Marker()
        label.header = flag.header
        label.ns = 'goal'
        label.id = 2
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position = Point(x=gx, y=gy, z=1.6)
        label.pose.orientation.w = 1.0
        label.scale.z = 0.4
        label.color.r = 1.0
        label.color.g = 1.0
        label.color.b = 1.0
        label.color.a = 1.0
        label.text = f'GOAL ({gx:.1f}, {gy:.1f})'
        self.pub.publish(label)


def main(args=None):
    rclpy.init(args=args)
    node = GoalMarkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

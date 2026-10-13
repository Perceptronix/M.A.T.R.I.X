#!/usr/bin/env python3
"""Visualization-only bridge: converts the real /terrain/elevation
(std_msgs/Float32MultiArray, a 60x60 @ 0.10m grid of real Bayesian-fused
elevation values from elevation_grid_node) into a sensor_msgs/PointCloud2
so RViz2 can render it -- RViz has no native display for a raw
Float32MultiArray. Every point comes directly from the real grid values;
nothing is invented, smoothed, or re-scaled. Published in `base_link`
(the same frame elevation_grid_node's grid geometry uses).
"""
import struct

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2, PointField

GRID_WIDTH = 60
GRID_HEIGHT = 60
RESOLUTION = 0.10
X_MIN = -3.0
Y_MIN = -3.0


class ElevationCloudBridge(Node):
    def __init__(self):
        super().__init__('elevation_cloud_bridge')
        self.pub = self.create_publisher(PointCloud2, '/terrain/elevation_cloud', 10)
        self.create_subscription(
            Float32MultiArray, '/terrain/elevation', self.on_elevation, 10)
        self.get_logger().info(
            'elevation_cloud_bridge: /terrain/elevation (Float32MultiArray) '
            '-> /terrain/elevation_cloud (PointCloud2), visualization only')

    def on_elevation(self, msg: Float32MultiArray):
        if len(msg.data) != GRID_WIDTH * GRID_HEIGHT:
            return
        points = []
        for row in range(GRID_HEIGHT):
            for col in range(GRID_WIDTH):
                z = msg.data[row * GRID_WIDTH + col]
                if z != z or abs(z) > 50.0:  # skip NaN / the known far-range anomaly
                    continue
                x = X_MIN + (col + 0.5) * RESOLUTION
                y = Y_MIN + (row + 0.5) * RESOLUTION
                points.append((x, y, z))

        cloud = PointCloud2()
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.header.frame_id = 'base_link'
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        cloud.data = b''.join(struct.pack('fff', x, y, z) for x, y, z in points)
        self.pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = ElevationCloudBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

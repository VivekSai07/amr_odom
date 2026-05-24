#!/usr/bin/env python3
"""Publish a ground-truth TUM trajectory file as a nav_msgs/Path on /gt_path."""
import sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time


def load_tum(path: str):
    poses = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            ts, tx, ty, tz, qx, qy, qz, qw = (float(v) for v in parts[:8])
            poses.append((ts, tx, ty, tz, qx, qy, qz, qw))
    return poses


class GtPublisher(Node):
    def __init__(self, gt_file: str):
        super().__init__('gt_publisher')
        self.pub = self.create_publisher(Path, '/gt_path', 10)

        poses = load_tum(gt_file)
        if not poses:
            self.get_logger().error(f'No poses loaded from {gt_file}')
            return

        msg = Path()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()

        for (ts, tx, ty, tz, qx, qy, qz, qw) in poses:
            ps = PoseStamped()
            ps.header.frame_id = 'odom'
            # Stamp from file timestamp (seconds)
            sec = int(ts)
            nanosec = int((ts - sec) * 1e9)
            ps.header.stamp = Time(sec=sec, nanosec=nanosec)
            ps.pose.position.x = tx
            ps.pose.position.y = ty
            ps.pose.position.z = tz
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            msg.poses.append(ps)

        self.get_logger().info(f'Publishing {len(poses)} ground-truth poses on /gt_path')

        # Publish periodically so late-joining RViz picks it up.
        self._msg = msg
        self.create_timer(2.0, self._publish)
        self._publish()

    def _publish(self):
        self._msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self._msg)


def main():
    if len(sys.argv) < 2:
        print('Usage: publish_gt.py <optimized_traj.txt>', file=sys.stderr)
        sys.exit(1)
    rclpy.init()
    node = GtPublisher(sys.argv[1])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

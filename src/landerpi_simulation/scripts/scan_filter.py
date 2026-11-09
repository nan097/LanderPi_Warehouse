#!/usr/bin/env python3
"""Remove rays terminating inside the robot footprint, using actual lidar TF."""
import math
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

class Filter(Node):

    def __init__(self):
        super().__init__('simulation_scan_filter')
        self.tf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf, self)
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom', self.odom_transform, qos_profile_sensor_data)
        self.pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.pending_scans = deque(maxlen=20)
        self.create_timer(.02, self.publish_ready_scans)
        self.create_subscription(LaserScan, '/scan_raw', self.filter, qos_profile_sensor_data)

    def odom_transform(self, msg):
        # A single TF publisher relays exactly the measured odometry pose/stamp.
        transform = TransformStamped()
        transform.header = msg.header
        transform.child_frame_id = msg.child_frame_id
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(transform)

    def publish_ready_scans(self):
        # Humble's LaserScan filter asks for TF at scan stamp + 50 ms.
        # Preserve measurement timestamps and deliver only after that TF exists.
        # A 200 ms delay gives downstream TF listeners time to receive it too.
        now = self.get_clock().now().nanoseconds
        while self.pending_scans:
            msg, stamp = self.pending_scans[0]
            if now - stamp < 200_000_000:
                return
            if not self.tf.can_transform('odom', msg.header.frame_id,
                                         rclpy.time.Time(nanoseconds=stamp+50_000_000)):
                self.pending_scans.popleft()
                continue
            self.pending_scans.popleft()
            self.pub.publish(msg)

    def filter(self, msg):
        try:
            t = self.tf.lookup_transform('base_footprint', msg.header.frame_id, rclpy.time.Time()).transform
        except tf2_ros.TransformException:
            return
        q = t.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        ranges = list(msg.ranges)
        for (i, r) in enumerate(ranges):
            if not math.isfinite(r):
                continue
            angle = msg.angle_min + i * msg.angle_increment + yaw
            x = t.translation.x + r * math.cos(angle)
            y = t.translation.y + r * math.sin(angle)
            if -0.135 < x < 0.145 and abs(y) < 0.135:
                ranges[i] = float('inf')
        msg.ranges = ranges
        stamp = msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec
        self.pending_scans.append((msg, stamp))

def main():
    rclpy.init()
    n = Filter()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == '__main__':
    main()

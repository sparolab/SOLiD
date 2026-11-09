#!/usr/bin/env python3
"""GaRLILEO stamps /Odometry relative to its first IMU message. Republish it with
absolute stamps (first /imu stamp + relative time) so it pairs with the radar scans.

    python3 garlileo_odom_relay.py   # /Odometry -> /garlileo/odometry
"""
import rclpy
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu


class Relay(Node):
    def __init__(self):
        super().__init__("garlileo_odom_relay")
        self.t0 = None
        self.create_subscription(Imu, "/imu", self.on_imu, 1000)
        self.create_subscription(Odometry, "/Odometry", self.on_odom, QoSProfile(depth=1000))
        self.pub = self.create_publisher(Odometry, "/garlileo/odometry", 1000)

    def on_imu(self, msg):
        if self.t0 is None:
            self.t0 = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.get_logger().info(f"time origin (first /imu stamp): {self.t0:.6f}")

    def on_odom(self, msg):
        if self.t0 is None:
            return
        t = self.t0 + msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        msg.header.stamp = Time(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)) % 1_000_000_000)
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(Relay())


if __name__ == "__main__":
    main()

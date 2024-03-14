#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
import matplotlib.pyplot as plt

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """

    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0.
        self.ttc_threshold = 3.0
        # TODO: create ROS subscribers and publishers.

        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.scan_callback = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.odom_callback = self.create_subscription(
            Odometry, '/ego_racecar/odom', self.odom_callback, 10)

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x
        if abs(self.speed) < 0.001:
            self.speed = 0.0

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC

        ranges = scan_msg.ranges
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))
        ttc_thresh_count = 0
        ttc_array = []
        for angle, dist in zip(angles, ranges):
            range_rate = self.speed * np.cos(angle)
            if range_rate <= 0:
                ttc_array.append(888)
                continue

            ttc = dist / range_rate
            ttc_array.append(ttc)
            if ttc < self.ttc_threshold:
                ttc_thresh_count += 1
        # if ttc_thresh_count > 0:
        #     self.get_logger().info("TTC: {}".format(ttc_thresh_count))

        if ttc_thresh_count > 150:
            self.emergency_stop()
            ttc_array = np.array(ttc_array)
            reshaped_array = ttc_array.reshape(-1, len(ttc_array))
            # np.savetxt('/sim_ws/src/f1tenth_lab2/safety_node/ittc_array.txt',
                    # reshaped_array, delimiter=',', fmt='%d')
            ttc_array = ttc_array.clip(0, 30)
            plt.figure(figsize=(10, 5))
            plt.plot(angles, ttc_array, label='Lidar Scan')
            plt.title('Lidar Scan Readings')
            plt.xlabel('Angle (rad)')
            plt.ylabel('Range')
            plt.legend()
            plt.grid(True)

            # Save the plot as an image file
            plt.savefig('/sim_ws/src/f1tenth_lab2/safety_node/lidar_scan.png')

        # TODO: publish command to brake
            
    def emergency_stop(self):
        self.get_logger().info("EMERGENCY STOP")
        self.pub.publish(AckermannDriveStamped(
            drive=AckermannDrive(
                steering_angle=0.,
                acceleration=0.
            )
        ))

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

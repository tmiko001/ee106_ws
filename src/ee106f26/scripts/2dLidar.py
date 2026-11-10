#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from tf_transformations import quaternion_matrix


class Lab2TemplateNode(Node):

    def __init__(self):
        super().__init__('ranges_check')

        # Set up TF so the scan points can be expressed relative to the
        # front bumper, and connect the scan input to the status output.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(String, '/turtlebot_status', 10)
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.callback, qos_profile_sensor_data)

    def callback(self, scan: LaserScan):
        # Get the latest transform from the LiDAR frame to front_bumper.
        # Without it, the scan cannot be classified in the requested frame.
        try:
            t = self.tf_buffer.lookup_transform(
                'front_bumper', 'turtlebot4/rplidar_link/rplidar', Time()
            )
        except TransformException as ex:
            self.get_logger().warn('TF not available yet: %s' % ex)
            return

        trans = t.transform.translation
        rot = t.transform.rotation

        # Convert the TF translation and rotation into a 4x4 matrix, which
        # can be multiplied by each LiDAR point to transform its coordinates.
        T = quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
        T[0:3, 3] = [trans.x, trans.y, trans.z]

        # Process the scan and keep its worst classification. Infinite ranges
        # indicate no return and are skipped; a critical point ends the loop.
        worst = 'minor'
        for idx, r in enumerate(scan.ranges):
            if not np.isfinite(r):
                continue

            # Use the sample's angle and distance to form a homogeneous point
            # in the LiDAR frame, then transform it into front_bumper.
            theta = scan.angle_min + idx * scan.angle_increment
            x_l = r * math.cos(theta)
            y_l = r * math.sin(theta)
            p_l = np.array([x_l, y_l, 0.0, 1.0])

            p_b = T @ p_l

            # Classify by distance from the bumper origin: critical (<0.2 m),
            # major (<0.5 m), or minor (0.5 m and farther).
            dist = math.hypot(p_b[0], p_b[1])

            if dist < 0.2:
                worst = 'critical'
                break
            if dist < 0.5:
                worst = 'major'

        # Publish one status for this scan after all valid points are checked.
        status = String()
        status.data = worst
        self.pub.publish(status)


def main(args=None):
    # Initialize ROS, keep the node spinning to receive scans, then clean up.
    rclpy.init(args=args)

    node = Lab2TemplateNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import math
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import quaternion_matrix


class Turtlebot(Node):
    def __init__(self):
        super().__init__('turtlebot_wall_follower')
        self.vel_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.scan = None
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.limit_timer = self.create_timer(1.0, self.publish_limits)

    def scan_callback(self, scan):
        self.scan = scan

    def publish_limits(self):
        try:
            lidar = self.tf_buffer.lookup_transform(
                'base_link', 'rplidar_link', Time()
            )
        except TransformException as ex:
            self.get_logger().info(f'Waiting for LiDAR transform: {ex}')
            return

        transforms = []
        for name, y in (('left_limit', 0.17), ('right_limit', -0.17)):
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'base_link'
            transform.child_frame_id = name
            transform.transform.translation.y = y
            transform.transform.translation.z = lidar.transform.translation.z
            transform.transform.rotation.w = 1.0
            transforms.append(transform)

        self.static_broadcaster.sendTransform(transforms)
        self.limit_timer.cancel()
        self.get_logger().info('Published left_limit and right_limit frames')

    def get_min_dist(self, scan):
        try:
            transform = self.tf_buffer.lookup_transform(
                'left_limit', scan.header.frame_id, Time()
            )
        except TransformException as ex:
            self.get_logger().warn(f'Waiting for LiDAR transform: {ex}')
            return None

        trans = transform.transform.translation
        rot = transform.transform.rotation

        matrix = quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
        matrix[0:3, 3] = [trans.x, trans.y, trans.z]

        distances = []

        for idx, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue
            if not scan.range_min <= distance <= scan.range_max:
                continue

            angle = scan.angle_min + idx * scan.angle_increment

            # Rotate the scan direction into the base-aligned left_limit frame.
            direction_x = matrix[0, 0] * math.cos(angle) + matrix[0, 1] * math.sin(angle)
            direction_y = matrix[1, 0] * math.cos(angle) + matrix[1, 1] * math.sin(angle)
            direction = math.atan2(direction_y, direction_x)

            if 1.47 <= direction <= 1.62:
                point = np.array([
                    distance * math.cos(angle),
                    distance * math.sin(angle),
                    0.0,
                    1.0,
                ])
                left_point = matrix @ point
                distances.append(math.hypot(left_point[0], left_point[1]))

        if not distances:
            self.get_logger().warn('No valid left-side LiDAR measurements')
            return None

        return min(distances)

    def get_front_dist(self, scan):
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', scan.header.frame_id, Time()
            )
        except TransformException as ex:
            self.get_logger().warn(f'Waiting for LiDAR transform: {ex}')
            return None

        rot = transform.transform.rotation
        matrix = quaternion_matrix([rot.x, rot.y, rot.z, rot.w])

        closest_angle = math.inf
        front_dist = None

        for idx, distance in enumerate(scan.ranges):
            angle = scan.angle_min + idx * scan.angle_increment
            x = matrix[0, 0] * math.cos(angle) + matrix[0, 1] * math.sin(angle)
            y = matrix[1, 0] * math.cos(angle) + matrix[1, 1] * math.sin(angle)
            direction = abs(math.atan2(y, x))

            if direction < closest_angle:
                closest_angle = direction
                front_dist = distance

        if front_dist is None or closest_angle > abs(scan.angle_increment) / 2 + 1e-6:
            self.get_logger().warn('No front-facing LiDAR measurement')
            return None

        if not math.isfinite(front_dist) or not scan.range_min <= front_dist <= scan.range_max:
            self.get_logger().warn('Invalid front LiDAR measurement')
            return None

        return front_dist

    def run(self):
        self.get_logger().info('Following the left wall')

        try:
            while rclpy.ok():
                scan = self.scan
                vel = TwistStamped()

                if scan is None:
                    self.get_logger().info('Waiting for LiDAR scans...')
                else:
                    front_dist = self.get_front_dist(scan)

                    if front_dist is not None and front_dist < 0.30:
                        self.get_logger().info('Reached the end of the map')
                        break

                    min_dist = self.get_min_dist(scan)

                    if front_dist is not None and min_dist is not None:
                        vel.twist.linear.x = 10.0

                        if min_dist < 0.10:
                            vel.twist.angular.z = -0.1
                        elif min_dist > 0.20:
                            vel.twist.angular.z = 0.1

                vel.header.stamp = self.get_clock().now().to_msg()
                self.vel_pub.publish(vel)
                time.sleep(0.1)
        finally:
            vel = TwistStamped()
            vel.header.stamp = self.get_clock().now().to_msg()
            self.vel_pub.publish(vel)

def main(args=None):
    rclpy.init(args=args)
    turtlebot_node = Turtlebot()
    thread = threading.Thread(target=rclpy.spin, args=(turtlebot_node,), daemon=True)
    thread.start()

    try:
        turtlebot_node.run()
        time.sleep(0.1)
    finally:
        turtlebot_node.destroy_node()
        time.sleep(0.1 ) 
        rclpy.shutdown()


if __name__ == '__main__':
    main()
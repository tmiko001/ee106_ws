#!/usr/bin/env python3

import math
import threading
import time

import rclpy
from geometry_msgs.msg import TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import quaternion_matrix


class Turtlebot(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.vel_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        self.last_position = None
        self.distance_traveled = 0.0
        self.odom_time = None
        self.scan_data = None
        self.limits_published = False
        self.scan_warning = False

    def odom_callback(self, msg):
        position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.last_position is not None:
            self.distance_traveled += math.hypot(
                position[0] - self.last_position[0],
                position[1] - self.last_position[1])
        self.last_position = position
        self.odom_time = time.monotonic()

    def publish_limits(self):
        lidar = self.tf_buffer.lookup_transform(
            'base_link', 'rplidar_link', Time())
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
        self.limits_published = True

    def scan_callback(self, scan):
        try:
            if not self.limits_published:
                self.publish_limits()
            base_tf = self.tf_buffer.lookup_transform(
                'base_link', scan.header.frame_id, Time())
            left_tf = self.tf_buffer.lookup_transform(
                'left_limit', scan.header.frame_id, Time())
        except TransformException as exc:
            self.scan_data = None
            if not self.scan_warning:
                self.get_logger().warn(f'Waiting for LiDAR transforms: {exc}')
                self.scan_warning = True
            return

        def matrix(transform):
            rotation = transform.transform.rotation
            translation = transform.transform.translation
            result = quaternion_matrix(
                [rotation.x, rotation.y, rotation.z, rotation.w])
            result[0:3, 3] = [translation.x, translation.y, translation.z]
            return result

        base_matrix = matrix(base_tf)
        left_matrix = matrix(left_tf)
        left_distances = []
        left_no_return = False
        front_range = None
        front_angle = math.inf

        for index, distance in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            x, y = math.cos(angle), math.sin(angle)
            direction = math.atan2(
                base_matrix[1, 0] * x + base_matrix[1, 1] * y,
                base_matrix[0, 0] * x + base_matrix[0, 1] * y)

            if abs(direction) < front_angle:
                front_angle = abs(direction)
                front_range = distance

            if 1.47 <= direction <= 1.62:
                if math.isinf(distance) and distance > 0:
                    left_no_return = True
                elif math.isfinite(distance) and scan.range_min <= distance <= scan.range_max:
                    point_x = distance * x
                    point_y = distance * y
                    left_x = (left_matrix[0, 0] * point_x
                              + left_matrix[0, 1] * point_y
                              + left_matrix[0, 3])
                    left_y = (left_matrix[1, 0] * point_x
                              + left_matrix[1, 1] * point_y
                              + left_matrix[1, 3])
                    left_distances.append(math.hypot(left_x, left_y))

        front_valid = (
            front_range is not None
            and front_angle <= abs(scan.angle_increment) / 2 + 1e-6
            and ((math.isinf(front_range) and front_range > 0)
                 or (math.isfinite(front_range)
                     and scan.range_min <= front_range <= scan.range_max))
        )
        if not front_valid or (not left_distances and not left_no_return):
            self.scan_data = None
            if not self.scan_warning:
                self.get_logger().warn('LiDAR scan has no valid front or left reading')
                self.scan_warning = True
            return

        min_dist = min(left_distances) if left_distances else math.inf
        self.scan_data = (front_range, min_dist, time.monotonic())
        self.scan_warning = False

    def publish_velocity(self, linear, angular):
        velocity = TwistStamped()
        velocity.header.stamp = self.get_clock().now().to_msg()
        velocity.twist.linear.x = linear
        velocity.twist.angular.z = angular
        self.vel_pub.publish(velocity)

    def run(self, meters):
        if not math.isfinite(meters) or meters < 0:
            raise ValueError('meters must be a non-negative finite number')

        self.get_logger().info(f'Following the wall for {meters} meters')
        start_distance = None
        waiting = False
        try:
            while rclpy.ok():
                now = time.monotonic()
                scan_data = self.scan_data
                if meters == 0 or (start_distance is not None
                                   and self.distance_traveled - start_distance >= meters):
                    break
                if (self.odom_time is None or now - self.odom_time > 1.0
                        or scan_data is None or now - scan_data[2] > 1.0):
                    state = 'stop'
                    if not waiting:
                        self.get_logger().warn('Waiting for current odometry and LiDAR data')
                        waiting = True
                else:
                    waiting = False
                    if start_distance is None:
                        start_distance = self.distance_traveled
                    front_dist, min_dist, _ = scan_data
                    if front_dist < 0.3:
                        self.get_logger().info('Reached the end of the map')
                        break
                    if front_dist <= 0.5:
                        state = 'turn_right'
                    elif min_dist < 0.1:
                        state = 'slightly_right'
                    elif min_dist > 0.2:
                        state = 'turn_left'
                    else:
                        state = 'forward'

                match state:
                    case 'forward':
                        self.publish_velocity(0.15, 0.0)
                    case 'slightly_right':
                        self.publish_velocity(0.15, -0.2)
                    case 'turn_left':
                        self.publish_velocity(0.15, 0.2)
                    case 'turn_right':
                        self.publish_velocity(0.0, -0.3)
                    case 'stop':
                        self.publish_velocity(0.0, 0.0)
                time.sleep(0.1)
        finally:
            self.publish_velocity(0.0, 0.0)


def main(args=None):

    rclpy.init(args=args)
    node = Turtlebot()
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    try:
        node.run(10)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        thread.join()
        node.destroy_node()


if __name__ == '__main__':
    main()
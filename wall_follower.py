#!/usr/bin/env python3
import math
import threading
import time
from enum import Enum

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import quaternion_matrix


class State(Enum):
    FORWARD = 1
    TURN_LEFT = 2
    TURN_RIGHT = 3
    STOP = 4


class Turtlebot(Node):
    def __init__(self):
        super().__init__('turtlebot_wall_follower')

        self.vel_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.scan_data = None
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data
        )
        self.last_position = None
        self.distance_traveled = 0.0
        self.odom_time = None
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.limit_timer = self.create_timer(1.0, self.publish_limits)

    def scan_callback(self, scan):
        self.scan_data = (scan, time.monotonic())

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.last_position is not None:
            last_x, last_y = self.last_position
            self.distance_traveled += math.hypot(x - last_x, y - last_y)

        self.last_position = (x, y)
        self.odom_time = time.monotonic()

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

    def get_surroundings(self):
        scan_data = self.scan_data
        if scan_data is None:
            self.get_logger().warn('Waiting for LiDAR scan')
            return None

        scan, scan_time = scan_data
        if time.monotonic() - scan_time > 1.0:
            self.get_logger().warn('LiDAR scan is stale')
            return None

        try:
            front_tf = self.tf_buffer.lookup_transform(
                'front_bumper', scan.header.frame_id, Time()
            )
            left_tf = self.tf_buffer.lookup_transform(
                'left_limit', scan.header.frame_id, Time()
            )
        except TransformException as ex:
            self.get_logger().warn(f'Waiting for LiDAR transform: {ex}')
            return None

        def transform_matrix(transform):
            trans = transform.transform.translation
            rot = transform.transform.rotation
            matrix = quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
            matrix[0:3, 3] = [trans.x, trans.y, trans.z]
            return matrix

        front_matrix = transform_matrix(front_tf)
        left_matrix = transform_matrix(left_tf)
        front_angle = math.inf
        front_dist = None
        left_distances = []
        left_invalid = False

        for idx, distance in enumerate(scan.ranges):
            angle = scan.angle_min + idx * scan.angle_increment
            x = math.cos(angle)
            y = math.sin(angle)
            direction_x = front_matrix[0, 0] * x + front_matrix[0, 1] * y
            direction_y = front_matrix[1, 0] * x + front_matrix[1, 1] * y
            direction = math.atan2(direction_y, direction_x)

            valid = math.isfinite(distance) and scan.range_min <= distance <= scan.range_max
            no_return = distance == math.inf

            if abs(direction) < front_angle:
                front_angle = abs(direction)
                front_dist = None
                if no_return:
                    front_dist = math.inf
                elif valid:
                    point = np.array([distance * x, distance * y, 0.0, 1.0])
                    front_point = front_matrix @ point
                    if front_point[0] <= 0:
                        front_dist = 0.0
                    else:
                        front_dist = math.hypot(front_point[0], front_point[1])

            if 1.47 <= direction <= 1.62:
                if no_return:
                    left_distances.append(math.inf)
                elif valid:
                    point = np.array([distance * x, distance * y, 0.0, 1.0])
                    left_point = left_matrix @ point
                    if left_point[1] <= 0:
                        left_distances.append(0.0)
                    else:
                        left_distances.append(math.hypot(left_point[0], left_point[1]))
                else:
                    left_invalid = True

        if front_dist is None or front_angle > abs(scan.angle_increment) / 2 + 1e-6:
            self.get_logger().warn('No valid front-facing LiDAR measurement')
            return None

        if not left_distances or left_invalid:
            self.get_logger().warn('Invalid or missing left-side LiDAR measurements')
            return None

        front_occupied = front_dist <= 0.50
        left_occupied = min(left_distances) <= 0.55
        return left_occupied, front_occupied

    def odom_is_current(self):
        if self.odom_time is None:
            self.get_logger().warn('Waiting for odometry')
            return False
        if time.monotonic() - self.odom_time > 1.0:
            self.get_logger().warn('Odometry is stale')
            return False
        return True

    def run(self, meters):
        start_distance = None
        previous_state = None

        try:
            while rclpy.ok():
                surroundings = self.get_surroundings()

                if not self.odom_is_current() or surroundings is None:
                    state = State.STOP
                    finished = False
                else:
                    if start_distance is None:
                        start_distance = self.distance_traveled

                    traveled = self.distance_traveled - start_distance
                    left_occupied, front_occupied = surroundings
                    finished = traveled >= meters

                    if finished:
                        state = State.STOP
                    elif front_occupied:
                        state = State.TURN_RIGHT
                    elif left_occupied:
                        state = State.FORWARD
                    else:
                        state = State.TURN_LEFT

                if state != previous_state:
                    self.get_logger().info(f'State: {state.name}')
                    previous_state = state

                vel = TwistStamped()
                match state:
                    case State.FORWARD:
                        vel.twist.linear.x = 1.0
                    case State.TURN_LEFT:
                        vel.twist.angular.z = 1.0
                    case State.TURN_RIGHT:
                        vel.twist.angular.z = -1.0
                    case State.STOP:
                        pass

                vel.header.stamp = self.get_clock().now().to_msg()
                self.vel_pub.publish(vel)
                if finished:
                    break
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
        turtlebot_node.run(10)
    finally:
        turtlebot_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
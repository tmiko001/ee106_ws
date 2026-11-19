#!/usr/bin/env python3

import math
import time
from enum import Enum, auto

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
from tf_transformations import euler_from_quaternion, quaternion_matrix


class State(Enum):
    STOP = auto()
    FORWARD = auto()
    TURN_LEFT = auto()
    TURN_RIGHT = auto()


def angle_difference(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))


def transform_matrix(transform):
    rotation = transform.transform.rotation
    translation = transform.transform.translation
    matrix = quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])
    matrix[:3, 3] = [translation.x, translation.y, translation.z]
    return matrix


class Turtlebot(Node):
    # Distances are in meters, angles in radians, and speeds in m/s or rad/s.
    LEFT_MIN = 0.10
    LEFT_MAX = 0.20
    FRONT_STOP = 0.30
    TURN_MIN_CLEARANCE = 0.18
    LEFT_CENTER_ANGLE = (1.47 + 1.62) / 2.0
    LEFT_HALF_ANGLE = (1.62 - 1.47) / 2.0
    FRONT_HALF_ANGLE = 0.20
    FORWARD_SPEED = 0.12
    CORRECTION_SPEED = 0.12
    TURN_SPEED = 0.35
    TURN_TOLERANCE = math.radians(3.0)
    GOAL_TOLERANCE = 0.10
    DATA_TIMEOUT = 5.0
    CONTROL_PERIOD = 0.1

    def __init__(self):
        super().__init__('lab5_controller')
        self.vel_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        self.pose = None
        self.start_pose = None
        self.odom_received_at = None
        self.scan = None
        self.scan_received_at = None
        self.limits_published = False
        self.turn_target = None
        self.state = State.STOP
        self.status = None

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        if not all(math.isfinite(value) for value in [position.x, position.y, *quaternion]):
            self.set_status('Odometry contains non-finite values; stopping.')
            self.odom_received_at = None
            return
        if sum(value * value for value in quaternion) < 1e-12:
            self.set_status('Odometry orientation is invalid; stopping.')
            self.odom_received_at = None
            return
        yaw = euler_from_quaternion(quaternion)[2]
        self.pose = (position.x, position.y, yaw)
        self.odom_received_at = time.monotonic()
        if self.start_pose is None:
            self.start_pose = self.pose
            self.get_logger().info(
                f'Start pose: x={position.x:.2f}, y={position.y:.2f}, yaw={yaw:.2f}')

    def scan_callback(self, msg):
        self.scan = msg
        self.scan_received_at = time.monotonic()

    def set_status(self, message):
        if message != self.status:
            if message:
                self.get_logger().warn(message)
            elif self.status:
                self.get_logger().info('Sensor data and transforms are available.')
            self.status = message

    def publish_limits(self):
        if self.limits_published:
            return True
        try:
            lidar = self.tf_buffer.lookup_transform(
                'base_link', 'rplidar_link', Time())
        except TransformException as exc:
            self.set_status(f'Waiting for base_link to rplidar_link TF: {exc}')
            return False

        transforms = []
        for frame, y in (('left_limit', 0.17), ('right_limit', -0.17)):
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'base_link'
            transform.child_frame_id = frame
            transform.transform.translation.y = y
            transform.transform.translation.z = lidar.transform.translation.z
            transform.transform.rotation.w = 1.0
            transforms.append(transform)
        self.static_broadcaster.sendTransform(transforms)
        self.limits_published = True
        self.get_logger().info('Published left_limit and right_limit static frames.')
        return True

    def scan_distances(self):
        scan = self.scan
        if (scan is None or not scan.header.frame_id or not scan.ranges
                or not math.isfinite(scan.angle_min)
                or not math.isfinite(scan.angle_increment)
                or scan.angle_increment == 0.0
                or not math.isfinite(scan.range_min)
                or not math.isfinite(scan.range_max)
                or scan.range_max <= scan.range_min):
            self.set_status('Invalid or empty LiDAR scan; stopping.')
            return None

        try:
            base_from_scan = transform_matrix(self.tf_buffer.lookup_transform(
                'base_link', scan.header.frame_id, Time()))
            left_from_scan = transform_matrix(self.tf_buffer.lookup_transform(
                'left_limit', scan.header.frame_id, Time()))
        except TransformException as exc:
            self.set_status(f'Waiting for LiDAR transforms: {exc}')
            return None

        left_dist = math.inf
        front_dist = math.inf
        closest_dist = math.inf
        left_observed = False
        front_observed = False
        for index, distance in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            direction = base_from_scan[:3, :3] @ np.array(
                [math.cos(angle), math.sin(angle), 0.0])
            robot_angle = math.atan2(direction[1], direction[0])
            is_left = abs(angle_difference(self.LEFT_CENTER_ANGLE, robot_angle)) <= self.LEFT_HALF_ANGLE
            is_front = abs(angle_difference(0.0, robot_angle)) <= self.FRONT_HALF_ANGLE
            if math.isinf(distance) and distance > 0:
                left_observed |= is_left
                front_observed |= is_front
                continue
            if not math.isfinite(distance) or not scan.range_min <= distance <= scan.range_max:
                continue

            point = np.array([
                distance * math.cos(angle), distance * math.sin(angle), 0.0, 1.0])
            base_point = base_from_scan @ point
            closest_dist = min(closest_dist, math.hypot(base_point[0], base_point[1]))
            if is_left:
                left_observed = True
                left_point = left_from_scan @ point
                left_dist = min(left_dist, math.hypot(left_point[0], left_point[1]))
            if is_front:
                front_observed = True
                front_dist = min(front_dist, math.hypot(base_point[0], base_point[1]))

        if not left_observed or not front_observed:
            self.set_status('LiDAR has no valid readings in a required sector; stopping.')
            return None
        return left_dist, front_dist, closest_dist

    def publish_velocity(self, linear=0.0, angular=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.vel_pub.publish(msg)

    def set_state(self, state):
        if state != self.state:
            self.state = state
            self.get_logger().info(f'State: {state.name}')

    def run(self, meters):
        if not math.isfinite(meters) or meters <= self.GOAL_TOLERANCE:
            raise ValueError('meters must be finite and greater than the goal tolerance')

        next_control = time.monotonic()
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                now = time.monotonic()
                if now < next_control:
                    continue
                next_control = now + self.CONTROL_PERIOD

                if (self.odom_received_at is None or self.scan_received_at is None
                        or now - self.odom_received_at > self.DATA_TIMEOUT
                        or now - self.scan_received_at > self.DATA_TIMEOUT):
                    self.set_status('Waiting for fresh odometry and LiDAR data; stopping.')
                    self.set_state(State.STOP)
                    self.publish_velocity()
                    continue

                x, y, yaw = self.pose
                start_x, start_y, start_yaw = self.start_pose
                progress = ((x - start_x) * math.cos(start_yaw)
                            + (y - start_y) * math.sin(start_yaw))
                if progress >= meters - self.GOAL_TOLERANCE:
                    self.set_state(State.STOP)
                    self.publish_velocity()
                    self.get_logger().info(f'Goal reached: {progress:.2f} m forward.')
                    break

                if not self.publish_limits():
                    self.set_state(State.STOP)
                    self.publish_velocity()
                    continue
                distances = self.scan_distances()
                if distances is None:
                    self.set_state(State.STOP)
                    self.publish_velocity()
                    continue
                left_dist, front_dist, closest_dist = distances

                if self.turn_target is not None:
                    remaining = angle_difference(self.turn_target, yaw)
                    if remaining >= -self.TURN_TOLERANCE:
                        self.turn_target = None
                        self.set_state(State.STOP)
                        self.publish_velocity()
                        continue
                    if closest_dist < self.TURN_MIN_CLEARANCE:
                        self.set_status('Obstacle too close to rotate safely; stopping.')
                        self.set_state(State.STOP)
                        self.publish_velocity()
                        continue
                    self.set_status(None)
                    self.set_state(State.TURN_RIGHT)
                    self.publish_velocity(angular=-min(self.TURN_SPEED, abs(remaining)))
                    continue

                if front_dist < self.FRONT_STOP:
                    self.turn_target = yaw - math.pi / 2.0
                    if closest_dist < self.TURN_MIN_CLEARANCE:
                        self.set_status('Obstacle too close to rotate safely; stopping.')
                        self.set_state(State.STOP)
                    else:
                        self.set_status(None)
                        self.set_state(State.TURN_RIGHT)
                elif left_dist < self.LEFT_MIN:
                    self.set_status(None)
                    self.set_state(State.TURN_RIGHT)
                elif left_dist <= self.LEFT_MAX:
                    self.set_status(None)
                    self.set_state(State.FORWARD)
                else:
                    self.set_status(None)
                    self.set_state(State.TURN_LEFT)

                match self.state:
                    case State.FORWARD:
                        self.publish_velocity(linear=self.FORWARD_SPEED)
                    case State.TURN_LEFT:
                        self.publish_velocity(
                            linear=self.FORWARD_SPEED, angular=self.CORRECTION_SPEED)
                    case State.TURN_RIGHT:
                        if self.turn_target is not None:
                            self.publish_velocity(angular=-self.TURN_SPEED)
                        else:
                            self.publish_velocity(
                                linear=self.FORWARD_SPEED, angular=-self.CORRECTION_SPEED)
                    case State.STOP:
                        self.publish_velocity()
        finally:
            self.publish_velocity()


def main(args=None):
    rclpy.init(args=args)
    node = Turtlebot()
    try:
        node.run(7)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
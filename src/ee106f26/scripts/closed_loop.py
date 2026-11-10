#!/usr/bin/env python3
import threading
import time
from math import pi, sqrt, atan2, cos, sin, radians, degrees, hypot
import numpy as np
from tf_transformations import euler_from_quaternion
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, Pose2D

class Controller:
    def __init__(self, P=0.0, D=0.0, set_point=0):
        self.Kp = P
        self.Kd = D
        self.set_point = set_point # reference (desired value)
        self.previous_error = 0

    def update(self, current_value):
        # calculate P_term and D_term
        error = self.set_point - current_value
        P_term = self.Kp * error
        D_term = self.Kd * (error - self.previous_error)
        self.previous_error = error
        return P_term + D_term

    def setPoint(self, set_point):
        self.set_point = set_point
        self.previous_error = 0

    def setPD(self, P=0.0, D=0.0):
        self.Kp = P
        self.Kd = D


class Turtlebot(Node):

    def __init__(self):
        super().__init__('turtlebot_move')

        self.get_logger().info("Press Ctrl + C to terminate")

        # Create publisher for publishing the velocity commands
        self.vel_pub = self.create_publisher(TwistStamped, "cmd_vel", 10)
        self.pose = Pose2D()
        self.logging_counter = 0
        self.trajectory = list()
        self.odom_sub = self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.controller = Controller()
    

    def run_forward_meters(self, meters):
        self.get_logger().info('Running {} meters forward'.format(meters))

        # Track distance along the robot's current heading rather than a
        # fixed world x/y coordinate.
        heading = self.pose.theta
        start_projection = (self.pose.x * cos(heading) + self.pose.y * sin(heading))
        target_projection = start_projection + meters

        self.controller.setPoint(target_projection)
        self.controller.setPD(P=5, D=5)

        vel = TwistStamped()
        while True:
            projection = (self.pose.x * cos(heading) + self.pose.y * sin(heading))
            if abs(self.controller.set_point - projection) <= 0.01:
                break

            vel.twist.linear.x = self.controller.update(projection)
            vel.header.stamp = self.get_clock().now().to_msg()
            self.vel_pub.publish(vel)
            time.sleep(0.1)

        vel.twist.linear.x = 0.0
        vel.header.stamp = self.get_clock().now().to_msg()
        self.vel_pub.publish(vel)

    def go_to_position(self, target_x, target_y):
        self.get_logger().info(
            'Moving to ({}, {})'.format(target_x, target_y)
        )

        dx = target_x - self.pose.x
        dy = target_y - self.pose.y
        distance = hypot(dx, dy)

        if distance <= 0.01:
            self.get_logger().info('Already at target position')
            return

        target_heading = atan2(dy, dx)
        self.turn_angle(degrees(target_heading))
        self.run_forward_meters(distance)

    def turn_angle(self, angle):
        self.get_logger().info('Turning angle: ' + str(angle))

        target_angle = radians(angle)
        self.controller.setPD(P=5, D=5)

        vel = TwistStamped()
        while True:
            error = atan2(sin(target_angle - self.pose.theta),cos(target_angle - self.pose.theta))
            if abs(degrees(error)) <= 0.01:
                break
            vel.twist.angular.z = self.controller.Kp * error
            vel.header.stamp = self.get_clock().now().to_msg()
            self.vel_pub.publish(vel)
            time.sleep(0.1)

        # Stop explicitly after reaching the target.
        vel.twist.angular.z = 0.0
        vel.header.stamp = self.get_clock().now().to_msg()
        self.vel_pub.publish(vel)

        

    def odom_callback(self, msg):
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(quarternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        # logging once every 6 times (Gazebo runs at 60Hz; we save it at 10Hz)
        self.logging_counter += 1
        if self.logging_counter == 6:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y])  # save trajectory
            # display (x, y, theta) on the terminal
            #self.get_logger().info("odom: x=" + str(self.pose.x) + ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))

def main(args=None):
    """
    Entry point function for the Open Loop control node.

    @param args - Arguments that need to be passed to ROS.
    """

    # Initialize the client library - this is required
    rclpy.init(args=args)

    # Create the publisher node
    turtlebot_node = Turtlebot()

    # Spin so that the node is busy processing
    # while no exceptions are encountered and ROS
    # is executing
    # For this lab - spin rclpy on separate thread
    thread = threading.Thread(target=rclpy.spin, args=(turtlebot_node,), daemon=True)
    thread.start()

    # Allow time for other nodes to start
    time.sleep(2.5)

    try:
        #Navigate to absolute positions around a square.
        turtlebot_node.run_forward_meters(4)
        turtlebot_node.turn_angle(90)

        turtlebot_node.run_forward_meters(4)
        turtlebot_node.turn_angle(180)

        turtlebot_node.run_forward_meters(4)
        turtlebot_node.turn_angle(270)

        turtlebot_node.run_forward_meters(4)
        time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # save trajectory into csv file
        np.savetxt('trajectory.csv', np.array(turtlebot_node.trajectory), fmt='%f', delimiter=',')

        # Destroy the node explicitly
        turtlebot_node.destroy_node()

        # Shut down this context
        rclpy.shutdown()

if __name__ == '__main__':
    main()






#code that didnt work
# def run_to_x_meters(self, meters):
#         """
#         Run function. There are a variety of design patterns.
#         An alternative would have been to use the `timer_callback`
#         design pattern shown in previous labs.
#         """

#         # Create a Twist object
#         # Essentially it's going to publish the velocity
#         vel = TwistStamped()

#         self.get_logger().info('Running {} meters in x'.format(meters))
        
#         #set desired point to 4 meters ahead
#         self.controller.setPoint(meters)  
#         self.controller.setPD(P=1.0, D=0.1)  # set PD gains

#         #while current pose is not at the desired point
#         while abs(self.controller.set_point - self.pose.x) > 0.01: 
#             vel.twist.linear.x = self.controller.update(self.pose.x) 
#             vel.header.stamp = self.get_clock().now().to_msg() 
#             self.vel_pub.publish(vel)
#             time.sleep(0.1)

#     def run_to_y_meters(self, meters):
#         self.get_logger().info('Running {} meters in y'.format(meters))
#         self.controller.setPoint(meters)  
#         self.controller.setPD(P=1.0, D=0.1)  # set PD gains 

#         vel = TwistStamped()
#         while abs(self.controller.set_point - self.pose.y) > 0.01: 
#             vel.twist.linear.x = self.controller.update(self.pose.y) 
#             vel.header.stamp = self.get_clock().now().to_msg() 
#             self.vel_pub.publish(vel)
#             time.sleep(0.1)
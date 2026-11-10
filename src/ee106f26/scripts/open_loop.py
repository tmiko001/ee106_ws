#!/usr/bin/env python3

#source install/local_setup.bash
#ros2 run ee106f26 open_loop.py
#ros2 launch ee106f26 turtlebot.launch.py model:=lite world:=empty



import rclpy
import threading
import time

import numpy as np

from math import pi

from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class Turtlebot(Node):

    def __init__(self):

        # Create and initialize the Node - this is required
        super().__init__('turtlebot_move')

        self.get_logger().info("Press Ctrl + C to terminate")

        # Create publisher for publishing the velocity commands
        self.vel_pub = self.create_publisher(TwistStamped, "cmd_vel", 10)

    def run_forward(self):
        """
        Run function. There are a variety of design patterns.
        An alternative would have been to use the `timer_callback`
        design pattern shown in previous labs.
        """

        # Create a Twist object
        # Essentially it's going to publish the velocity
        vel = TwistStamped()

        # Populate the fields of the message
        vel.twist.linear.x = 1.0 #aparently does nothing as is?
        vel.twist.angular.z = 0.0

        self.get_logger().info('Running open-loop control...')
        
        for _ in range(140):
                self.vel_pub.publish(vel)
                time.sleep(1) #scaled for simulator running at 10% of real time

        vel.twist.linear.x = 0.0
        vel.twist.angular.z = 0.0
        self.vel_pub.publish(vel)
    
    def turn(self):
        vel = TwistStamped()
        vel.twist.linear.x = 0.0
        vel.twist.angular.z = 0.5

        self.get_logger().info('Turning open-loop control...')

        for _ in range(42):
                self.vel_pub.publish(vel)
                time.sleep(0.9)
            
        vel.twist.linear.x = 0.0
        vel.twist.angular.z = 0.0
        self.vel_pub.publish(vel)
        time.sleep(0.5)

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
    time.sleep(5)

    try:
        #turtlebot_node.run_forward()
        turtlebot_node.run_forward()
        turtlebot_node.turn()
        #turtlebot_node.run_forward()
        turtlebot_node.run_forward()
        turtlebot_node.turn()
        #turtlebot_node.run_forward()
        turtlebot_node.run_forward()
        turtlebot_node.turn()
        #turtlebot_node.run_forward()
        turtlebot_node.run_forward()
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    turtlebot_node.destroy_node()

    # Shut down this context
    rclpy.shutdown()


if __name__ == '__main__':
    main()
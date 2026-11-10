#!/usr/bin/env python3

import random

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from ee106f26.msg import EE106LabCustomNew


class Publisher(Node):

    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(EE106LabCustomNew, 'EE106lab_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = EE106LabCustomNew()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.int_data1 = random.randint(0, 100)
        msg.int_data2 = random.randint(0, 100)
        self.publisher_.publish(msg)
        self.get_logger().info(
            'Publishing: %d, %d' % (msg.int_data1, msg.int_data2))


def main(args=None):
    rclpy.init(args=args)

    publisher = Publisher()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from ee106f26.msg import EE106LabCustomNew


class Subscriber(Node):

    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            EE106LabCustomNew,
            'EE106lab_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(
            '%d + %d = %d, Timestamp: %d.%09d s' % (
                msg.int_data1,
                msg.int_data2,
                msg.int_data1 + msg.int_data2,
                msg.header.stamp.sec,
                msg.header.stamp.nanosec))


def main(args=None):
    rclpy.init(args=args)

    subscriber = Subscriber()

    rclpy.spin(subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from tf_transformations import euler_from_quaternion, quaternion_matrix


class TfListener(Node):

    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        try:
            # capture the latest available tf of the two frames (Time() means "latest")
            t = self.tf_buffer.lookup_transform('base_link', 'turtlebot4/rplidar_link/rplidar', Time())
        except TransformException as ex:
            self.get_logger().info('Could not get transform: %s' % ex)
            return

        trans = t.transform.translation
        rot = t.transform.rotation

        # print of the Translation and Rotation information, by demonstrating the Quaternion, Euler, and Rotation Matrix representation of the latter.
        print("The translation is (x,y,z) = " + str((trans.x, trans.y, trans.z)))
        print("The rotation (quaternion) is (x,y,z,w) = " + str((rot.x, rot.y, rot.z, rot.w)))
        print("The rotation (euler) is (r,p,y) = " + str(euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])))
        rot_mat = quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
        print("The rotation (rotation matrix) is = " + str(rot_mat))

        # we assume that a Lidar point is detected, w.r.t the Lidar's frame
        laser_point_detected = [1, 0, 0, 1]

        # initialization of the tf matrix to describe it in the base_link frame
        rot_mat[0, 3] = trans.x
        rot_mat[1, 3] = trans.y
        rot_mat[2, 3] = trans.z
        print(np.dot(rot_mat, laser_point_detected))


def main(args=None):
    rclpy.init(args=args)

    tf_listener = TfListener()

    rclpy.spin(tf_listener)

    tf_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
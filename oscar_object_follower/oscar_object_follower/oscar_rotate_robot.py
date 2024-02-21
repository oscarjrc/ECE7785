#!/usr/bin/env python
#Oscar Jed Chuy

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist, Vector3

import sys
import numpy as np
import cv2
from cv_bridge import CvBridge

class RotateRobot(Node):
    def __init__(self):
        # Creates the node.
        super().__init__('rotate_robot')

        self._twist_publish = self.create_publisher(Twist,'/cmd_vel',10)

        self._point_subscriber = self.create_subscription(
                Point,
                '/obj_loc',
                self._loc_callback,
                10)
        self._point_subscriber # Prevents unused variable warning.

    def _loc_callback(self, pt):
        msg = Twist()
        vec = Vector3()

        # print(pt.x)
        x = pt.x
        kp = 0.01

        error = 160 - x
        control = round(error * kp,2)

        msg.angular.z = control
        #vec.z = control
        #msg.angular = vec
        self._twist_publish.publish(msg)

        # print(control)

def main(args = None):
    rclpy.init() #init routine needed for ROS2.
    rotate_robot = RotateRobot() #Create class object to be used.
    rclpy.spin(rotate_robot)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rotate_robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python
#Oscar Jed Chuy and Jack Curtis

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose
import math


import numpy as np

class GetObjectRange(Node):
    def __init__(self):
        super().__init__('get_object_range')

        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=1)
        qos_policy2 = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            depth=1)
        
        self._laser_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self._laser_callback,
            qos_policy2)
        
        self._dir_publish = self.create_publisher(PoseArray,'/obj_loc',qos_policy)

    def _laser_callback(self, LaserScan):
        msg2 = PoseArray()

        angl = LaserScan.angle_increment
        rng = LaserScan.ranges
        rng2 = np.zeros(len(rng))

        bound = 1 #meters

        for i in range(0,len(rng)-1):
            if rng[i] <= bound:
                _msg2 = Pose()
                _msg2.orientation.z = float(i)
                _msg2.position.z = float(len(rng))
                _msg2.position.x = float(rng[i])
                msg2.poses.append(_msg2)


        # print(msg2)
        # print(angl * 180 / math.pi)
        self._dir_publish.publish(msg2)
        # print('---------------------------------------')


def main(args = None):
    rclpy.init() #init routine needed for ROS2.
    get_object_range = GetObjectRange() #Create class object to be used.
    rclpy.spin(get_object_range)

    #Clean up and shutdown.
    get_object_range.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
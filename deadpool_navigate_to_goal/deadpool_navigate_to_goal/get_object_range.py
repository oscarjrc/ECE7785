#!/usr/bin/env python
#Oscar Jed Chuy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose


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

        size = int(((360/360) * len(rng)) / 2)
        # print(len(rng))

        # rng_left = np.flip(rng[0:(size)])
        # rng_right = np.flip(rng[(len(rng)-(size)):len(rng)])
        # rng = np.append(rng_left,rng_right)

        for i in range(1,len(rng)-1):
            # if(abs(rng[i-1] - rng[i+1]) >= 1.0) and ((i < 60) or (i > 300)):
            if(abs(rng[i-1] - rng[i+1]) >= 1.0):
                _msg2 = Pose()
                rng2[i] = rng[i]

                _msg2.orientation.z = float(i)
                _msg2.position.z = float(len(rng))
                _msg2.position.x = float(rng[i])
                msg2.poses.append(_msg2)
            else:
                rng2[i] = 0
            if rng2[i] > LaserScan.range_max:
                rng2[i] = 0

        # print(rng2)
        self._dir_publish.publish(msg2)


def main(args = None):
    rclpy.init() #init routine needed for ROS2.
    get_object_range = GetObjectRange() #Create class object to be used.
    rclpy.spin(get_object_range)

    #Clean up and shutdown.
    get_object_range.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python
#Oscar Jed Chuy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Point, Twist

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

        self._obj_subscriber = self.create_subscription(
            Point,
            '/obj_loc',
            self._obj_callback,
            10)
        
        self._laser_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self._laser_callback,
            qos_policy2)
        
        self._dir_publish = self.create_publisher(Twist,'/obj_rng',qos_policy)


    def _obj_callback(self, Point):
        self.point = Point

    def _laser_callback(self, LaserScan):
        msg = Twist()

        angl = LaserScan.angle_increment
        rng = LaserScan.ranges

        size = int(((60/360) * len(rng)) / 2)
        # print(size)

        rng_left = np.flip(rng[0:(size)])
        rng_right = np.flip(rng[(len(rng)-(size)):len(rng)])
        rng = np.append(rng_left,rng_right)
        for i in range(0,len(rng)):
            if rng[i] > LaserScan.range_max:
                rng[i] = 0.2
            elif rng[i] < LaserScan.range_min:
                rng[i] = 0.2

        # print(str(rng) + '\n')
        # print(pt)
        # print(len(rng))
        pt = self.point.x

        loc = round(pt / 320 * len(rng))
        # print(loc)
        dist = rng[loc]

        angle = -(loc - size) * angl
        
        msg.linear.x = float(round(dist,2))
        msg.angular.z = float(round(angle,2))

        if self.point.z != float(0):
            msg.linear.x = 0.45 #match with ref in chase_object

        self._dir_publish.publish(msg)
        # print(angle)

def main(args = None):
    rclpy.init() #init routine needed for ROS2.
    get_object_range = GetObjectRange() #Create class object to be used.
    rclpy.spin(get_object_range)

    #Clean up and shutdown.
    get_object_range.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
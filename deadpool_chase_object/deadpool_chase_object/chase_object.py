#!/usr/bin/env python
#Oscar Jed Chuy

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import numpy as np

class ChaseObject(Node):
    def __init__(self):
        super().__init__('chase_object')

        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=1)

        self._obj_subscriber = self.create_subscription(
            Twist,
            '/obj_rng',
            self._dir_callback,
            qos_policy)
        
        self._vel_publish = self.create_publisher(Twist,'/cmd_vel',10)


    def _dir_callback(self, twist):
        msg = Twist()

        dist = twist.linear.x   #meters
        angl = twist.angular.z  #rad

        kp_d = 0.3
        kp_a = 0.09

        ref = 0.45                       #meters
        dist_e = ref - dist
        angl_e = angl * (180 / np.pi)   #deg

        dist_u = -round(dist_e * kp_d,2)
        angl_u = round(angl_e * kp_a,2)

        if dist_u >= 0.1:
            dist_u = 0.1
        if angl_u >= 0.15:
            angl_u = 0.15
        
        msg.linear.x = dist_u
        msg.angular.z = angl_u
        self._vel_publish.publish(msg)

        # print(msg)


def main(args = None):
    rclpy.init() #init routine needed for ROS2.
    chase_object = ChaseObject() #Create class object to be used.
    rclpy.spin(chase_object)

    #Clean up and shutdown.
    chase_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
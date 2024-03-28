#!/usr/bin/env python
#Oscar Jed Chuy and Jack Curtis

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
import math

import numpy as np

class NavGoal(Node):

    def __init__(self):
        super().__init__('nav_goal')
        
        self._vel_publish = self.create_publisher(PoseStamped,'/goal_pose',10)

        self._imu_subscriber = self.create_subscription(
            Imu,
            '/imu',
            self._imu_callback,
            198)

        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def _imu_callback(self, imu):
        self.headertime = imu.header.stamp

    def timer_callback(self):
        msg = PoseStamped()

        goalpoint = [2.82,-0.77,0.0]
        goalpointori = self.euler_to_quaternion(0, 0, 90 * math.pi/180)

        # print(self.headertime)

        msg.header.frame_id = "map"
        msg.header.stamp = self.headertime
        msg.pose.position.x = float(goalpoint[0])
        msg.pose.position.y = float(goalpoint[1])
        msg.pose.position.z = float(goalpoint[2])
        msg.pose.orientation.x = float(goalpointori[0])
        msg.pose.orientation.y = float(goalpointori[1])
        msg.pose.orientation.z = float(goalpointori[2])
        msg.pose.orientation.w = float(goalpointori[3])

        self._vel_publish.publish(msg)

    def euler_to_quaternion(self, yaw, pitch, roll):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

def main(args = None):
    rclpy.init() #init routine needed for ROS2.
    nav_goal = NavGoal() #Create class object to be used.
    rclpy.spin(nav_goal)

    #Clean up and shutdown.
    nav_goal.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
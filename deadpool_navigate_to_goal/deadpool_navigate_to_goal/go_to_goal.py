#!/usr/bin/env python
#Oscar Jed Chuy

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose, Twist, Quaternion
import math

import numpy as np

class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')

        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=1)
        qos_policy2 = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            depth=1)
        
        self._obj_subscriber = self.create_subscription(
            PoseArray,
            '/obj_loc',
            self._object_callback,
            qos_policy)
        
        self._odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self._odometry_callback,
            qos_policy)
        
        self._vel_publish = self.create_publisher(Twist,'/cmd_vel',10)

    def _odometry_callback(self, Odometry): #Global
        data = Quaternion()

        self.odom = Odometry
        self.position = self.odom.pose.pose.position
        self.orientation = self.odom.pose.pose.orientation

        rpy = self.euler_from_quaternion(self.orientation)

        self.yaw = rpy[2]
        self.yaw = self.yaw*180/np.pi #deg
        # print(self.position)
        # print(self.yaw)
        # print('---------------------------------------------')
        # print('Test1')

    def _object_callback(self, PoseArray):  #Local
        self.posearray = PoseArray.poses
        msg = Twist()
        
        self.rays = int(self.posearray[1].position.z)
        dis = np.array([])
        ori = np.array([])
        for i in range(0,len(self.posearray)):
            dis = np.append(dis,self.posearray[i].position.x)
            ori = np.append(ori,self.posearray[i].orientation.z)

        # print(dis)
        # print('---------------------------------------------')
        # print(ori)
        # print('---------------------------------------------')
        # print(self.rays)

        # self.local_to_globalobj(dis,ori)
        
        #Go to Goal w/o objects
        kp_d = 0.3
        kp_a = 0.1

        ref = np.array([1., 0.])                    #meters
        loc = np.array([float(self.position.x),float(self.position.y)])
        dif = ref - loc
        dist_e = np.linalg.norm(dif)
        # print('num: ' + str(ref[0] - loc[0]) + ' den: ' + str(dist_e))
        # print((ref[0] - loc[0])/dist_e)
        angl = ( math.acos((ref[0] - loc[0])/(dist_e)) * 180 / math.pi) - self.yaw #deg

        angl_e = angl * (180 / np.pi)   #deg

        print(angl)
        # print(loc)
        # print(dif)
        # print(dist_e)
        print('---------------------------------------------')
        
    
    def local_to_globalobj(self,distance,orientation):
        for i in range(len(distance)):
            print('Dist: ' + str(distance[i]) + ' Orient: ' + str(orientation[i]))
        print('---------------------------------------------')
            
    def euler_from_quaternion(self,quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return [roll, pitch, yaw]


def main(args = None):
    rclpy.init() #init routine needed for ROS2.
    go_to_goal = GoToGoal() #Create class object to be used.
    rclpy.spin(go_to_goal)

    #Clean up and shutdown.
    go_to_goal.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
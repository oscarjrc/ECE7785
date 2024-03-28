#!/usr/bin/env python
#Oscar Jed Chuy and Jack Curtis

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose, Twist, Quaternion, Point
import math

import numpy as np

class GoToGoal(Node):
    Init = True

    state = 0   #0 to 2
    waypoint = 0    #0: looking for waypoint, 1:moving to waypoint
    ref_new = np.array([])

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
        
        self.odom = Odometry
        self.position = self.odom.pose.pose.position
        self.orientation = self.odom.pose.pose.orientation

        rpy = self.euler_from_quaternion(self.orientation)

        self.yaw = rpy[2]
        self.yaw = self.yaw*180/np.pi #deg

        # print('Position: ' + str(self.position))
        # print((self.position.x == 0.0))

        if GoToGoal.Init:
            self.Init_pos = Point()

            if ((self.position.x != 0.0) and (self.position.y != 0.0)):
                GoToGoal.Init = False
            # print('In intit')
            #The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
            self.Init_ang = self.yaw    #deg
            self.Init_pos.x = self.position.x
            self.Init_pos.y = self.position.y
            # print('Position: ' + str(self.position))
            # print('Init Pos: ' + str(self.Init_pos))
            # print('Init Agl: ' + str(self.Init_ang))

        # Resets current pos and ori to (0,0) & 0 deg
        self.position.x = self.position.x - self.Init_pos.x
        self.position.y = self.position.y - self.Init_pos.y
        self.yaw = self.yaw - self.Init_ang

        # print('State: ' + str(GoToGoal.Init))
        # print('Init Pos: ' + str(self.Init_pos))
        # print('Init Agl: ' + str(self.Init_ang))
        # print('Position: ' + str(self.position))
        # print('Angle: ' + str(self.yaw))
        # print('---------------------------------------------')

    def _object_callback(self, PoseArray):  #Local
        self.posearray = PoseArray.poses
        if len(self.posearray) != 0:
            self.rays = int(self.posearray[0].position.z)
        else:
            self.rays = 0

        self.dis = np.array([])
        self.ori = np.array([])
        for i in range(0,len(self.posearray)):
            self.dis = np.append(self.dis,self.posearray[i].position.x)
            self.ori = np.append(self.ori,self.posearray[i].orientation.z)
        
        self.gotogoal()
            
        # print(self.dis)
        # print('---------------------------------------------')
        # print(self.ori)
        # print('---------------------------------------------')
        # print(self.rays)
    
    def gotogoal(self):
        msg = Twist()

        #Go to Goal w/o objects
        kp_d = 0.5
        kp_a = 0.2

        self.loc = np.array([float(self.position.x),float(self.position.y)])
        if GoToGoal.waypoint == 0:
            self.ref = np.array([[1.5, 0.0],[1.5, 1.4],[0.0, 1.4]])                    #meters
            dif = self.ref[GoToGoal.state] - self.loc
            self.dist_e = np.linalg.norm(dif)

            # angl = ( math.acos((self.ref[GoToGoal.state][0] - self.loc[0])/(self.dist_e)) * 180 / math.pi) - self.yaw #deg
            angl = math.atan2(dif[1],dif[0]) * 180 / math.pi - self.yaw       #deg
        else:
            # print('In waypoint')
            dif = GoToGoal.ref_new - self.loc
            self.dist_e = np.linalg.norm(dif)

            # angl = (( math.acos((GoToGoal.ref_new[0] - self.loc[0])/(self.dist_e)) * 180 / math.pi) - self.yaw) #deg
            angl = math.atan2(dif[1],dif[0]) * 180 / math.pi - self.yaw       #deg
        
        angl_e = angl  #deg
        if angl_e > 180:
            angl_e = angl_e - 360
        if angl_e < -180:
            angl_e = angl_e + 360
        
        self.dist_u = round(self.dist_e * kp_d,2)
        self.angl_u = round(angl_e * kp_a,2)

        if abs(angl_e) >= 5.0:
            self.dist_u = 0.0

        if self.dist_u >= 0.09:
            self.dist_u = 0.09
        elif self.dist_u <= -0.09:
            self.angl_u = -0.09

        if self.angl_u >= 0.2:
            self.angl_u = 0.2
        elif self.angl_u <= -0.2:
            self.angl_u = -0.2

        if (self.dist_e < 0.01) and (GoToGoal.waypoint != 1):
            GoToGoal.state += 1
            if GoToGoal.state > 2:
                GoToGoal.state = 2
        elif (self.dist_e < 0.01) and (GoToGoal.waypoint == 1):
            GoToGoal.waypoint = 0
        
        msg.linear.x = self.dist_u
        msg.angular.z = self.angl_u
        self._vel_publish.publish(msg)

        print('State: ' + str(GoToGoal.state))
        print('dif: ' + str(dif))
        print('theta_red: ' + str(self.yaw) + ' theta_blue: ' + str(angl_e + self.yaw))
        print('loc: ' + str(self.loc))
        print('error_d: ' + str(self.dist_e) + ' error_a: ' + str(angl_e) + ' angle_: ' + str(angl))
        print('lin_vel: ' + str(self.dist_u) + ' ang_vel: ' + str(self.angl_u))
        print('ref_new: ' + str(GoToGoal.ref_new))
        print('Waypoint: ' + str(GoToGoal.waypoint))
        print('---------------------------------------------')

        if GoToGoal.waypoint == 0:
            self.pickobj()

    def pickobj(self):
        boundary = 0.4  #0.4

        distance = self.dis
        orientation = self.ori
        obj = np.array([0,0])
        for i in range(len(distance)):
            if distance[i] < boundary: #meters
                obj = np.vstack((obj,np.array([distance[i], orientation[i]])))
            # print('Dist: ' + str(distance[i]) + ' Orient: ' + str(orientation[i]))
        
        # print('Objects: ' + str(obj))
                
        print(len(obj))
        if (len(obj) != 2) and (len(obj) != 3) and (len(obj) != 4) and (len(obj) != 5) and (len(obj) != 6) and (len(obj) != 7):
            # If too many points, pick midlle ones --------------------------------------------
            hlf = math.floor(len(obj)/2)
            obj = np.vstack((obj[hlf-3],obj[hlf+3]))
            # print('Updated Objects: ' + str(obj))
        
            #for two point case:
            obj_G = self.LtoG(obj)
            self.vect_goal = np.array([self.ref[GoToGoal.state][0] - self.loc[0], self.ref[GoToGoal.state][1] - self.loc[1]])
        
            self.vect_obj = np.array([0,0])
            pickpt = np.array([])
            obj_dis = np.array([])
            direction_ = np.array([0,0])
            for i in range(1,len(obj_G)):
                direction = [obj_G[i][0] - self.loc[0],obj_G[i][1]] - self.loc[1]
                # print('Direction ' + str(i) + ": "+str(direction))
                direction_ = np.vstack((direction_,direction))
                
                self.vect_obj = np.vstack((self.vect_obj,direction))
                obj_dis = np.append(obj_dis,np.linalg.norm(direction))
                # print(obj_dis)
                chs_ang = np.arccos(np.dot(self.vect_goal,direction) / (obj_dis[i-1] * self.dist_e)) * 180 / math.pi #deg
                pickpt = np.append(pickpt,chs_ang)

                # print('Dotproduct: ' + str(np.dot(self.vect_goal,direction) ))
            
            # print(obj_dis)
            print(pickpt)

            # Project a new point along direction
            epsilon = 0.7   # new distance
            if pickpt[0] < pickpt[1]:
            # if True:
                # print('L2R')
                edg = obj_G[1]
                # pt2pt = [obj_G[1][0] - obj_G[2][0],obj_G[1][1]] - obj_G[2][1]   #L2R
                pt2pt = obj_G[1]-obj_G[2]
                pt2pt_unit = pt2pt / np.linalg.norm(pt2pt)
                new_point = pt2pt + (epsilon*pt2pt_unit) + obj_G[2]
            else:
                print('R2L')
                edg = obj_G[2]
                # pt2pt = [obj_G[2][0] - obj_G[1][0],obj_G[2][1]] - obj_G[1][1]   #R2L
                pt2pt = obj_G[2]-obj_G[1]
                pt2pt_unit = pt2pt / np.linalg.norm(pt2pt)
                new_point = pt2pt + (epsilon*pt2pt_unit) + obj_G[1]
                
            GoToGoal.ref_new = new_point
            GoToGoal.waypoint = 1

            print('New pt Direction: ' + str(pt2pt))
            print('New pt Unit Direction: ' + str(pt2pt_unit))
            # print('Vectors ' + str(direction_))
            # print('Object Distance: ' + str(obj_dis))
            # print('Vector Angles: ' + str(pickpt))
            # print('Close to object: \n' + str(obj_G))
            # print('Choosen edge: ' + str(edg))
            print('New Point: ' + str(new_point))
            # print('Goal Vector: ' + str(self.vect_goal))
            # print('Object Vector: \n' + str(self.vect_obj))
            print('---------------------------------------------')
        

    def LtoG(self,data):
        new = np.array([0,0])
        for i in range(0,len(data)):
            theta_i = data[i][1] * 360 / self.rays
            x_G = self.loc[0] + (data[i][0] * math.cos((self.yaw + theta_i) * math.pi / 180))
            y_G = self.loc[1] + (data[i][0] * math.sin((self.yaw + theta_i) * math.pi / 180))
            
            new = np.vstack((new,[x_G,y_G]))
            # print('x_G: ' + str(x_G) + ' y_G: ' + str(y_G))
        # print('---------------------------------------------')
            
        return new

            
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
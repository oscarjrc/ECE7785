#!/usr/bin/env python
#Oscar Jed Chuy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Point

import sys
import numpy as np
import cv2
from cv_bridge import CvBridge


class DetectObject(Node):
    def __init__(self):
        # Creates the node.
        super().__init__('detect_object')

        # Set Parameters
        self.declare_parameter('show_image_bool', True)
        self.declare_parameter('window_name', "Raw Image")

        #Determine Window Showing Based on Input
        self._display_image = bool(self.get_parameter('show_image_bool').value)

        # Declare some variables
        self._titleOriginal = self.get_parameter('window_name').value # Image Window Title

        #Set up QoS Profiles for passing images over WiFi
        image_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            depth=1
        )
        
        self._point_publish = self.create_publisher(Point,'/obj_loc',10)
        self._vid_publish = self.create_publisher(CompressedImage,'/overlay',10)

        #Declare that the minimal_video_subscriber node is subcribing to the /camera/image/compressed topic.
        self._video_subscriber = self.create_subscription(
                CompressedImage,
                # '/simulated_camera/image_raw/compressed',
                '/image_raw/compressed',
                self._image_callback,
                image_qos_profile)
        self._video_subscriber # Prevents unused variable warning.

        self.cntr = 160 #define

    def _image_callback(self, CompressedImage):
        # The "CompressedImage" is transformed to a color image in BGR space and is store in "_imgBGR"
        self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage,
        "bgr8")
        
        self.trackblue(self._imgBGR)


    def trackblue(self,img):
        msg = Point()
        msg2 = CompressedImage()

        self.hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        self.obj = cv2.inRange(self.hsv, (100, 25, 25), (111, 255,255))   #Blue
        # self.obj = cv2.inRange(self.hsv, (50, 25, 25), (60, 255,255))     #Green

        self.obj = cv2.medianBlur(self.obj,5)

        kernel = np.ones((7,7),np.uint8)
        self.closing = cv2.morphologyEx(self.obj, cv2.MORPH_CLOSE, kernel)
        self.opening = cv2.morphologyEx(self.closing, cv2.MORPH_OPEN, kernel)
        self.opening = cv2.morphologyEx(self.opening, cv2.MORPH_OPEN, kernel)

        rows = self.opening.shape[0]
        circles = cv2.HoughCircles(self.opening,cv2.HOUGH_GRADIENT,1,rows/2,
                            param1=80,param2=15,minRadius=2,maxRadius=0)
        # print(circles)

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                center = (i[0], i[1])
                # circle center
                cv2.circle(self._imgBGR, center, 1, (0, 100, 100), 3)
                # print(center[1])
                self.cntr = center[0]
                # circle outline
                radius = i[2]
                cv2.circle(self._imgBGR, center, radius, (255, 0, 255), 3)

                msg.x = float(self.cntr)
        else:
            msg.x = float(160)
            msg.z = float(160)
        
        msg2 = CvBridge().cv2_to_compressed_imgmsg(self._imgBGR)

        self._point_publish.publish(msg)
        self._vid_publish.publish(msg2)

def main(args = None):
    rclpy.init() #init routine needed for ROS2.
    detect_object = DetectObject() #Create class object to be used.
    rclpy.spin(detect_object)

    #Clean up and shutdown.
    detect_object.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
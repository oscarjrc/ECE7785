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


class ViewImage(Node):
    def __init__(self):
        # Creates the node.
        super().__init__('view_image')

        # Set Parameters
        self.declare_parameter('show_image_bool', True)
        self.declare_parameter('window_name', "Raw Image")

        #Determine Window Showing Based on Input
        self._display_image = bool(self.get_parameter('show_image_bool').value)

        # Declare some variables
        self._titleOriginal = self.get_parameter('window_name').value # Image Window Title

        #Only create image frames if we are not running headless (_display_image sets this)
        if(self._display_image):
        # Set Up Image Viewing
           cv2.namedWindow(self._titleOriginal, cv2.WINDOW_AUTOSIZE ) #Viewing Window
           cv2.moveWindow(self._titleOriginal, 50, 50) # Viewing Window Original Location

        #Set up QoS Profiles for passing images over WiFi
        image_qos_profile = QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=1)

        #Declare that the minimal_view_image node is subcribing to the /camera/image/compressed topic.
        self._view_image = self.create_subscription(
                CompressedImage,
                '/overlay',
                #'/simulated_camera/image_raw/compressed',
                self._image_callback,
                image_qos_profile)
        self._view_image # Prevents unused variable warning.

    def _image_callback(self, CompressedImage):
        # The "CompressedImage" is transformed to a color image in BGR space and is store in "_imgBGR"
        self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage,
        "bgr8")
        
        if(self._display_image):
        # Display the image in a window
           self.show_image(self._imgBGR)

    def get_image(self):
       return self._imgBGR

    def show_image(self, img):
       cv2.imshow(self._titleOriginal, img)    
       # Cause a slight delay so image is displayed
       self._user_input=cv2.waitKey(50) #Use OpenCV keystroke grabber for delay.

    def get_user_input(self):
       return self._user_input

def main(args = None):
    rclpy.init() #init routine needed for ROS2.
    view_image = ViewImage() #Create class object to be used.

    while rclpy.ok():
        rclpy.spin_once(view_image) # Trigger callback processing.
        if(view_image._display_image):
            view_image.show_image(view_image.get_image())		
            if view_image.get_user_input() == ord('q'):
                cv2.destroyAllWindows()
                break

    #Clean up and shutdown.
    view_image.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
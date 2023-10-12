#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Bool, Int32
from cv_bridge import CvBridge, CvBridgeError
from vision.msg import objectDetection, objectDetectionArray
from vision.srv import DetectColorPattern, DetectColorPatternResponse
import pathlib
from geometry_msgs.msg import Point, PoseArray, Pose
import actionlib

import sys
sys.path.append(str(pathlib.Path(__file__).parent) + '/../include')
from vision_utils import *

#IMAGE_SUB = '/zedImageFiltered'
IMAGE_SUB = '/zed2/zed_node/rgb/image_rect_color'


class DetectorColores:
    def __init__(self):
        rospy.init_node('detector_colores', anonymous=True)

        self.boxes = []
        self.detections = []

        self.color_detections = []
        self.aruco_detections = []
        self.letter_detections = []

        self.image = np.array([])
        self.camera_info = CameraInfo()

        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/vision_out', Image, queue_size=10)

        #Suscriber topics changed for simulation
        
        self.sub = rospy.Subscriber(IMAGE_SUB, Image, self.callback)
        self.subscriberInfo = rospy.Subscriber("/zed2/zed_node/depth/camera_info", CameraInfo, self.infoImageRosCallback)

        rospy.Subscriber("/vision/color_detect", objectDetectionArray, self.color_cb)
        rospy.Subscriber("/vision/aruco_detect", objectDetectionArray, self.aruco_cb)
        rospy.Subscriber("/vision/letter_detect", objectDetectionArray, self.letter_cb)

        """
        self.sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback)
        self.subscriberDepth = rospy.Subscriber("/camera/depth/image_raw", Image, self.depthImageRosCallback)
        self.subscriberInfo = rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.infoImageRosCallback)
        """
        

        # server for detecting color pattern
        
        rospy.loginfo("Subscribed to image")
        self.main()

    def color_cb(self, data):
        self.color_detections = data.detections

    def aruco_cb(self, data):
        self.aruco_detections = data.detections

    def letter_cb(self, data):
        self.letter_detections = data.detections
    

    # Function to handle a ROS camera info input.
    def infoImageRosCallback(self, data):
        self.camera_info = data
        self.subscriberInfo.unregister()

    def draw_detections(self):
        
        #draw all current detections on image
        for d in self.color_detections:
            self.image = cv2.rectangle(self.image, (d.xmin, d.ymin), (d.xmax, d.ymax), (0, 255, 0), 2)
        
        for d in self.aruco_detections:
            self.image = cv2.rectangle(self.image, (d.xmin, d.ymin), (d.xmax, d.ymax), (0, 0, 255), 2)
        
        for d in self.letter_detections:
            self.image = cv2.rectangle(self.image, (d.xmin, d.ymin), (d.xmax, d.ymax), (255, 0, 0), 2)
    

    def callback(self, data):
        # rospy.loginfo(data.data)
        # implement cv_bridge
        self.image = self.bridge.imgmsg_to_cv2(data,desired_encoding="bgr8") 
        self.draw_detections()
        self.pub.publish(self.bridge.cv2_to_imgmsg(self.image, encoding="bgr8"))

        self.boxes = []
        self.detections = []
        
    def main(self):
        rospy.logwarn("Starting listener")
        rate = rospy.Rate(10)
        try:
            while not rospy.is_shutdown():
               
                #self.pubData.publish(self.dtections)
                rate.sleep()
                cv2.waitKey(1)
        except KeyboardInterrupt:
            rospy.logwarn("Keyboard interrupt detected, stopping listener")

if __name__ == '__main__':
    DetectorColores()
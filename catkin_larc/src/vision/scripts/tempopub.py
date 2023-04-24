#!/usr/bin/env python3
import cv2
import numpy as np
import matplotlib.pyplot as plt
import pathlib
import sys
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo


class tempo():
    def __init__(self):
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/object', Image, queue_size=10)
        rospy.init_node('posting', anonymous=True)
        while not rospy.is_shutdown():

            img = "/home/jabv/Desktop/LARC-2023/Vision/A.png"
            #change image to a numpy array
            img = cv2.imread(img)

            self.pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))

if __name__ == '__main__':
    rospy.init_node('posting', anonymous=True)
    try:
        rospy.loginfo("Subscribed to image")
        tempo()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import math
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Int32
from cv_bridge import CvBridge, CvBridgeError
from vision.msg import objectDetection, objectDetectionArray
import pathlib
from geometry_msgs.msg import Point, PoseArray, Pose

import sys
sys.path.append(str(pathlib.Path(__file__).parent) + '/../include')
from vision_utils import *

class LineDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/lines_out', Image, queue_size=10)
        self.pubLines = rospy.Publisher('/number_of_lines', Int32, queue_size=10)

        #Suscriber topics changed for simulation
        #self.sub = rospy.Subscriber('/zed2/zed_node/rgb/image_rect_color', Image, self.callback)
        #self.subscriberDepth = rospy.Subscriber("/zed2/zed_node/depth/depth_registered", Image, self.depthImageRosCallback)
        #self.subscriberInfo = rospy.Subscriber("/zed2/zed_node/depth/camera_info", CameraInfo, self.infoImageRosCallback)

        self.sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback)
        
        self.mask  = None
        self.cv_image = np.array([])
        rospy.loginfo("Subscribed to image")
        self.main()

    def callback(self,data):
        self.cv_image = self.bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")
        self.detect_lines()
        self.pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, encoding="bgr8"))

    def detect_lines(self):    
        frame = self.cv_image
        lowMask = np.array([97,11,4],np.uint8)
        upperMask = np.array([179,84,152],np.uint8)

        img = frame
        h,w = img.shape[0:2]
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,lowMask,upperMask)
        #bitwise and mask and original image
        res = cv2.bitwise_and(img,img,mask=mask)
        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        gray = cv2.dilate(mask,None,iterations=1)
        _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

        kernel = np.ones((5,5), np.uint8)
        thresh = cv2.dilate(thresh, kernel, iterations=2)

        lines = cv2.HoughLines(thresh, 1, np.pi/20, 700)

        existent_pt1 = []
        existent_pt2 = []

        horizontal_lines = []
        for line in lines:
            rho, theta = line[0]
            a = math.cos(theta)
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
            pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))

            if abs(theta - np.pi/2) < 0.05:
                valid = True
                for ePt1 in existent_pt1:
                    if abs(ePt1[0] - pt1[0]) < 50 and abs(ePt1[1] - pt1[1]) < 50:
                        valid = False
                for ePt2 in existent_pt2:
                    if abs(ePt2[0] - pt2[0]) < 50 and abs(ePt2[1] - pt2[1]) < 50:
                        valid = False
                if valid:
                    horizontal_lines.append(line)
                    existent_pt1.append(pt1)
                    existent_pt2.append(pt2)
                    cv2.line(frame, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)

        # Count the number of horizontal lines
        num_lines = len(horizontal_lines)
        print("Number of horizontal lines:", num_lines)

        self.pubLines.publish(Int32(num_lines))    
    
    def main(self):
        rospy.logwarn("Starting listener")
        rospy.init_node('line_detector', anonymous=True)
        rate = rospy.Rate(10)
        try:
            while not rospy.is_shutdown():
                rate.sleep()
                cv2.waitKey(1)
        except KeyboardInterrupt:
            rospy.logwarn("Keyboard interrupt detected, stopping listener")

if __name__ == '__main__':
    LineDetector()
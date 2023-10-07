#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os 
import numpy as np
from time import sleep
from PIL import Image as pilimage
from PIL import ImageEnhance

class zedFilter:

    def __init__(self) -> None:
        self.camSubscriber = '/zed2/zed_node/rgb/image_rect_color'
        self.camPublisher = '/zedImageFiltered'
        #camSubscriber = 'image'
        rospy.init_node('image_viewer')
        rospy.loginfo('image received')
        rospy.Subscriber(self.camSubscriber, Image, self.process_image, queue_size=1)
        self.camPub = rospy.Publisher(self.camPublisher, Image, queue_size=1)
        while not rospy.is_shutdown():
            rospy.Rate(60.0).sleep()
        pass

    def process_image(self, msg):
        try:
            bridge = CvBridge()
            orig = bridge.imgmsg_to_cv2(msg, "rgb8")
           #print(f"read image")
            img =cv2.cvtColor(orig, cv2.COLOR_RGB2BGR)
            #increase brightness and saturation reduce contrast
            #change to PIL
            img = pilimage.fromarray(img)
            img = ImageEnhance.Brightness(img).enhance(1.5)
            img = ImageEnhance.Contrast(img).enhance(0.8)
            img = ImageEnhance.Color(img).enhance(2.5)
            img = np.array(img)
            img =cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            imgMsg = bridge.cv2_to_imgmsg(img, "rgb8")
            self.camPub.publish(imgMsg)
        except Exception as e:
            print(e)

    def show_image(self, img):
        global state, count
        cv2.imshow('image', img)
        
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        zedFilter()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DetectorAruco:
    def __init__(self):
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/aruco_out', Image, queue_size=10)
        self.sub = rospy.Subscriber('/zed2/zed_node/rgb/image_rect_color', Image, self.callback)
        # self.subscriberDepth = rospy.Subscriber("/zed2/zed_node/depth/depth_registered", Image, self.depthImageRosCallback)
        # self.subscriberInfo = rospy.Subscriber("/zed2/zed_node/depth/camera_info", CameraInfo, self.infoImageRosCallback)
        
        self.pubmask = rospy.Publisher('/mask_aruco', Image, queue_size=10)
        self.mask  = None
        self.cv_image = np.array([])
        rospy.loginfo("Subscribed to image")
        self.main()

    def detectar_arucos(self):
        frame = self.cv_image
        dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters_create()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
        frame_with_markers = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        self.cv_image = frame_with_markers
        self.pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, encoding="bgr8"))
        
    def callback(self, data):

        self.cv_image = self.bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")
        self.detectar_arucos()
    
    def main(self):
        rospy.logwarn("Starting listener")
        rospy.init_node('detector_arucos', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        try :
            while not rospy.is_shutdown():
                
                rate.sleep()
        except KeyboardInterrupt:
            rospy.logwarn("Keyboard interrupt detected, stopping listener")


if __name__ == '__main__':
    DetectorAruco()
#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def webcam_publisher():
    # Initialize the node
    rospy.init_node('webcam_publisher')

    # Create a publisher to publish the image on the topic "webcam_image"
    pub = rospy.Publisher('webcam_image', Image, queue_size=10)

    # Create a CvBridge object to convert between OpenCV images and ROS images
    bridge = CvBridge()

    # Initialize the OpenCV video capture object
    cap = cv2.VideoCapture(0)

    # Set the frame rate of the video capture object to 10 frames per second
    cap.set(cv2.CAP_PROP_FPS, 10)

    # Loop to capture and publish the image
    while not rospy.is_shutdown():
        # Read the current frame from the video capture object
        ret, frame = cap.read()

        # If the frame was read successfully
        if ret:
            # Convert the OpenCV image to a ROS image message
            ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")

            # Publish the ROS image message on the "webcam_image" topic
            pub.publish(ros_image)

if __name__ == '__main__':
    try:
        webcam_publisher()
    except rospy.ROSInterruptException:
        pass

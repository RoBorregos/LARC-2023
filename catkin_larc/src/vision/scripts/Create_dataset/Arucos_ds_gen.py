#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys
import tty
import termios

def get_key():
    # Save the terminal settings
    old_settings = termios.tcgetattr(sys.stdin)
    
    try:
        tty.setcbreak(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        # Restore the terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    
    return ch

def image_callback(msg):
    cont =0
    try:
        # Convert ROS image message to OpenCV image
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # Save the image to a file
        while True:
            #print("Press 's' to save the image or 'q' to quit")
            key = get_key()
            if key =='s':
                cv2.imwrite(str(cont) + ".png", cv_image)
                rospy.loginfo("Photo saved successfully!")
                cont = cont + 1
    except Exception as e:
        rospy.logerr(e)

def main():
    rospy.init_node('photo_capture_node', anonymous=True)
    rospy.Subscriber('/webcam_image', Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()

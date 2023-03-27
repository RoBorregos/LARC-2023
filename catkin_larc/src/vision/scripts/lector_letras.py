import rospy
import cv2
import easyocr
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import warnings
warnings.filterwarnings("ignore", category=UserWarning)


def callback(data): 
    
    rospy.loginfo(rospy.get_caller_id(), data.data)

    rospy.init_node('letter_publisher')

    rospy.spin()


def listener():
    sub = rospy.Subscriber('webcam_image', Image, callback)

def ordenar_puntos(puntos):
    n_puntos = np.concatenate([puntos[0],puntos[1], puntos[2],puntos[3]]).tolist()

    y_order = sorted(n_puntos, key=lambda n_puntos: n_puntos[1])

    x1_order = y_order[:2]
    x1_order = sorted(x1_order, key=lambda x1_order: x1_order[0])

    x2_order = y_order[2:4]
    x2_order = sorted(x2_order, key=lambda x2_order: x2_order[0])

    return [x1_order[0], x1_order[1],x2_order[0], x2_order[1]]

reader = easyocr.Reader(["en"],gpu=True)



    
if __name__== '__main __':
    listener()



#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

class DetectorColores:
    def __init__(self):
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/colores_out', Image, queue_size=10)
        self.pubcolor = rospy.Publisher('colors', String, queue_size=10)
        self.sub = rospy.Subscriber('/zed2/zed_node/rgb/image_rect_color', Image, self.callback)
        # self.subscriberDepth = rospy.Subscriber("/zed2/zed_node/depth/depth_registered", Image, self.depthImageRosCallback)
        # self.subscriberInfo = rospy.Subscriber("/zed2/zed_node/depth/camera_info", CameraInfo, self.infoImageRosCallback)

        self.pubmask = rospy.Publisher('/mask_colores', Image, queue_size=10)
        self.mask  = None
        self.cv_image = np.array([])
        rospy.loginfo("Subscribed to image")
        self.main()
    
    def dibujar(self,mask,color):
        frame= self.cv_image
        contornos,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        for c in contornos:
            area = cv2.contourArea(c)
            if area > 3000:
                M = cv2.moments(c)
                if (M["m00"]): M["m00"] = 1
                x = int(M["m10"]/M["m00"])
                y = int(M['m01']/M["m00"])
                nuevoContorno = cv2.convexHull(c)
                #cv2.circle(frame,(x,y),7,(0,255,0),-1)  
                #cv2.putText(frame,'{},{}'.format(x,y), (x+10,y), font, 0.75,(0,255,0),1,cv2.LINE_AA)

                if color == (255,0,0):
                    print('azul')
                    self.pubcolor.publish('azul')
                if color == (0,255,0):
                    print('verde')
                    self.pubcolor.publish('verde')
                if color == (0,0,255):
                    print('rojo')   
                    self.pubcolor.publish('rojo')
                if color == (0,255,255):
                    print('Amarillo')
                    self.pubcolor.publish('amarillo')

                cv2.drawContours(frame,[nuevoContorno],0,color,3)

    def callback(self, data):
        # rospy.loginfo(data.data)
        # implement cv_bridge
        self.cv_image = self.bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")
        self.detectar_colores()
        self.pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, encoding="bgr8"))

    def detectar_colores(self):
        frame = self.cv_image
        redBajo1 = np.array([0,150,45],np.uint8)
        redAlto1 = np.array([5,255,200],np.uint8)

        redBajo2 = np.array([170,100,45],np.uint8)
        redAlto2 = np.array([179,255,255],np.uint8)

        azulBajo = np.array([110,130,45],np.uint8)
        azulAlto = np.array([125,255,255],np.uint8)

        verdeBajo = np.array([50,100,20],np.uint8)
        verdeAlto = np.array([80,255,255],np.uint8)

        amarillobajo = np.array([15,100,20],np.uint8)
        amarilloalto = np.array([45,255,255],np.uint8) 

        font = cv2.FONT_HERSHEY_SIMPLEX

        frameHSV = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        maskAzul = cv2.inRange(frameHSV,azulBajo,azulAlto)
        maskVerde = cv2.inRange(frameHSV,verdeBajo,verdeAlto)
        maskamarillo = cv2.inRange(frameHSV,amarillobajo, amarilloalto)
        maskRed1 = cv2.inRange(frameHSV,redBajo1, redAlto1)
        maskRed2 = cv2.inRange(frameHSV,redBajo2, redAlto2)
        maskred = cv2.add(maskRed1,maskRed2)
        self.dibujar(maskAzul,(255,0,0))
        self.dibujar(maskamarillo,(0,255,255))
        self.dibujar(maskVerde,(0,255,0))
        self.dibujar(maskred,(0,0,255))
        #frame = cv2.resize(frame, (0, 0), fx = 0.3, fy = 0.3)
        #cv2.imshow('frame',frame)
        
    def main(self):
        rospy.logwarn("Starting listener")
        rospy.init_node('detector_colores', anonymous=True)
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
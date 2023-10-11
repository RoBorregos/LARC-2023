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

IMAGE_SUB = '/zedImageFiltered'
#IMAGE_SUB = '/zed2/zed_node/rgb/image_rect_color'


class DetectorColores:
    def __init__(self):
        rospy.init_node('detector_colores', anonymous=True)

        self.boxes = []
        self.cx = 0.0
        self.cy = 0.0
        self.detections = []

        self.image = np.array([])
        self.depth_image = np.array([])
        self.camera_info = CameraInfo()

        self.color_detections_data = objectDetectionArray()

        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/colores_out', Image, queue_size=10)
        self.pubData = rospy.Publisher('/vision/color_detect', objectDetectionArray, queue_size=5)
        self.pubcolor = rospy.Publisher('colors', String, queue_size=10)
        self.posePublisher = rospy.Publisher("/test/detectionposes", PoseArray, queue_size=5)
        self.flagsubs = rospy.Subscriber("flag", Bool, self.callback_flag)
        self.flag = True
        self.flag_selection = rospy.Subscriber("flag_selection", Int32, self.callback_flag_selection)
        self.flag_selection = 0 # 0 = pattern, 1 = cubes

        #Suscriber topics changed for simulation
        
        self.sub = rospy.Subscriber(IMAGE_SUB, Image, self.callback)
        self.subscriberDepth = rospy.Subscriber("/zed2/zed_node/depth/depth_registered", Image, self.depthImageRosCallback)
        self.subscriberInfo = rospy.Subscriber("/zed2/zed_node/depth/camera_info", CameraInfo, self.infoImageRosCallback)

        """
        self.sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback)
        self.subscriberDepth = rospy.Subscriber("/camera/depth/image_raw", Image, self.depthImageRosCallback)
        self.subscriberInfo = rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.infoImageRosCallback)
        """
        

        # server for detecting color pattern
        self.static_color_seq = "GBYRYBG" # static color sequence
        self.color_pattern_server = rospy.Service('/detect_color_pattern', DetectColorPattern, self.detect_color_pattern_cb)
        
        self.pubmask = rospy.Publisher('/mask_colores', Image, queue_size=10)
        self.mask  = None
        rospy.loginfo("Subscribed to image")
        self.main()
    
    def callback_flag(self, data):
        #self.flag = data.data
        rospy.loginfo(self.flag)
    
    def callback_flag_selection(self, data):
        self.flag_selection = data.data
        rospy.loginfo(self.flag_selection)


    def depthImageRosCallback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)


    # Function to handle a ROS camera info input.
    def infoImageRosCallback(self, data):
        self.camera_info = data
        self.subscriberInfo.unregister()
    
   

    def dibujar(self,mask,color):
        frame= self.image
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=5)
        contornos,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
       
        temp = []
        for c in contornos:
            area = cv2.contourArea(c)
            if area > 1000:
                M = cv2.moments(c)
                self.cx = int(M["m10"]/M["m00"]) / self.image.shape[1]
                self.cy = int(M['m01']/M["m00"]) / self.image.shape[0]

                if (M["m00"]):
                    M["m00"] = 1
                x = int(M["m10"]/M["m00"])
                y = int(M['m01']/M["m00"])
                nuevoContorno = cv2.convexHull(c)
                #cv2.circle(frame,(x,y),7,(0,255,0),-1)  
                #cv2.putText(frame,'{},{}'.format(x,y), (x+10,y), font, 0.75,(0,255,0),1,cv2.LINE_AA)
                x, y, w, h = cv2.boundingRect(c)
                xmenor = x
                ymenor = y
                xmayor = x + w
                ymayor = y + h

                temp =  ymenor, xmenor, ymayor, xmayor

                if color == (255,0,0):
                    print('azul')

                    self.detections.append('azul')
                    
                if color == (0,255,0):
                    print('verde')
                    self.detections.append('verde')

                if color == (0,0,255):
                    print('rojo')   
                    self.detections.append('rojo')

                if color == (0,255,255):
                    print('amarillo')
                    self.detections.append('amarillo')

                cv2.drawContours(frame,[nuevoContorno],0,color,3)
                self.boxes.append(temp)

    def pc_callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")
            self.color_detection()
            self.pubcolor = rospy.Publisher('colors', String, queue_size=10)
        except CvBridgeError as e:
            print(e)
    

    def callback(self, data):
        # rospy.loginfo(data.data)
        # implement cv_bridge
        self.image = self.bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")
        if self.flag:
            self.color_detection()
            self.pub.publish(self.bridge.cv2_to_imgmsg(self.image, encoding="bgr8"))
        self.boxes = []
        self.detections = []

    def get_objects(self, boxes, detections):
        res = []
        #sort boxes full content by first parameter, then second, and save in new obj
        sorted_boxes = []
        sorted_detections = []
        for box in boxes:
            appended = False
            for i in range(len(sorted_boxes)):
                if sorted_boxes[i][1]>box[1] and abs(sorted_boxes[i][0]-box[0])<50 or sorted_boxes[i][0]-box[0]>50:
                    sorted_boxes.insert(i, box)
                    sorted_detections.insert(i, detections[boxes.index(box)])
                    appended = True
                    break
            if not appended:
                sorted_boxes.append(box)
                sorted_detections.append(detections[boxes.index(box)])
        boxes = sorted_boxes
        detections = sorted_detections

        pa = PoseArray()
        pa.header.frame_id = "camera_depth_frame"
        pa.header.stamp = rospy.Time.now()
        for index in range(len(boxes)):
            if True:
                point3D = Point()
                rospy.logwarn("---------------------------")


                rospy.logwarn("pose")
                point2D  = get2DCentroid(boxes[index])
                rospy.logwarn(point2D)
                # Dummy point2d

                if len(self.depth_image) != 0:
                    depth = get_depth(self.depth_image, point2D)
                    point3D_ = deproject_pixel_to_point(self.camera_info, point2D, depth)
                    point3D.y = point3D_[1]
                    point3D.z = point3D_[2]
                    if point3D.z > 1.3:
                        point3D.x = point3D_[0] +0.02
                    else:
                        point3D.x = point3D_[0] - 0.05

                    pa.poses.append(Pose(position=point3D))
                    res.append(
                    objectDetection(
                        label = int(index), # 1
                        labelText = str(detections[index]), # "H"
                        category = str('color'),
                        #score = float(0.0),
                        cx = float(self.cx),
                        cy = float(self.cy),
                        ymin = float(boxes[index][0]),
                        xmin = float(boxes[index][1]),
                        ymax = float(boxes[index][2]),
                        xmax = float(boxes[index][3]),
                        depth = float(depth),
                        point3D = point3D
                    )
                )
            self.posePublisher.publish(pa)

        self.color_detections_data = objectDetectionArray(detections=res)
        self.pubData.publish( objectDetectionArray(detections=res) )
        

    def color_detection(self):
        frame = self.image

        lowerRed = np.array([0,162,150], np.uint8)
        upperRed = np.array([7,255,255], np.uint8)
        lowerRed2 = np.array([155,162,150], np.uint8)
        upperRed2 = np.array([179,255,255], np.uint8)
        
        lowerBlue = np.array([105,250,200], np.uint8)
        upperBlue = np.array([121,255,255], np.uint8)
        
        lowerYellow = np.array([26,171,168], np.uint8)
        upperYellow = np.array([34,255,255], np.uint8)
        
        lowerGreen = np.array([76,150,15], np.uint8)
        upperGreen = np.array([103,255,255], np.uint8)

        if self.flag_selection == 1:
            lowerRed = np.array([0,143,110], np.uint8)
            upperRed = np.array([1,237,255], np.uint8)
            lowerRed2 = np.array([166,143,110], np.uint8)
            upperRed2 = np.array([179,237,255], np.uint8)

            lowerBlue = np.array([123,164,100], np.uint8)
            upperBlue = np.array([135,255,192], np.uint8)

            """lowerGreen = np.array([81,108,72], np.uint8)
            upperGreen = np.array([104,185,88], np.uint8)"""

            lowerGreen = np.array([81,79,61], np.uint8)
            upperGreen = np.array([131,188,91], np.uint8)

            lowerYellow = np.array([13,170,113], np.uint8)
            upperYellow = np.array([34,255,255], np.uint8)           

        
        """lowerRed = np.array([0,128,71], np.uint8)
        upperRed = np.array([12,255,224], np.uint8)
        lowerRed2 = np.array([174,128,71], np.uint8)
        upperRed2 = np.array([179,255,224], np.uint8)"""
        


        """lowerBlue = np.array([109,99,49], np.uint8)
        upperBlue = np.array([126,255,163], np.uint8)

        lowerGreen2 = np.array([51,90,29], np.uint8)
        upperGreen2 = np.array([72,255,168], np.uint8)

        lowerGreen2 = np.array([71,30,26], np.uint8)
        upperGreen2 = np.array([107,191,66], np.uint8)

        lowerYellow = np.array([21,163,82], np.uint8)
        upperYellow = np.array([30,255,255], np.uint8)"""

        

        font = cv2.FONT_HERSHEY_SIMPLEX

        frameHSV = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        maskAzul = cv2.inRange(frameHSV, lowerBlue, upperBlue)
        maskVerde1 = cv2.inRange(frameHSV, lowerGreen, upperGreen)
        #maskVerde2 = cv2.inRange(frameHSV, lowerGreen2, upperGreen2)
        maskamarillo = cv2.inRange(frameHSV, lowerYellow, upperYellow)
        maskRed1 = cv2.inRange(frameHSV, lowerRed, upperRed)
        maskRed2 = cv2.inRange(frameHSV, lowerRed2, upperRed2)
        maskred = cv2.add(maskRed1,maskRed2)
        #maskverde = cv2.add(maskVerde1,maskVerde2)
        maskverde = maskVerde1
        self.dibujar(maskAzul,(255,0,0))
        self.dibujar(maskamarillo,(0,255,255))
        self.dibujar(maskverde,(0,255,0))
        self.dibujar(maskred,(0,0,255))

        self.get_objects(self.boxes, self.detections)
        #frame = cv2.resize(frame, (0, 0), fx = 0.3, fy = 0.3)
        #cv2.imshow('frame',frame)

    def detect_color_pattern_cb(self, req):
        print("detect_color_pattern_cb")
        data = self.color_detections_data
        xTile = 0
        yTile = 0
        cTile = DetectColorPatternResponse()
        cTile.tileX = xTile
        cTile.tileY = yTile

        sz = len(data.detections)
        if sz == 0:
            return cTile
    
        y_min_first = data.detections[0].ymin
        x_last_max = data.detections[0].xmin
        point_x_min_id = 0
        color_seq = ""

        color2Letter = {
            "rojo": "R",
            "verde": "G",
            "azul": "B",
            "amarillo": "Y"
        }
    
        for i in range(sz):
            if abs(data.detections[i].ymin - y_min_first) >= 50 or abs(data.detections[i].xmin - x_last_max) >= 230:
                continue
            color_seq += color2Letter[ data.detections[i].labelText ]

            x_last_max = data.detections[i].xmax
            if( abs(data.detections[i].point3D.x) < abs(data.detections[point_x_min_id].point3D.x)):
                point_x_min_id = i

        #check if subsequence
        print(color_seq)
        if color_seq in self.static_color_seq and len(color_seq) >= 3:
            rospy.loginfo("Color sequence detected: " + color_seq)
            #get square from closer point x and adjacents
            x_square_label = color2Letter[ data.detections[point_x_min_id].labelText ]
            x_square_cont = ""
            if point_x_min_id > 0:
                x_square_cont = color2Letter[ data.detections[point_x_min_id - 1].labelText ] + x_square_label
            else:
                x_square_cont = x_square_label + color2Letter[ data.detections[point_x_min_id + 1].labelText ]

            if x_square_label == "G" and x_square_cont == "GB":
                xTile = 7
            elif x_square_label == "B" and (x_square_cont == "GB" or x_square_cont == "BY"):
                xTile = 6
            elif x_square_label == "Y" and (x_square_cont == "BY" or x_square_cont == "YR"):
                xTile = 5
            elif x_square_label == "R":
                xTile = 4
            elif x_square_label == "Y" and (x_square_cont == "RY" or x_square_cont == "YB"):
                xTile = 3
            elif x_square_label == "B" and (x_square_cont == "YB" or x_square_cont == "BG"):
                xTile = 2
            elif x_square_label == "G" and x_square_cont == "BG":
                xTile = 1

            y_point = data.detections[point_x_min_id].point3D.z
            if y_point < 0.47: #0.52
                yTile = 1
            elif y_point < 0.71:
                yTile = 2
            elif y_point < 0.96:
                yTile = 3
            elif y_point < 1.23:
                yTile = 4
            elif y_point < 1.5:
                yTile = 5
            elif y_point < 1.76:
                yTile = 6

            print("xTile: " + str(xTile) + ", yTile: " + str(yTile))

        cTile = DetectColorPatternResponse()
        cTile.tileX = xTile
        cTile.tileY = yTile

        return cTile
        
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
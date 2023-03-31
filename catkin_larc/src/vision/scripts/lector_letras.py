#!/usr/bin/env python3
# USAGE
# python3 Detection2d.py

import rospy
import cv2
import easyocr
import numpy as np  
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from vision.msg import objectDetection, objectDetectionArray
import pathlib
from geometry_msgs.msg import Point, PoseArray, Pose

import warnings
warnings.filterwarnings("ignore", category=UserWarning)

import sys
sys.path.append(str(pathlib.Path(__file__).parent) + '/../include')
from vision_utils import *

reader = easyocr.Reader(["en"],gpu=True)

class DetectorLetras:
    def __init__(self):
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/letras_out', Image, queue_size=10)
        self.pubData = rospy.Publisher('detections', objectDetectionArray, queue_size=5)
        self.publetter = rospy.Publisher('letters', String, queue_size=10)
        self.posePublisher = rospy.Publisher("/test/detectionposes", PoseArray, queue_size=5)
        self.sub = rospy.Subscriber('/zed2/zed_node/rgb/image_rect_color', Image, self.callback)
        self.subscriberDepth = rospy.Subscriber("/zed2/zed_node/depth/depth_registered", Image, self.depthImageRosCallback)
        self.subscriberInfo = rospy.Subscriber("/zed2/zed_node/depth/camera_info", CameraInfo, self.infoImageRosCallback)

        self.pubmask = rospy.Publisher('/mask_letras', Image, queue_size=10)
        self.mask  = None
        self.cv_image = np.array([])
        rospy.loginfo("Subscribed to image")
        self.main()
    
    # Function to handle a ROS depth input.
    def depthImageRosCallback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)

    # Function to handle a ROS camera info input.
    def infoImageRosCallback(self, data):
        self.camera_info = data
        self.subscriberInfo.unregister()

    def callback(self, data):
        # rospy.loginfo(data.data)
        # implement cv_bridge
        self.cv_image = self.bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")
        self.lector()
        self.pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, encoding="bgr8"))


    def ordenar_puntos(self, puntos):
        n_puntos = np.concatenate([puntos[0],puntos[1], puntos[2],puntos[3]]).tolist()

        y_order = sorted(n_puntos, key=lambda n_puntos: n_puntos[1])

        x1_order = y_order[:2]
        x1_order = sorted(x1_order, key=lambda x1_order: x1_order[0])

        x2_order = y_order[2:4]
        x2_order = sorted(x2_order, key=lambda x2_order: x2_order[0])

        return [x1_order[0], x1_order[1],x2_order[0], x2_order[1]]
    

    def cleanup_text(self, text):
        return "".join([c if ord(c) < 128 else "" for c in text]).strip()


    def lector(self):
        rospy.loginfo("lector")
        frame = self.cv_image
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        canny = cv2.Canny(gray,300,150)
        canny = cv2.dilate(canny,None, iterations=1)
        cnts = cv2.findContours(canny,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        cnts = sorted(cnts, key = cv2.contourArea, reverse=True)[:5]
    

        for c in cnts:
            epsilon = 0.01*cv2.arcLength(c,True)
            approx = cv2.approxPolyDP(c,epsilon,True)

            if len(approx)==4:
                cv2.drawContours(frame, [approx], 0,(0,255,255),2)
                puntos = self.ordenar_puntos(approx)
                rospy.logwarn("Something detected")
                cv2.circle(frame, tuple(puntos[0]), 7, (255,0,0),2)
                cv2.circle(frame, tuple(puntos[1]), 7, (0,255,0),2)
                cv2.circle(frame, tuple(puntos[2]), 7, (0,0,255),2)
                cv2.circle(frame, tuple(puntos[3]), 7, (255,255,0),2)
                pts1 = np.float32(puntos)
                pts2 = np.float32([[0,0],[270,0],[0,270],[270,270]])
                M = cv2.getPerspectiveTransform(pts1,pts2)  
                dst = cv2.warpPerspective(gray,M,(270,310))
                dst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
                cv2.imshow('dst',dst)
                cv2.waitKey(1)
        
                result = reader.readtext(dst)
                # for res in result:

                #     rospy.logwarn(res[1])
                #     self.publetter.publish(res[1])
                
                bb = []
                detections = []
                conf = []
                for bounding_box, text, prob in result:

                    text = self.cleanup_text(text)
                    if text >= "A" and text <= "I" or text >="a" and text <= "i":
                        #print(f'{prob:0.4f} : {text}')

                        
                        rospy.logwarn(text)
                        #self.publetter.publish(text)

                        #rospy.logwarn(bounding_box)
                        tl, tr, br, bl = bounding_box
                        
                        # Llamas get_objects mandandole como atributo la lista de bounding boxes y la deteccion correspondiente en lista.
                        # deteccion de texto "text"
                        # deteccion de bounding box "bounding_box"
                        # probabilidad de deteccion "prob"
                        tl = (int(tl[0]), int(tl[1]))
                        tr = (int (tr[0]), int(tr[1]))
                        br = (int (br[0]), int(br[1]))
                        bl = (int (bl[0]), int(bl[1]))
                        xmayor = max(tl[0], tr[0], br[0], bl[0])
                        ymayor = max(tl[1], tr[1], br[1], bl[1])
                        xmenor = min(tl[0], tr[0], br[0], bl[0])
                        ymenor = min(tl[1], tr[1], br[1], bl[1])

                        tempo = ymenor, xmenor, ymayor, xmayor
                        rospy.logwarn(tempo)
                        cv2.circle(frame, tuple(puntos[0]), 7, (255,0,0),2)
                        cv2.circle(frame, tuple(puntos[1]), 7, (0,255,0),2)
                        result = reader.readtext(dst)
                        cv2.rectangle(dst, tl, br, (0, 255, 0), 2)
                        cv2.putText(dst, text, (tl[0], tl[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                        #añadir a la lista de objetos
                        bb.append(tempo)
                        rospy.logwarn(bb)
                        detections.append(text)
                        conf.append(prob)

            #mandar los pkgs
                self.get_objects(bb, detections, conf)

        # frame = cv2.resize(frame, (0, 0), fx = 0.3, fy = 0.3)
        #cv2.imshow('Frame',frame)
        self.cv_image = frame
        


    def obtener_texto(self, puntos, frame):
        puntos = self.ordenar_puntos(puntos)
        pts1 = np.float32(puntos)
        pts2 = np.float32([[0,0],[300,0],[0,300],[300,300]])
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
        img_warp_colored = cv2.warpPerspective(frame, matrix, (300,300))
        img_warp_colored = cv2.cvtColor(img_warp_colored, cv2.COLOR_BGR2RGB)
        resultado = reader.readtext(img_warp_colored)
        return resultado
    
    def my_get2dCentroid(self, box):
        # box = [ymin, xmin, ymax, xmax]
        # centroid = [(xmin+xmax)/2, (ymin+ymax)/2]
        centroid = [(box[1]+box[3])/2, (box[0]+box[2])/2]
        return centroid

    
    def my_get_2d_centroid(self, image, bbox, depth_map):
        # Get coordinates of bounding box
        x1, y1, x2, y2 = bbox

        # Crop image and depth map to bounding box
        cropped_image = image[y1:y2, x1:x2]
        cropped_depth = depth_map[y1:y2, x1:x2]

        # Calculate centroid of cropped image
        M = cv2.moments(cropped_image)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX, cY = 0, 0

        # Get depth value at centroid
        depth_value = cropped_depth[cY, cX]

        # Return centroid coordinates and depth value
        return (x1 + cX, y1 + cY), depth_value


    


    # This function creates the output array of the detected objects with its 2D & 3D coordinates.
    def get_objects(self, boxes, detections, scores):
        res = []

        pa = PoseArray()
        pa.header.frame_id = "camera_depth_frame"
        pa.header.stamp = rospy.Time.now()
        for index in range(len(boxes)):
            if scores[index] > 0.8:
                point3D = Point()
                rospy.logwarn("---------------------------")


                rospy.logwarn("pose")
                point2D  = get2DCentroid(boxes[index])
                rospy.logwarn(point2D)
                # Dummy point2d

                if len(self.depth_image) != 0:
                    depth = get_depth(self.depth_image, point2D)
                    point3D_ = deproject_pixel_to_point(self.camera_info, point2D, depth)
                    point3D.x = point3D_[0]
                    point3D.y = point3D_[1]
                    point3D.z = point3D_[2]
                    pa.poses.append(Pose(position=point3D))
                    res.append(
                    objectDetection(

                        label = int(index), # 1
                        labelText = str(detections[index]), # "H"
                        score = float(scores[index]), # 0.9
                        ymin = float(boxes[index][0]),
                        xmin = float(boxes[index][1]),
                        ymax = float(boxes[index][2]),
                        xmax = float(boxes[index][3]),
                        point3D = point3D
                    )
                )
            self.posePublisher.publish(pa)

        self.pubData.publish(objectDetectionArray(detections=res))                


    def main(self):
        rospy.logwarn("Starting listener")
        rospy.init_node('letter_reader', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        try:
            while not rospy.is_shutdown():
                
                rate.sleep()
        except KeyboardInterrupt:
            rospy.logwarn("Keyboard interrupt detected, stopping listener")

        


if __name__ == "__main__":
    DetectorLetras()



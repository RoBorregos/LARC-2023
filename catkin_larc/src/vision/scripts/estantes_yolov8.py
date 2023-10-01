#!/usr/bin/env python3
# USAGE
# python3 Detection2d.py

import rospy
import ultralytics
import cv2
import time
import numpy as np
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from vision.msg import objectDetection, objectDetectionArray
import pathlib
from geometry_msgs.msg import Point, PoseArray, Pose

import sys
sys.path.append(str(pathlib.Path(__file__).parent) + '/../include')
from vision_utils import *

class estantes_yolov8:
        def __init__(self):
            #self.model = ultralytics.YOLO("/home/nvidia/Desktop/LARC-2023/catkin_larc/src/vision/scripts/yolov8n.pt")
            self.model = ultralytics.YOLO("/home/nvidia/Desktop/LARC-2023/catkin_larc/src/vision/scripts/estantes_yolov8_v2.pt")
            self.bridge = CvBridge()

            self.pubdata = rospy.Publisher('vision/estantes/info', objectDetectionArray, queue_size=5)
            self.pubimg = rospy.Publisher('vision/estantes/image', Image, queue_size=10)
            self.posePublisher = rospy.Publisher("vision/estantes/detectionposes", PoseArray, queue_size=5)

            self.sub = rospy.Subscriber('/zed2/zed_node/rgb/image_rect_color', Image, self.callback)
            self.subscriberDepth = rospy.Subscriber("/zed2/zed_node/depth/depth_registered", Image, self.depthImageRosCallback)
            self.subscriberInfo = rospy.Subscriber("/zed2/zed_node/rgb/camera_info", CameraInfo, self.infoImageRosCallback)
            self.flagsubs = rospy.Subscriber("flag", Bool, self.callback_flag)

            self.cv_image = np.array([])
            rospy.loginfo("letras_yolov8 initialized")
            self.main()

            rospy.loginfo("Loading model...")
            rospy.loginfo("estantes_yolov8 lectura")
            self.yolov8_warmup(self.model, 10, False)



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
            if flag:
                self.lector()
                self.pubimg.publish(self.bridge.cv2_to_imgmsg(self.cv_image, encoding="bgr8"))
        
        def callback_flag(self, data):
            global flag
            flag = data.data
            #rospy.loginfo(flag)
        
        def lector(self):
            
            frame = self.cv_image
            #model = torch.hub.load("ultralytics/yolov5", "yolov5s")  # or yolov5n - yolov5x6, custom
            
            #prevtime = time.time()

            #print(f"Model loaded in {time.time() - prevtime} seconds")

            #prevtime = time.time()
            results = self.model(frame, verbose=False)
            boxes, confidences, classids = self.generate_boxes_confidences_classids_v8(results, 0.85)
            #print(f"Prediction done in {time.time() - prevtime} seconds")
            # Draw results
            bb = []
            detections = []
            for i in range(len(boxes)):
                x, y, w, h = boxes[i]
                cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
                cv2.putText(frame, str(confidences[i]), (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                class_name = self.model.model.names[classids[i]]
                cv2.putText(frame, str(class_name), (x, y-30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                xmenor = x
                ymenor = y
                xmayor = x + w
                ymayor = y + h

                temp =  ymenor, xmenor, ymayor, xmayor
                bb.append(temp)
                detections.append(classids[i])
            
            self.get_objects(bb, detections)
            self.pubimg.publish(self.bridge.cv2_to_imgmsg(self.cv_image, encoding="bgr8"))

        def get_objects(self, boxes, detections):
            res = []

            pa = PoseArray()
            pa.header.frame_id = "zed2_base_link"
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
                        point3D.x = point3D_[0]
                        point3D.y = point3D_[1]
                        point3D.z = point3D_[2]
                        pa.poses.append(Pose(position=point3D))
                        res.append(
                        objectDetection(

                            label = int(index), # 1
                            labelText = str(detections[index]), # "H"
                            #score = float(0.0),
                            ymin = float(boxes[index][0]),
                            xmin = float(boxes[index][1]),
                            ymax = float(boxes[index][2]),
                            xmax = float(boxes[index][3]),
                            point3D = point3D
                        )
                    )
                self.posePublisher.publish(pa)

            self.pubdata.publish(objectDetectionArray(detections=res))

            #cv2.imshow("frame", frame)
            # ret, frame = cap.read()
            # if cv2.waitKey(1) & 0xFF == ord("q"):
            #     break

            # cap.release()
            #cv2.destroyAllWindows()

              
        def generate_boxes_confidences_classids_v8(self, outs, threshold):
            boxes = []
            confidences = []
            classids = []

            for out in outs:
                    for box in out.boxes:
                        x1, y1, x2, y2 = [round(x) for x in box.xyxy[0].tolist()]
                        class_id = box.cls[0].item()
                        prob = round(box.conf[0].item(), 2)
                        if prob > threshold:
                            # Append to list
                            boxes.append([x1, y1, x2-x1, y2-y1])
                            confidences.append(float(prob))
                            classids.append(class_id)
        
            return boxes, confidences, classids
        

        def yolov8_warmup(self, model, repetitions=1, verbose=False):
            # Warmup model
            startTime = time.time()
            # create an empty frame to warmup the model
            for i in range(repetitions):
                warmupFrame = np.zeros((360, 640, 3), dtype=np.uint8)
                self.model.predict(source=warmupFrame, verbose=verbose)
            rospy.loginfo(f"Model warmed up in {time.time() - startTime} seconds")

        def main(self):
            rospy.logwarn("Starting listener")
            #rospy.logwarn("aqui mamo")

            rospy.init_node('detector_letras', anonymous=True)
            rate = rospy.Rate(10)
            try:
                while not rospy.is_shutdown():
                
                    #self.pubData.publish(self.dtections)
                    rate.sleep()
                    cv2.waitKey(1)
            except KeyboardInterrupt:
                rospy.logwarn("Keyboard interrupt detected, stopping listener")

if __name__ == '__main__':
    estantes_yolov8()


#--------------------------------------------------------------------------------------------------------------

# def generate_boxes_confidences_classids_v8(outs, threshold):
# 		boxes = []
# 		confidences = []
# 		classids = []

# 		for out in outs:
# 				for box in out.boxes:
# 					x1, y1, x2, y2 = [round(x) for x in box.xyxy[0].tolist()]
# 					class_id = box.cls[0].item()
# 					prob = round(box.conf[0].item(), 2)
# 					if prob > threshold:
# 						# Append to list
# 						boxes.append([x1, y1, x2-x1, y2-y1])
# 						confidences.append(float(prob))
# 						classids.append(class_id)
	
# 		return boxes, confidences, classids

# def yolov8_warmup(model, repetitions=1, verbose=False):
#     # Warmup model
#     startTime = time.time()
#     # create an empty frame to warmup the model
#     for i in range(repetitions):
#         warmupFrame = np.zeros((360, 640, 3), dtype=np.uint8)
#         model.predict(source=warmupFrame, verbose=verbose)
#     print(f"Model warmed up in {time.time() - startTime} seconds")
    
# #model = torch.hub.load("ultralytics/yolov5", "yolov5s")  # or yolov5n - yolov5x6, custom
# print("Loading model...")
# prevtime = time.time()
# model = ultralytics.YOLO("larc_yolov8_test4.pt")
# yolov8_warmup(model, 10, False)
# print(f"Model loaded in {time.time() - prevtime} seconds")

# cap = cv2.VideoCapture(0)
# ret, frame = cap.read()

# while ret:
#     prevtime = time.time()
#     results = model(frame, verbose=False)
#     boxes, confidences, classids = generate_boxes_confidences_classids_v8(results, 0.65)
#     print(f"Prediction done in {time.time() - prevtime} seconds")
#     # Draw results
#     for i in range(len(boxes)):
#         x, y, w, h = boxes[i]
#         cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
#         cv2.putText(frame, str(confidences[i]), (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
#         class_name = model.model.names[classids[i]]
#         cv2.putText(frame, str(class_name), (x, y-30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
#     cv2.imshow("frame", frame)
#     ret, frame = cap.read()
#     if cv2.waitKey(1) & 0xFF == ord("q"):
#         break

# cap.release()
# #cv2.destroyAllWindows()
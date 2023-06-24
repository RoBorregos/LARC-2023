#!/usr/bin/env python3

# Asynchronous service that runs YOLOv5 model to detect clothing objects in an image
import rospy
import rospkg
from humanAnalyzer.srv import imagetoAnalyze, imagetoAnalyzeResponse
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tensorflow as tf
import cv2
import torch
import json

ArUcos = ['a1', 'a2', 'a3', 'a4', 'a5', 'a6', 'a7', 'a8', 'a9']

class aruco_detection:

    def __init__(self):
        rospy.init_node('aruco_detection')
        self.doAction = False
        # Setting as busy to initialize
        self.statePub = rospy.Publisher("/aruco_detection", Bool, queue_size=1)
        self.statePub.publish(True)

        # Setting up image receiver
        self.received_image = None
        self.bridge = CvBridge()
        self.imageSub = rospy.Subscriber(
            'image', Image, self.process_image, queue_size=10)
        
        # Setting up server
        self.aruco_detection = rospy.Service('aruco_detection', imagetoAnalyze, self.handle_request)
        
        # Model location
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('vision')
        self.model_path = f"{pkg_path}/src/scripts/best.pt"
        self.imgs_path = f"{pkg_path}/src/scripts/"
        # Loading model
        device = torch.device("gpu" if torch.cuda.is_available() else "cpu")
        self.model = torch.hub.load("ultralytics/yolov5", "custom", self.model_path, device = "gpu")  # or yolov5n - yolov5x6, custom

    # Receive image from camera
    def process_image(self, msg):
        try:
            self.received_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        except Exception as e:
            print("Image not received")
            print(e)
    
    #-----------SERVER---------------------------------
    def handle_request(self, req):
        print(f"Received request from client : {req}")
        self.req = req
        self.doAction = True
        return True

    
    def tempProcess(self):
        if self.doAction:
            self.statePub.publish(True)
            # Running the model
            print("Running YOLOv7 model...")
            # by default check the image received from the camera
            print("Using image from webcam")
            face_id = self.req.imagetoAnalyze.identity
            face_x = self.req.imagetoAnalyze.x
            face_y = self.req.imagetoAnalyze.y
            face_w = self.req.imagetoAnalyze.w
            face_h = self.req.imagetoAnalyze.h
            img = self.received_image[face_y:face_y+face_h, face_x:face_x+face_w]

            results = self.model(img)
            showimg = img.copy()
            accesories = []
            for *xyxy, conf, cls in results.pandas().xyxy[0].itertuples(index=False):
                if cls in ArUcos:
                    print(f"Predicted {cls} at {[round(elem, 2) for elem in xyxy ]} with confidence {conf:.2f}.")
                    accesories.append(cls)
                showimg = cv2.rectangle(showimg, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), (0, 255, 0), 2)
                showimg = cv2.putText(showimg, f"{cls} {conf:.2f}", (int(xyxy[0]), int(xyxy[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            if len(accesories) > 0:
                data = {}
                print(f"Found {len(accesories)} accesories.")
                data[face_id]["Accesories"] = accesories
                print("Accesories added to json")
                with open(self.json_path, 'w') as outfile:
                    json.dump(data, outfile)
            else:
                print("No accesories found")
            cv2.imshow("ClothingAnalysis", showimg)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            self.statePub.publish(False)
            self.doAction = False
            return True

    def run(self):
        
        print("Ready to receive requests")
        while not rospy.is_shutdown():
            self.statePub.publish(False)
            self.tempProcess()
            rospy.Rate(5.0).sleep()

# run with cpu

aruco_detection().run()
#!/usr/bin/env python3
import argparse
import time
from pathlib import Path

import rospy
import cv2
import torch
import torch.backends.cudnn as cudnn
from numpy import random
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import os

from std_msgs.msg import Int32
from vision.msg import objectDetection, objectDetectionArray
from sensor_msgs.msg import Image, CameraInfo

from models.experimental import attempt_load
from utils.datasets import LoadStreams, LoadImages
from utils.general import check_img_size, check_requirements, check_imshow, non_max_suppression, apply_classifier, \
    scale_coords, xyxy2xywh, strip_optimizer, set_logging, increment_path
from utils.plots import plot_one_box
from utils.torch_utils import select_device, load_classifier, time_synchronized, TracedModel

class DetectorAruco:

    def __init__(self):
        self.bridge = CvBridge()
        self.pubInfo = rospy.Publisher('/aruco_info', objectDetectionArray, queue_size=5)
        self.pubFrame = rospy.Publisher('/aruco_frame', Image, queue_size=10)
        #self.sub = rospy.Subscriber('/zed2/zed_node/rgb/image_rect_color', Image, self.callback) #zed2
        self.sub = rospy.Subscriber('/webcam_image', Image, self.callback) #webcam
        
        self.cv_image = np.array([])
        self.main()


    def callback(self, data):
        self.cv_image = self.bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")
        device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')

        with torch.no_grad():
            self.detect("0", "/home/jabv/Desktop/LARC-2023/catkin_larc/src/vision/scripts/best.pt", device, img_size = 640, iou_thres = 0.45, conf_thres = 0.5)


    def detect(self,source, weights, device, img_size, iou_thres, conf_thres):

        frame = self.cv_image

        webcam = source.isnumeric()

        set_logging()
        #device = select_device(device)
        device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
        half = device.type != 'cpu'  # half precision only supported on CUDA

        # Load model
        model = attempt_load(weights, map_location=device)  # load FP32 model
        stride = int(model.stride.max())  # model stride
        imgsz = check_img_size(img_size, s=stride)  # check img_size


        if half:
            model.half()  # to FP16

        # Set Dataloader
        if webcam:
            view_img = check_imshow()
            cudnn.benchmark = True  # set True to speed up constant image size inference
            dataset = LoadStreams(source,img_size=imgsz, stride=stride)

        # Get names and colors
        names = model.module.names if hasattr(model, 'module') else model.names
        colors = [[random.randint(0, 255) for _ in range(3)] for _ in names]

        # Run inference
        if device.type != 'cpu':
            model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))  # run once

        old_img_w = old_img_h = img_size
        old_img_b = 1

        t0 = time.perf_counter()

        for path, img, im0s, vid_cap in dataset:
            img = torch.from_numpy(img).to(device)
            img = img.half() if half else img.float()  # uint8 to fp16/32
            img /= 255.0  # 0 - 255 to 0.0 - 1.0
            if img.ndimension() == 3:
                img = img.unsqueeze(0)

            # Warmup
            if device.type != 'cpu' and (old_img_b != img.shape[0] or old_img_h != img.shape[2] or old_img_w != img.shape[3]):
                old_img_b = img.shape[0]
                old_img_h = img.shape[2]
                old_img_w = img.shape[3]
            
            # Inference
            t1 = time_synchronized()
            with torch.no_grad():   # Calculating gradients would cause a GPU memory leak
                pred = model(img)[0]
            t2 = time_synchronized()

            # Apply NMS
            pred = non_max_suppression(pred, conf_thres, iou_thres)
            t3 = time_synchronized()


            # Process detections
            for i, det in enumerate(pred):  # detections per image
                #print("webcam",webcam)
                if webcam:  # batch_size >= 1
                    p, s, im0, frame = path[i], '%g: ' % i, im0s[i].copy(), dataset.count

                #p = path(p)  # to Path

                gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
                if len(det):
                    # Rescale boxes from img_size to im0 size
                    det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                    # Print results
                    for c in det[:, -1].unique():
                        n = (det[:, -1] == c).sum()  # detections per class
                        s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

                    # Write results
                    detect_num=0
                    tempo = []
                    for *xyxy, conf, cls in reversed(det):
                            
                        label = f'{names[int(cls)]} {conf:.2f}'
                        confidece = f'{conf:.2f}'

                        if(float(confidece)>0.5): #Truncate the confidence level less than 0.5
                            #print(pred[0][j])

                            name=names[int(cls)]
                            xmin=float(pred[0][detect_num][0])
                            ymin=float(pred[0][detect_num][1])
                            xmax=float(pred[0][detect_num][2])
                            ymax=float(pred[0][detect_num][3])

                            rospy.logwarn("---------------------------")

                            tempo.append(objectDetection(
                                    label = int(detect_num),
                                    labelText = str(name),
                                    score = confidece,
                                    ymin = ymin,
                                    xmin = xmin,
                                    ymax = ymax,
                                    xmax = xmax,
                                )
                            )

                            print ("detect_num",detect_num)
                            print(confidece)
                            print(name)
                            print("xmin",xmin)
                            print("ymin",ymin)
                            print("xmax",xmax)
                            print("ymax",ymax)

                            detect_num=detect_num+1

                        plot_one_box(xyxy, im0, label=label, color=colors[int(cls)], line_thickness=1)

                        self.pubFrame.publish(self.bridge.cv2_to_imgmsg(im0, "bgr8"))

                    self.pubInfo.publish(tempo)

                # Print time (inference + NMS)
                #print(f'{s}Done. ({(1E3 * (t2 - t1)):.1f}ms) Inference, ({(1E3 * (t3 - t2)):.1f}ms) NMS')

                # Stream results
            
            #cv2.imshow("frame", im0)
            #cv2.waitKey(1) # 1 millisecond

        #print(f'Done. ({time.time() - t0:.3f}s)')   

    def main(self):
        rospy.logwarn("ArUco_Detector Node Started")
        rospy.init_node('ArUco_Detector', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        try:
            while not rospy.is_shutdown():
                rate.sleep()
        except KeyboardInterrupt:
            rospy.logwarn("ArUco_Detector Node Stopped")



if __name__ == '__main__':

    DetectorAruco()
#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
from cv_bridge import CvBridge, CvBridgeError
from vision.msg import objectDetection, objectDetectionArray
import pathlib
from geometry_msgs.msg import Point, PoseArray, Pose

class Letras_tf:
    def __init__(self):
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/letras_out', Image, queue_size=10)
        self.pubData = rospy.Publisher('detections', objectDetectionArray, queue_size=5)
        self.publetter = rospy.Publisher('letters', String, queue_size=10)
        self.posePublisher = rospy.Publisher("/test/detectionposes", PoseArray, queue_size=5)
        self.sub = rospy.Subscriber('/zed2/zed_node/rgb/image_rect_color', Image, self.callback)
        self.subscriberDepth = rospy.Subscriber("/zed2/zed_node/depth/depth_registered", Image, self.depthImageRosCallback)
        self.ubscriberInfo = rospy.Subscriber("/zed2/zed_node/depth/camera_info", CameraInfo, self.infoImageRosCallback)
        self.pcsubs = rospy.Subscriber("/object", Image, self.pc_callback)
    
        self.pubmask = rospy.Publisher('/mask_letras', Image, queue_size=10)
        self.mask  = None
        self.cv_image = np.array([])
        rospy.loginfo("Subscribed to image")
        self.main()
        
    def pc_callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")
            self.lector()
        except CvBridgeError as e:
            print(e)
    
        
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
        
    def lector(self):
        image = self.cv_image
        interpreter = tf.lite.Interpreter(model_path="primero.tflite")
        interpreter.allocate_tensors()

        output = interpreter.get_output_details()[0]
        input = interpreter.get_input_details()[0]
        input_data = tf.constant(1., shape=[1, 1])
        
        image = "F.png" #aqui va el topico de la imagen que se esta recibiendo
        image = cv2.imread(image, cv2.IMREAD_UNCHANGED)
        shape = interpreter.get_input_details()[0]['shape']
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image = np.asanyarray(image, dtype="uint8")
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB) 
        image = cv2.resize(image, (shape[1], shape[2])) 
        plt.imshow(image)
        image = image.reshape(shape)
        interpreter.set_tensor(input['index'], image)
        interpreter.invoke()
        val = (interpreter.get_tensor(output['index'])[0])
        acum = 0
        max = 0
        
        if val[8]-20>0:
            val[8] = val[8]-20
            
        for i in range(len(val)):
            acum += val[i]
            if val[max] < val[i]:
                max = i

        data = ['A', 'B', 'C', 'D','E', 'F', ' G', 'H','I']

        print(val)
        print(data[max])
        self.publetter.publish(data[max])
        
    def get_objects(self, boxes, detections):
        res = []

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

        self.pubData.publish(objectDetectionArray(detections=res)) 

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
    Letras_tf()
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

class DetectorLetras:
    def __init__(self):
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/letras_out', Image, queue_size=10)
        self.pubData = rospy.Publisher('detections', objectDetectionArray, queue_size=5)
        self.publetter = rospy.Publisher('letters', String, queue_size=10)
        self.posePublisher = rospy.Publisher("/test/detectionposes", PoseArray, queue_size=5)
        self.sub = rospy.Subscriber('/zed2/zed_node/rgb/image_rect_color', Image, self.callback)
        self.subscriberDepth = rospy.Subscriber("/zed2/zed_node/depth/depth_registered", Image, self.depthImageRosCallback)
        self.ubscriberInfo = rospy.Subscriber("/zed2/zed_node/depth/camera_info", CameraInfo, self.infoImageRosCallback)
        
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
        
    def lector(self):
        interpreter = tf.lite.Interpreter(model_path="primero.tflite")
        interpreter.allocate_tensors()

        output = interpreter.get_output_details()[0]
        tf.lite.Interpreter(
            model_path="D:\\UsX\\Escritorio\\LARC-2023\\Vision\\primero.tflite",
            model_content=None,
            experimental_delegates=None,
            num_threads=None,
            experimental_op_resolver_type=tf.lite.experimental.OpResolverType.AUTO,
            experimental_preserve_all_tensors=False
        )
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
import rospy
import torch
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, \
    ObjectHypothesisWithPose
from vision.msg import objectDetectionArray, objectDetection
from geometry_msgs.msg import Pose2D
from typing import List, Union


def create_header():
    h = Header()
    h.stamp = rospy.Time.now()
    return h


def create_detection_msg(img_msg: Image, detections: torch.Tensor, class_labels):
    """
    :param img_msg: original ros image message
    :param detections: torch tensor of shape [num_boxes, 6] where each element is
        [x1, y1, x2, y2, confidence, class_id]
    :returns: detections as a ros message of type Detection2DArray
    """
    detection_array_msg = Detection2DArray()

    # header
    header = create_header()
    detection_array_msg.header = header
    detect_num = 0
    tempo = []
    for detection in detections:
        x1, y1, x2, y2, conf, cls = detection.tolist()
        single_detection_msg = Detection2D()
        single_detection_msg.header = header

        # src img
        single_detection_msg.source_img = img_msg

        # bbox
        bbox = BoundingBox2D()
        w = int(round(x2 - x1))
        h = int(round(y2 - y1))
        cx = int(round(x1 + w / 2))
        cy = int(round(y1 + h / 2))
        bbox.size_x = w
        bbox.size_y = h


        if(float(conf)>0.5): #Truncate the confidence level less than 0.5
            #print(pred[0][j])

            name=class_labels[int(cls)]
            xmin=x1
            ymin=y1
            xmax=x2
            ymax=y2
            
            #print("name: ",name, "xmin: ",xmin, "ymin: ",ymin, "xmax: ",xmax, "ymax: ",ymax, "conf: ",conf,Pose2D())

            #rospy.logwarn("---------------------------")

            tempo.append(objectDetection(
                    label = int(detect_num),
                    labelText = str(name),
                    score = conf,
                    ymin = ymin,
                    xmin = xmin,
                    ymax = ymax,
                    xmax = xmax
                )
            )
            detect_num=detect_num+1
        
        bbox.center = Pose2D()
        bbox.center.x = cx
        bbox.center.y = cy

        single_detection_msg.bbox = bbox

        # class id & confidence
        obj_hyp = ObjectHypothesisWithPose()
        obj_hyp.id = int(cls)
        obj_hyp.score = conf
        single_detection_msg.results = [obj_hyp]
        detection_array_msg.detections.append(single_detection_msg)

    return tempo
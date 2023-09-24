#!/usr/bin/env python3

import rospy
import tf_conversions
import tf2_ros
from vision.msg import objectDetection, objectDetectionArray
import geometry_msgs.msg
from geometry_msgs.msg import Point, Pose
from main_engine.srv import Intake, IntakeResponse

FIND_INIT_POS = "find_init_pos"
ROTATE_TO_CUBES = "rotate_to_cubes"
PICK_CUBE_TARGET = "pick_cube_target"
DRIVE_TO_TARGET = "drive_to_target"

class MainEngine:
    def __init__(self):
        self.current_time = rospy.Time.now()
        self.state = PICK_CUBE_TARGET
        self.target_success = False
        self.selected_target = Point()

        self.subColorDetect = rospy.Subscriber('/color_detect', objectDetectionArray, self.colorDetectCb)

        self.br = tf2_ros.TransformBroadcaster()
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)


    def colorDetectCb(self, data):
        if self.state == PICK_CUBE_TARGET:
            sz = len(data.detections)
            if sz == 0:
                return
            point_x_min_id = 0
            y_lowest = data.detections[0].ymin
            y_lowest_id = 0
            for i in range(sz):
                if data.detections[i].ymin - y_lowest >= 80:
                    y_lowest = data.detections[i].ymin
                    y_lowest_id = i
                    point_x_min_id = i
                elif abs(data.detections[i].ymin - y_lowest) < 80:
                    if( abs(data.detections[i].point3D.x) < abs(data.detections[point_x_min_id].point3D.x)):
                        point_x_min_id = i

            self.target_success = True
            self.selected_target = data.detections[point_x_min_id].point3D

    def run(self):
        self.current_time = rospy.Time.now()

        if self.state == PICK_CUBE_TARGET:
            if self.target_success:
                self.state = DRIVE_TO_TARGET

        if self.state == DRIVE_TO_TARGET:
            t = geometry_msgs.msg.TransformStamped()

            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "cam"
            t.child_frame_id = "qbo1"
            t.transform.translation.x = self.selected_target.x
            t.transform.translation.y = self.selected_target.y
            t.transform.translation.z = self.selected_target.z
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            self.br.sendTransform(t)

            rospy.Rate(5.0).sleep()
            
            tfIntake = self.tfBuffer.lookup_transform('intake', 'qbo1', rospy.Time())

            print(tfIntake)

            rospy.Rate(20.0).sleep()



if __name__ == '__main__':
    rospy.init_node('mainEngine', anonymous=True)
    rospy.loginfo("mainEngine node started")
    rate = rospy.Rate(10) # 10hz
    
    try:
        mainEngine = MainEngine()
    except rospy.ROSInterruptException:
        pass

    while not rospy.is_shutdown():
        mainEngine.run()   
        rate.sleep()
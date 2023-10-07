#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from vision.msg import objectDetectionArray, objectDetection
from geometry_msgs.msg import Twist, Point
from main_engine.srv import MechanismCommand, MechanismCommandResponse

X_TARGET = "X_TARGET"
Y_TARGET = "Y_TARGET"

class FollowCube:
    def __init__(self):
        self.target_success = False
        self.selected_target = objectDetection()
        self.dis_to_intake = 0.10
        self.tolerance = 0.04
        self.xkP = 0.15
        self.xkD = 2.0
        self.last_error_x = 0
        self.ykP = 1.0
        self.state = X_TARGET


        rospy.Subscriber('/vision/color_detect', objectDetectionArray, self.colorDetectCb)
        #rospy.Subscriber('/vision/aruco_detect', objectDetectionArray, self.arucoDetectCb)
        #rospy.Subscriber('/vision/letter_detect', objectDetectionArray, self.lettersDetectCb)
        #self.flag_pub = rospy.Publisher('/flag', Bool, queue_size=10)
        rospy.Subscriber('/intake_presence', Bool, self.intakePresenceCb)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.run()

    def intakePresenceCb(self, data):
        if data.data:
            msg = Twist()
            self.cmd_vel_pub.publish(msg)
            rospy.Rate(0.5).sleep()
            self.target_success = False
            self.selected_target = objectDetection()

    def colorDetectCb(self, data):
        if not self.target_success:
            sz = len(data.detections)
            if sz == 0:
                return
            point_x_min_id = 0
            y_lowest = data.detections[0].ymin
            y_lowest_id = 0
            for i in range(sz):
                if data.detections[i].ymin - y_lowest >= 60:
                    y_lowest = data.detections[i].ymin
                    point_x_min_id = i
                elif abs(data.detections[i].ymin - y_lowest) < 60:
                    if( abs(data.detections[i].point3D.x) < abs(data.detections[point_x_min_id].point3D.x)):
                        point_x_min_id = i

            self.target_success = True
            self.selected_target = data.detections[point_x_min_id]
            rospy.wait_for_service('/intake')
            try:
                intake = rospy.ServiceProxy('/intake', MechanismCommand)
                intake(1)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
        
        if self.target_success:
            # check if target is still in sight, based on the bounding pixels, then obtain the target's 3D position
            sz = len(data.detections)
            if sz == 0:
                return
            for i in range(sz):
                if data.detections[i].labelText == self.selected_target.labelText: #and abs(data.detections[i].point3D.x - self.selected_target.point3D.x) < 0.15 and abs(data.detections[i].point3D.z - self.selected_target.point3D.z) < 0.15:
                    self.selected_target = data.detections[i]
                    return
                
    def run(self):
        if self.target_success:
            msg = Twist()
            print(self.selected_target.point3D.x)
            if self.state == X_TARGET:
                p_offset = - self.xkP * self.selected_target.point3D.x
                d_offset = - self.xkD * (self.selected_target.point3D.x - self.last_error_x)
                print(f"P: {p_offset}, D: {d_offset}")
                msg.linear.y = - 0.15 * self.selected_target.point3D.x / abs(self.selected_target.point3D.x) + p_offset + d_offset
                self.last_error_x = self.selected_target.point3D.x
                if abs(self.selected_target.point3D.x) < self.tolerance:
                    self.state = Y_TARGET
            elif self.state == Y_TARGET:
                msg.linear.x = max( (self.selected_target.cy - 0.5) * self.ykP, 0.125)
                p_offset = - self.xkP * self.selected_target.point3D.x
                d_offset = - self.xkD * (self.selected_target.point3D.x - self.last_error_x)
                print(f"P: {p_offset}, D: {d_offset}")
                msg.linear.y = - 0.15 * self.selected_target.point3D.x / abs(self.selected_target.point3D.x) + p_offset + d_offset
                self.last_error_x = self.selected_target.point3D.x
            self.cmd_vel_pub.publish(msg)
                

if __name__ == '__main__':
    rospy.init_node('follow_cube')
    fc = FollowCube()
    while not rospy.is_shutdown():
        fc.run()
        rospy.Rate(10).sleep()
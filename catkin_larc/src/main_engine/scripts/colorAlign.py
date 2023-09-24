#!/usr/bin/env python3
#encoding=utf-8
#wait for message

import rospy
from std_msgs.msg import Bool, Int32, Float64, String
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from vision.msg import objectDetection, objectDetectionArray
from vision.srv import DetectColorPattern, DetectColorPatternResponse
from enum import Enum

#import sys

#sys.path.append(str(pathlib.Path(__file__).parent) + '/../include')

FIND_PATTERN = "find_pattern_state"

class MainEngine:
    def __init__(self):
        # publishers
        self.pubCmdVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # subscribers
        self.subOdom = rospy.Subscriber('/odom', Odometry, self.odomCallback)
        #self.subColorDetect = rospy.Subscriber('/color_detect', objectDetectionArray, self.colorDetectCallback)

        # variables
        self.current_time = rospy.Time.now()
        self.state_time = rospy.Time.now()
        self.state = FIND_PATTERN
        self.odom = Odometry()
        self.static_color_seq = "GBYRYBG" # static color sequence
        self.xSquare = 0
        self.color_sequence_detected = False

        self.targetReached = False
        self.targetX = 0
        self.targetY = 0
        self.posTolerance = 0.02
        self.iter = 0
        self.rotateCount = 0

    def odomCallback(self, data):
        self.odom = data

    def colorDetectCallback(self, data):
        sz = len(data.detections)
        if sz == 0:
            return
    
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
            if abs(data.detections[i].ymin - y_min_first) >= 80 or abs(data.detections[i].xmin - x_last_max) >= 70:
                continue
            color_seq += color2Letter[ data.detections[i].labelText ]

            x_last_max = data.detections[i].xmax
            if( abs(data.detections[i].point3D.x) < abs(data.detections[point_x_min_id].point3D.x)):
                point_x_min_id = i

        #check if subsequence
        #print(color_seq)
        if color_seq in self.static_color_seq and len(color_seq) >= 3:
            rospy.loginfo("Color sequence detected: " + color_seq)
            self.color_sequence_detected = True
            #get square from closer point x and adjacents
            x_square_label = color2Letter[ data.detections[point_x_min_id].labelText ]
            x_square_cont = ""
            if point_x_min_id > 0:
                x_square_cont = color2Letter[ data.detections[point_x_min_id - 1].labelText ] + x_square_label
            else:
                x_square_cont = x_square_label + color2Letter[ data.detections[point_x_min_id + 1].labelText ]

            if x_square_label == "G" and x_square_cont == "GB":
                self.xSquare = 7
            elif x_square_label == "B" and (x_square_cont == "GB" or x_square_cont == "BY"):
                self.xSquare = 6
            elif x_square_label == "Y" and (x_square_cont == "BY" or x_square_cont == "YR"):
                self.xSquare = 5
            elif x_square_label == "R":
                self.xSquare = 4
            elif x_square_label == "Y" and (x_square_cont == "RY" or x_square_cont == "YB"):
                self.xSquare = 3
            elif x_square_label == "B" and (x_square_cont == "YB" or x_square_cont == "BG"):
                self.xSquare = 2
            elif x_square_label == "G" and x_square_cont == "BG":
                self.xSquare = 1

            print( "x_square: " + str(self.xSquare) )
            

    def driveToTarget(self):
        if abs( self.odom.pose.pose.position.x - self.targetX ) <= self.posTolerance and abs( self.odom.pose.pose.position.y - self.targetY ) <= self.posTolerance:
            self.targetReached = True
            rospy.loginfo("Target reached")
            cmd_vel = Twist()
            cmd_vel.linear.x = 0
            cmd_vel.linear.y = 0
            cmd_vel.linear.z = 0
            cmd_vel.angular.x = 0
            cmd_vel.angular.y = 0
            cmd_vel.angular.z = 0
            self.pubCmdVel.publish(cmd_vel)
        else:
            print( "current X: " +str(self.odom.pose.pose.position.x) + " current Y: " + str(self.odom.pose.pose.position.y) )
            cmd_vel = Twist()
            cmd_vel.linear.x = (self.targetX - self.odom.pose.pose.position.x) * 2.5
            cmd_vel.linear.y = (self.targetY - self.odom.pose.pose.position.y) * 2.5
            cmd_vel.linear.z = 0
            cmd_vel.angular.x = 0
            cmd_vel.angular.y = 0
            cmd_vel.angular.z = 0
            self.pubCmdVel.publish(cmd_vel)


    def run(self):
        self.current_time = rospy.Time.now()
        
        if self.state == FIND_PATTERN:
            # gira 90 grados

            rospy.wait_for_service('detect_color_pattern')
            try:
                detect_color_pattern = rospy.ServiceProxy('detect_color_pattern', DetectColorPattern)
                self.x

        if self.state == State.START:
            if (self.current_time - self.state_time).to_sec() > 4:
                self.driveSpin(1)
                self.state_time = self.current_time
            if self.color_sequence_detected:
                self.state = State.FWD1
                self.color_sequence_detected = False
                self.state_time = self.current_time
                rospy.loginfo("State changed to ROTATE_POSITIONING")
                self.driveSpin(-1)
                self.state = State.ROTATE1
                self.iter = 0

        if self.state == State.ROTATE1:
            if (self.current_time - self.state_time).to_sec() > 3:
                self.driveSpin(1)
                self.state_time = self.current_time
                self.iter += 1
            if self.iter == 2:
                self.state = State.FWD1


        #jasdkjasdjkasd
    
    def setTarget(self, x, y):
        self.targetX = x
        self.targetY = y

    def driveSpin(self, dir):
        #publish to cmd_vel with angular velocity Z = dir * 0.5
        cmd_vel = Twist()
        cmd_vel.angular.z = -dir * 0.5
        self.pubCmdVel.publish(cmd_vel)

    def driveFwdToLine(self):
        #publish to cmd_vel with linear velocity X = 0.5
        cmd_vel = Twist()
        cmd_vel.linear.y = 0.4
        self.pubCmdVel.publish(cmd_vel)
    
    def driveFwdTimed(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.4
        self.pubCmdVel.publish(cmd_vel)

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
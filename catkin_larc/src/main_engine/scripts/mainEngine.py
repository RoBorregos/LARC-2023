#!/usr/bin/env python3
#encoding=utf-8
#wait for message

import rospy
from std_msgs.msg import Bool, Int32, Float64, String
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from vision.msg import objectDetection, objectDetectionArray
from enum import Enum

#import sys

#sys.path.append(str(pathlib.Path(__file__).parent) + '/../include')

class State(Enum):
    START = 1
    FWD1 = 2
    WAIT1 = 3
    ELE1 = 4
    STACK1 = 5
    LINES_FWD = 6
    OUT_EL = 7
    OUT_WAREHOUSE = 8
    OUT_EL_POS = 9
    OUT_FWD = 10
    OUT_INTAKE = 11
    OUT_WAIT = 12
    OUT_RESET = 13
    ROTATE_180 = 14
    FWD_COLOR = 15
    FINISH = 16

class MainEngine:
    def __init__(self):
        # publishers
        self.pubCmdVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pubIntake = rospy.Publisher('/intake', Int32, queue_size=10)
        self.pubElevator = rospy.Publisher('/elevator', Int32, queue_size=10)
        self.pubWarehouse = rospy.Publisher('/warehouse', Int32, queue_size=10)
        self.pubResetOdom = rospy.Publisher('/reset_odom', Bool, queue_size=10)
        self.pubGlobalSetpoint = rospy.Publisher('/global_setpoint', Bool, queue_size=10)

        # subscribers
        self.subOdom = rospy.Subscriber('/odom', Odometry, self.odomCallback)
        self.subColorDetect = rospy.Subscriber('/color_detect', objectDetectionArray, self.colorDetectCallback)

        # variables
        self.current_time = rospy.Time.now()
        self.state_time = rospy.Time.now()
        self.state = State.START
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
    
        for i in range(sz):
            if abs(data.detections[i].ymin - y_min_first) >= 80 or abs(data.detections[i].xmin - x_last_max) >= 70:
                continue
            if data.detections[i].labelText == "rojo":
                color_seq += "R"
            elif data.detections[i].labelText == "verde":
                color_seq += "G"
            elif data.detections[i].labelText == "azul":
                color_seq += "B"
            elif data.detections[i].labelText == "amarillo":
                color_seq += "Y"

            x_last_max = data.detections[i].xmax
            if( abs(data.detections[i].point3D.x) < abs(data.detections[point_x_min_id].point3D.x)):
                point_x_min_id = i

        #check if subsequence
        #print(color_seq)
        if color_seq in self.static_color_seq and len(color_seq) >= 3:
            rospy.loginfo("Color sequence detected: " + color_seq)
            self.color_sequence_detected = True
            #get square from closer point x and adjacents
            x_square_label = data.detections[point_x_min_id].labelText
            x_square_cont = ""
            if point_x_min_id > 0:
                x_square_cont = data.detections[point_x_min_id - 1].labelText + x_square_label
            else:
                x_square_cont = x_square_label + data.detections[point_x_min_id + 1].labelText

            if x_square_label == "verde" and x_square_cont == "verdeazul":
                self.xSquare = 7
            elif x_square_label == "azul" and (x_square_cont == "verdeazul" or x_square_cont == "azulamarillo"):
                self.xSquare = 6
            elif x_square_label == "amarillo" and (x_square_cont == "azulamarillo" or x_square_cont == "rojoamarillo"):
                self.xSquare = 5
            elif x_square_label == "rojo":
                self.xSquare = 4
            elif x_square_label == "amarillo" and (x_square_cont == "rojoamarillo" or x_square_cont == "amarilloazul"):
                self.xSquare = 3
            elif x_square_label == "azul" and (x_square_cont == "amarilloazul" or x_square_cont == "azulverde"):
                self.xSquare = 2
            elif x_square_label == "verde" and x_square_cont == "azulverde":
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

        if self.state == State.START:
            if (self.current_time - self.state_time).to_sec() > 3:
                self.state = State.FWD1
                tx = 0.11 + self.iter*0.11
                self.targetReached = False
                self.setTarget(tx, 0)
                self.pubIntake.publish(1)
                self.state_time = self.current_time
                rospy.loginfo("State changed to FWD1")

        elif self.state == State.FWD1:
            self.driveToTarget()
            if self.targetReached or (self.current_time - self.state_time).to_sec() > 8:
                self.state = State.WAIT1
                self.state_time = self.current_time
                rospy.loginfo("State changed to FWD2")

        elif self.state == State.WAIT1:
            if (self.current_time - self.state_time).to_sec() > 2:
                self.state = State.ELE1
                if self.iter == 0 or self.iter == 2:
                    self.pubElevator.publish(3)
                elif self.iter == 1 or self.iter == 3:
                    self.pubElevator.publish(2)
                self.state_time = self.current_time
                rospy.loginfo("State changed to FWD1")

        elif self.state == State.ELE1:
            if (self.current_time - self.state_time).to_sec() > 3:
                self.state = State.STACK1
                self.pubIntake.publish(2)
                self.state_time = self.current_time
                rospy.loginfo("State changed to SET_SETPOINT")

        elif self.state == State.STACK1:
            if (self.current_time - self.state_time).to_sec() > 3:
                self.pubElevator.publish(1)
                self.state_time = self.current_time
                self.iter += 1
                if self.iter < 4:
                    self.state = State.START
                else:
                    self.state = State.LINES_FWD
                    self.targetReached = False
                    self.setTarget(0.7, 0)
                    self.iter = 0
                rospy.loginfo("State changed to START")
        
        elif self.state == State.LINES_FWD:
            self.driveToTarget()
            if self.targetReached or (self.current_time - self.state_time).to_sec() > 10:
                self.pubElevator.publish(5)
                self.state = State.OUT_EL
                rospy.loginfo("State changed to LINE FOLLOW")

        elif self.state == State.OUT_EL:
            if (self.current_time - self.state_time).to_sec() > 4:
                self.state_time = self.current_time
                if self.iter == 0 or self.iter == 2:
                    self.pubWarehouse.publish(3)
                elif self.iter == 1 or self.iter ==3:
                    self.pubWarehouse.publish(2)
                self.state = State.OUT_WAREHOUSE
                rospy.loginfo("State changed to WAREHOUSE")
        
        elif self.state == State.OUT_WAREHOUSE:
            if (self.current_time - self.state_time).to_sec() > 4:
                self.state_time = self.current_time
                self.pubIntake.publish(3)
                self.state = State.OUT_EL_POS
                rospy.loginfo("State changed to ELEVATOR PLACE")

        elif self.state == State.OUT_EL_POS:
            if (self.current_time - self.state_time).to_sec() > 4:
                self.state_time = self.current_time
                if self.iter == 0:
                    self.pubElevator.publish(8)
                if self.iter == 1:
                    self.pubElevator.publish(9)
                if self.iter == 2:
                    self.pubElevator.publish(10)
                if self.iter == 3:
                    self.pubElevator.publish(1)
                self.state = State.OUT_FWD
                rospy.loginfo("State changed to FWD")
        
        elif self.state == State.OUT_FWD:
            if (self.current_time - self.state_time).to_sec() > 4:
                self.state_time = self.current_time
                if self.iter == 3:
                    self.pubIntake.publish(4)
                    self.state = State.FINISH
                    rospy.loginfo("State changed to FINISH")
                else:
                    self.targetReached = False
                    self.setTarget(0.85, 0)
                    self.state = State.OUT_INTAKE
                    rospy.loginfo("State changed to INTAKE")

        elif self.state == State.OUT_INTAKE:
            self.driveToTarget()
            if self.targetReached or (self.current_time - self.state_time).to_sec() > 8:
                self.state_time = self.current_time
                self.pubIntake.publish(4)
                self.state = State.OUT_WAIT
                rospy.loginfo("State changed to OUT WAIT")
        
        elif self.state == State.OUT_WAIT:
            if (self.current_time - self.state_time).to_sec() > 4:
                self.state_time = self.current_time
                self.targetReached = False
                self.setTarget(0.7, 0)
                self.state = State.OUT_RESET
                rospy.loginfo("State changed to RESET")

        elif self.state == State.OUT_RESET:
            self.driveToTarget()
            if self.targetReached or (self.current_time - self.state_time).to_sec() > 8:
                self.state_time = self.current_time
                self.iter += 1
                self.state = State.OUT_EL
                if self.iter == 1:
                    self.pubElevator.publish(6)
                elif self.iter == 2:
                    self.pubElevator.publish(5)
                elif self.iter >= 3:
                    self.rotateCount = 0
                    self.state = State.FINISH
                rospy.loginfo("State changed to OUT ROTATE")

        elif self.state == State.ROTATE_180:
            if (self.current_time - self.state_time).to_sec() > 5:
                self.rotateCount += 1
                if( self.rotateCount >= 3 ):
                    self.targetReached = False
                    self.setTarget(1.7, 0)
                    self.state = State.FWD_COLOR
                    rospy.loginfo("State changed to COLOR FWD")
                else:
                    self.driveSpin(1)
                self.state_time = self.current_time

        elif self.state == State.FWD_COLOR:
            self.driveToTarget()
            if self.targetReached or (self.current_time - self.state_time).to_sec() > 15:
                self.state_time = self.current_time
                self.pubIntake.publish(6)
                self.state = State.OUT_EL
                rospy.loginfo("State changed to OUT EL COLOR")


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
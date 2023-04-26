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
    SEARCH_HEADING = 0
    WAIT4ROTATION = 1
    SET_SETPOINT = 2
    ROTATE_POSITIONING = 3
    POSITION_FOR_SWEEP = 4
    ROTATE_PICK = 5
    PICK = 6
    MOVE2PICK = 7
    DROP = 8
    FINISH = 9

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
        self.state = State.SEARCH_HEADING
        self.odom = Odometry()
        self.static_color_seq = "GBYRYBG" # static color sequence
        self.xSquare = 0
        self.color_sequence_detected = False

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
            if abs(data.detections[i].ymin - y_min_first) >= 50 or abs(data.detections[i].xmin - x_last_max) >= 50:
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
            

    def run(self):
        self.current_time = rospy.Time.now()

        if self.state == State.SEARCH_HEADING:
            if (self.current_time - self.state_time).to_sec() > 4:
                self.driveSpin(1)
                self.state_time = self.current_time
            if self.color_sequence_detected:
                self.state = State.WAIT4ROTATION
                self.state_time = self.current_time
                rospy.loginfo("State changed to WAIT4ROTATION")

        elif self.state == State.WAIT4ROTATION:
            if (self.current_time - self.state_time).to_sec() > 4:
                self.state = State.SET_SETPOINT
                self.pubGlobalSetpoint.publish(True)
                self.state_time = self.current_time
                rospy.loginfo("State changed to SET_SETPOINT")
        
        elif self.state == State.SET_SETPOINT:
            if (self.current_time - self.state_time).to_sec() > 3:
                self.driveSpin(-1)
                self.state = State.ROTATE_POSITIONING
                self.state_time = self.current_time
                rospy.loginfo("State changed to ROTATE_POSITIONING")

        elif self.state == State.ROTATE_POSITIONING:
            if (self.current_time - self.state_time).to_sec() > 4:
                self.state = State.POSITION_FOR_SWEEP
                self.state_time = self.current_time
                rospy.loginfo("State changed to POSITION_FOR_SWEEP")
                self.driveFwdToLine()

        elif self.state == State.POSITION_FOR_SWEEP:
            if (self.current_time - self.state_time).to_sec() > 4:
                self.state = State.ROTATE_PICK
                self.state_time = self.current_time
                rospy.loginfo("State changed to ROTATE_PICK")
                self.driveSpin(-1)
        
        elif self.state == State.ROTATE_PICK:
            if(self.current_time - self.state_time).to_sec() > 4:
                self.pubIntake.publish(1)
                self.state = State.PICK
                self.state_time = self.current_time
                rospy.loginfo("State changed to PICK")

        elif self.state == State.PICK:
            if(self.current_time - self.state_time).to_sec() > 4:
                self.driveFwdTimed()
                self.state = State.MOVE2PICK
                self.state_time = self.current_time
                rospy.loginfo("State changed to MOVE2PICK")
        
        elif self.state == State.MOVE2PICK:
            if(self.current_time - self.state_time).to_sec() > 4:
                self.pubElevator.publish(2)
                self.state = State.DROP
                self.state_time = self.current_time
                rospy.loginfo("State changed to DROP")
        
        elif self.state == State.DROP:
            if(self.current_time - self.state_time).to_sec() > 4:
                self.pubIntake.publish(4)
                self.state = State.FINISH
                self.state_time = self.current_time
                rospy.loginfo("State changed to FINISH")


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
#!/usr/bin/env python3

import rospy
import tf_conversions
import tf2_ros
import actionlib
from vision.msg import objectDetection, objectDetectionArray
from vision.srv import DetectColorPattern, DetectColorPatternResponse
import geometry_msgs.msg
from std_msgs.msg import Int32, Bool, Float32
from geometry_msgs.msg import Point, Pose, Twist
from main_engine.srv import MechanismCommand, MechanismCommandResponse
from nav_main.msg import Drive2TargetAction, Drive2TargetGoal, Drive2TargetResult, Drive2TargetFeedback
from nav_msgs.msg import Odometry

FIND_INIT_POS = "find_init_pos"
X_MOVE_TO_PICK_POS1 = "x_move_to_pick_pos1"
Y_MOVE_TO_PICK_POS = "y_move_to_pick_pos1"
X_MOVE_TO_PICK_POS2 = "x_move_to_pick_pos2"
ROTATE_TO_CUBES = "rotate_to_cubes"

PICK_CUBE_TARGET = "pick_cube_target"
DRIVE_TO_TARGET = "drive_to_target"
PICK_CUBE = "pick_cube"
ACTIVATE_ELEVATOR_TO_STORE = "activate_elevator_to_store"
STORE_CUBE = "store_cube"
MOVE_BACK_TO_DETECT = "move_back_to_detect"
RESET_ELEVATOR = "reset_elevator"
PARTIAL_PICK_FINISH = "partial_pick_finish"
PICK_FINISH = "pick_finish"
PARTIAL_FINISH = "partial_finish"
FINISH = "finish"

X_TARGET = "x_target"
Y_TARGET = "y_target"

MOVE_BACK_TO_DETECT_LIMIT = 0.5 # 60cm maximum to go back from cube
MOVE_LEFT_TO_DETECT_LIMIT = 1.0 # 1m maximum to go left when not finding a cube

NUMBER_OF_CUBES = 12



class MainEngine:
    def __init__(self):
        rospy.loginfo("MainEngine init")
        self.current_time = rospy.Time.now()
        self.state = FIND_INIT_POS
        self.target_success = [False, False, False]
        self.detected_targets= [objectDetection(), objectDetection(), objectDetection()]
        self.selected_target = objectDetection()
        self.dis_to_intake = 0.10
        self.tolerance = 0.02
        self.xkP = 0.15
        self.xkD = 2.0
        self.last_error_x = 0
        self.ykP = 0.35
        self.target_nav_back = 0.0

        self.search_tries = 0

        self.categoryDict = { 'color': 0, 'aruco': 1, 'letter': 2 }
        self.color_stack = []
        self.aruco_stack = []
        self.letter_stack = []
        self.nav_odom = Odometry()

        self.x_tile = 0
        self.y_tile = 0
        self.current_angle = 0.0
        self.color_target_to_align = ""
        self.color_target_dis = 5000

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.Subscriber('/intake_presence', Bool, self.intakePresenceCb)
        self.intakePresenceData = False
        rospy.Subscriber('/odom', Odometry, self.odomCb)
        self.flag_pub = rospy.Publisher('/flag', Bool, queue_size=10)

        #rospy.Subscriber('/odom', Odometry, self.odomCb)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rotate_pub = rospy.Publisher('/rotate', Float32, queue_size=10)
        rospy.Subscriber('/vision/color_detect', objectDetectionArray, self.colorDetectCb)

    def odomCb(self, data):
        self.nav_odom = data

    def colorDetectCb(self, data):
        if self.state == X_MOVE_TO_PICK_POS1 or self.state == X_MOVE_TO_PICK_POS2:
            sz = len(data.detections)
            if sz == 0:
                return
            y_min_first = data.detections[0].ymin
            point_x_min_id = 0

            for i in range(sz):
                if abs(data.detections[i].ymin - y_min_first) >= 120 or data.detections[i].labelText != self.color_target_to_align:
                    continue
                if( abs(data.detections[i].point3D.x) < abs(data.detections[point_x_min_id].point3D.x)):
                    point_x_min_id = i
            
            self.color_target_dis = data.detections[point_x_min_id].point3D.x

    def intakePresenceCb(self, data):
        self.intakePresenceData = data.data
        if data.data and self.state == Y_TARGET:
            msg = Twist()
            self.cmd_vel_pub.publish(msg)
            rospy.Rate(0.5).sleep()
            self.state = PICK_CUBE
            #self.flag_pub.publish(False)

    def mechanismCommandSvr(self, srv, command):
        rospy.wait_for_service(srv)
        try:
            client = rospy.ServiceProxy(srv, MechanismCommand)
            client(command)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def run(self):
        self.current_time = rospy.Time.now()

        if self.state == FIND_INIT_POS:
            self.flag_pub.publish(True)
            print( self.current_angle )
            msg_f = Float32()
            msg_f.data = float(self.current_angle)

            self.rotate_pub.publish( msg_f )
            rospy.sleep(10)
            rospy.wait_for_service('/detect_color_pattern')
            try:
                client = rospy.ServiceProxy('/detect_color_pattern', DetectColorPattern)
                resp = client()
                if resp.tileX != 0 and resp.tileY != 0:
                    self.x_tile = resp.tileX
                    self.y_tile = resp.tileY
                    rospy.loginfo(f"Tile X: {self.x_tile}")
                    rospy.loginfo(f"Tile Y: {self.y_tile}")
                    if self.y_tile > 4 and (self.x_tile == 2 or self.x_tile == 3 or self.x_tile == 5 or self.x_tile == 6):
                        self.state = X_MOVE_TO_PICK_POS1
                    elif self.y_tile > 1:
                        self.state = Y_MOVE_TO_PICK_POS
                    else:
                        self.state = X_MOVE_TO_PICK_POS2
                else:
                    self.current_angle += 90.0
            except rospy.ServiceException as e:
                print("Pattern call failed: %s"%e)
            

        elif self.state == X_MOVE_TO_PICK_POS1:
            rospy.loginfo("Moving to x pick pos 1")
            self.color_target_to_align = "rojo"
            current_odom_y = self.nav_odom.pose.pose.position.y
            target_odom_y = current_odom_y + (4 - self.x_tile) * 0.3
            sign = (4-self.x_tile) / abs(4-self.x_tile)
            print(f"Current odom x: {current_odom_y}")
            print(f"Max odom x: {target_odom_y}")
            
            # While no cube is detected and the limit has not been reached, move back
            while abs(current_odom_y - target_odom_y) > 0.1:
                msg = Twist()
                msg.linear.y = 0.2 * sign
                self.cmd_vel_pub.publish(msg)
                current_odom_y = self.nav_odom.pose.pose.position.y
            
            while self.color_target_dis == 5000:
                print("Waiting for color target dis")
                pass

            while abs(self.color_target_dis) > self.tolerance:
                msg = Twist()
                msg.linear.y = - 0.11 * self.color_target_dis / abs(self.color_target_dis)
                self.cmd_vel_pub.publish(msg)
                print(f"Color target dis: {self.color_target_dis}")

            msg = Twist()
            self.cmd_vel_pub.publish(msg) # stop
            self.color_target_dis = 5000
            rospy.sleep(1)
            self.x_tile = 4
            self.state = Y_MOVE_TO_PICK_POS

        elif self.state == Y_MOVE_TO_PICK_POS:
            rospy.loginfo("Moving to y pick pos")
            current_odom_x = self.nav_odom.pose.pose.position.x
            target_odom_x = current_odom_x + (self.y_tile-1) * 0.33
            print(f"Current odom y: {current_odom_x}")
            print(f"Max odom y: {target_odom_x}")
            
            # While no cube is detected and the limit has not been reached, move back
            while target_odom_x - current_odom_x > 0.05:
                msg = Twist()
                msg.linear.x = 0.2
                self.cmd_vel_pub.publish(msg)
                current_odom_x = self.nav_odom.pose.pose.position.x
            
            msg = Twist()
            self.cmd_vel_pub.publish(msg) # stop
            rospy.sleep(1)
            self.y_tile = 1
            self.state = X_MOVE_TO_PICK_POS2

        elif self.state == X_MOVE_TO_PICK_POS2:
            rospy.loginfo("Moving to x pick pos 2")
            self.color_target_to_align = "azul"
            current_odom_y = self.nav_odom.pose.pose.position.y
            target_odom_y = current_odom_y + (6 - self.x_tile) * 0.3
            sign = (6-self.x_tile) / abs(6-self.x_tile)
            print(f"Current odom y: {current_odom_y}")
            print(f"Max odom y: {target_odom_y}")
            
            # While no cube is detected and the limit has not been reached, move back
            while abs(current_odom_y - target_odom_y) > 0.1:
                msg = Twist()
                msg.linear.y = 0.2 * sign
                self.cmd_vel_pub.publish(msg)
                current_odom_y = self.nav_odom.pose.pose.position.y
            
            while self.color_target_dis == 5000:
                print("Waiting for color target dis")
                pass

            while abs(self.color_target_dis) > self.tolerance:
                msg = Twist()
                msg.linear.y = - 0.11 * self.color_target_dis / abs(self.color_target_dis)
                self.cmd_vel_pub.publish(msg)

            msg = Twist()
            self.cmd_vel_pub.publish(msg) # stop
            self.color_target_dis = 5000
            rospy.sleep(1)
            self.x_tile = 6
            self.state = ROTATE_TO_CUBES

        elif self.state == ROTATE_TO_CUBES:
            self.current_angle += 180
            if self.current_angle > 270:
                self.current_angle -= 360

            print( self.current_angle )
            msg_f = Float32()
            msg_f.data = float(self.current_angle)

            self.rotate_pub.publish( msg_f )
            rospy.sleep(3)
            self.state = FINISH

        elif self.state == FINISH:
                rospy.loginfo("Finish")
                rospy.loginfo(f"Color stack: {self.color_stack}")
                rospy.loginfo(f"Aruco stack: {self.aruco_stack}")
                rospy.loginfo(f"Letter stack: {self.letter_stack}")
                rospy.loginfo("Finished")
                while (1):
                    pass


        
if __name__ == '__main__':
    rospy.init_node('mainEngine', anonymous=True)
    rospy.loginfo("mainEngine node started")
    rate = rospy.Rate(10) # 10hz

    """rospy.wait_for_service('/reset_teensy')
    try:
        client = rospy.ServiceProxy('/reset_teensy', MechanismCommand)
        client(0)
    except rospy.ServiceException as e:
        print("Service reset teensy failed: %s"%e)"""
    
    try:
        mainEngine = MainEngine()
    except rospy.ROSInterruptException:
        exit() 

    while not rospy.is_shutdown():
        mainEngine.run()   
        rate.sleep()
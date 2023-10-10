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

PICK_UNLOAD_TARGET = "pick_unload_target"
FIND_UPLOAD_TILE = "find_upload_tile"
MOVE_Y_UPLOAD_TILE = "move_y_upload_tile"
MOVE_X_UPLOAD_TILE = "move_x_upload_tile"
UNLOAD_CUBE = "unload_cube"
DRIVE_TO_COLOR_AREA = "drive_to_color_area"
MOVE_BACK_TO_UNLOAD_COLOR = "move_back_to_unload_color"
PARTIAL_FINISH = "partial_finish"

FIND_TILE_SHELF = "find_tile_shelf"
MOVE_Y_SHELF_INIT = "move_y_shelf_init"
MOVE_X_SHELF_INIT = "move_x_shelf_init"
GET_SHELF_UNLOAD_TARGET = "get_shelf_unload_target"
MOVE_X_SHELF_UNLOAD = "move_x_shelf_unload"
MOVE_Y_SHELF_UNLOAD = "move_y_shelf_unload"


FINISH = "finish"

X_TARGET = "x_target"
Y_TARGET = "y_target"

ELEVATOR_SMALLSTEPDOWN_COMMAND = 50

MOVE_BACK_TO_DETECT_LIMIT = 0.5 # Xcm maximum to go back from cube
MOVE_BACK_TO_DETECT_MINIMUM = 0.45 # Xcm minimum to go back from cube
MOVE_LEFT_TO_DETECT_LIMIT = 1.0 # 1m maximum to go left when not finding a cube

NUMBER_OF_CUBES = 6



class MainEngine:
    def __init__(self):
        rospy.loginfo("MainEngine init")
        self.current_time = rospy.Time.now()
        self.state = RESET_ELEVATOR
        self.pick_result = PARTIAL_PICK_FINISH
        self.target_success = [False, False, False]
        self.detected_targets= [objectDetection(), objectDetection(), objectDetection()]
        self.selected_target = objectDetection()
        self.dis_to_intake = 0.10
        self.tolerance = 0.03
        self.align_tolerance = 0.05

        self.xkP = 0.17
        self.xkD = 2.0
        self.last_error_x = 0
        self.ykP = 0.4
        self.target_nav_back = 0.0
        self.pick_cube_init_odom = 0.0

        self.speed_nav = 0.5
        self.speed_align = 0.12

        self.search_tries = 0
        self.left_side_pick = False

        self.pick_drive_target_timeout = 0.0
        self.pick_drive_target_timeout_flag = False

        self.categoryDict = { 'color': 0, 'aruco': 1, 'letter': 2 }
        self.color_stack = []
        self.aruco_stack = []
        self.letter_stack = []
        self.nav_odom = Odometry()
        self.selected_target_seen = False
        
        self.x_tile = 0
        self.y_tile = 0
        self.current_angle = 0.0
        self.color_target_to_align = ""
        self.color_target_dis = 5000

        self.category_to_unload = 0
        self.current_shelf = 1
        self.shelf_target = 0
        self.shelf_target_height = 0

        rospy.Subscriber('/vision/color_detect', objectDetectionArray, self.colorDetectCb)
        rospy.Subscriber('/vision/aruco_detect', objectDetectionArray, self.arucoDetectCb)
        rospy.Subscriber('/vision/letter_detect', objectDetectionArray, self.lettersDetectCb)
        rospy.Subscriber('/intake_presence', Bool, self.intakePresenceCb)
        self.intakePresenceData = False
        rospy.Subscriber('/odom', Odometry, self.odomCb)
        self.flag_pub = rospy.Publisher('/flag', Bool, queue_size=10)
        self.flag_selection = rospy.Publisher('/flag_selection', Int32, queue_size=10)
        self.rotate_pub = rospy.Publisher('/rotate_angle', Float32, queue_size=10)

        #rospy.Subscriber('/odom', Odometry, self.odomCb)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.targetPointFbPub = rospy.Publisher('/target_point_fb', Point, queue_size=10)
        self.followCubePub = rospy.Publisher('/follow_cube', Bool, queue_size=10)
        self.driveTargetClient = actionlib.SimpleActionClient("drive_to_target", Drive2TargetAction)

    def odomCb(self, data):
        self.nav_odom = data

    def intakePresenceCb(self, data):
        self.intakePresenceData = data.data
        if data.data and self.state == Y_TARGET:
            msg = Twist()
            self.cmd_vel_pub.publish(msg)
            self.state = PICK_CUBE
            #self.flag_pub.publish(False)

    def cubeTargetsHandler(self, data):
        if self.state == PICK_CUBE_TARGET or self.state == MOVE_BACK_TO_DETECT:
            sz = len(data.detections)
            if sz == 0:
                return
            category_id = self.categoryDict[ data.detections[0].category ]
            point_x_min_id = 0
            y_lowest = (data.detections[0].ymin + data.detections[0].ymax) / 2
            y_lowest_id = 0
            right_half_cube = False
            for i in range(sz):
                # break camera frame in 3, go for the lowest cube with priority the rightmost
                c_y = (data.detections[i].ymin + data.detections[i].ymax) / 2
                if c_y >= y_lowest:
                    y_lowest = c_y
                    point_x_min_id = i
                
                """if data.detections[i].ymax - y_lowest >= 50:
                    y_lowest = data.detections[i].ymax
                    point_x_min_id = i
                elif abs(data.detections[i].ymax - y_lowest) < 50:
                    if( data.detections[i].point3D.x > data.detections[point_x_min_id].point3D.x ):
                        point_x_min_id = i"""
            
            self.target_success[category_id] = True
            self.detected_targets[category_id] = data.detections[point_x_min_id]

        if self.state == X_TARGET or self.state == Y_TARGET:
            # check if target is still in sight, based on the bounding pixels, then obtain the target's 3D position
            sz = len(data.detections)
            if sz == 0:
                self.selected_target_seen = False
                return
            for i in range(sz):
                if data.detections[i].labelText == self.selected_target.labelText:
                    self.selected_target = data.detections[i]
                    self.selected_target_seen = True
                    return
        
    def colorDetectCb(self, data):
        self.cubeTargetsHandler(data)

        if self.state == MOVE_X_UPLOAD_TILE or self.state == MOVE_Y_UPLOAD_TILE or self.state == X_MOVE_TO_PICK_POS1 or self.state == X_MOVE_TO_PICK_POS2 or self.state == MOVE_X_SHELF_INIT or self.state == MOVE_Y_SHELF_INIT or self.state == MOVE_X_SHELF_UNLOAD or self.state == PICK_CUBE_TARGET:
            sz = len(data.detections)
            if sz == 0:
                return
            y_min_first = data.detections[0].ymin
            point_x_min_id = 0
            detected = False

            for i in range(sz):
                if abs(data.detections[i].ymin - y_min_first) >= 120 or data.detections[i].labelText != self.color_target_to_align:
                    continue
                if( abs(data.detections[i].point3D.x) <= abs(data.detections[point_x_min_id].point3D.x)):
                    detected = True
                    point_x_min_id = i
            if not detected:
                return
            
            if self.state == MOVE_X_UPLOAD_TILE or self.state == X_MOVE_TO_PICK_POS1 or self.state == X_MOVE_TO_PICK_POS2 or self.state == MOVE_X_SHELF_INIT or self.state == MOVE_X_SHELF_UNLOAD or self.state == PICK_CUBE_TARGET:
                self.color_target_dis = data.detections[point_x_min_id].point3D.x
            elif self.state == MOVE_Y_UPLOAD_TILE or self.state == MOVE_Y_SHELF_INIT:
                self.color_target_dis = data.detections[point_x_min_id].point3D.z
                
    def arucoDetectCb(self, data):
        self.cubeTargetsHandler(data)

    def lettersDetectCb(self, data):
        self.cubeTargetsHandler(data)

    def mechanismCommandSvr(self, srv, command):
        rospy.wait_for_service(srv)
        try:
            client = rospy.ServiceProxy(srv, MechanismCommand)
            client(command)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def driveNavTarget(self, target_x, target_y, speed):
        current_odom_y = self.nav_odom.pose.pose.position.y
        current_odom_x = self.nav_odom.pose.pose.position.x
        target_odom_y = current_odom_y - target_y
        target_odom_x = current_odom_x + target_x
        sign_y = 1
        sign_x = 1

        rospy.loginfo(f"Moving y (horizontal): {-target_y} m")
        rospy.loginfo(f"Moving x (vertical): {target_x} m")

        while abs(target_odom_x-current_odom_x) > 0.02 and abs(target_odom_y-current_odom_y) > 0.015:
            try:
                sign_y = (target_odom_y-current_odom_y) / abs(target_odom_y-current_odom_y)
                sign_x = (target_odom_x-current_odom_x) / abs(target_odom_x-current_odom_x)
            except:
                sign_y = 1
                sign_x = 1
            msg = Twist()
            msg.linear.y = speed * sign_y
            msg.linear.x = speed * sign_x
            self.cmd_vel_pub.publish(msg)
            current_odom_y = self.nav_odom.pose.pose.position.y
            current_odom_x = self.nav_odom.pose.pose.position.x

        msg = Twist()
        self.cmd_vel_pub.publish(msg) # stop
        self.mechanismCommandSvr('hard_stop', 1)
        rospy.sleep(1)
        rospy.loginfo("Nav target drive reached")


    def run(self):
        self.current_time = rospy.get_time()
        
        if self.state == FIND_INIT_POS:
            self.mechanismCommandSvr('hard_stop', 1)
            self.flag_pub.publish(True)
            self.flag_selection.publish(0)
            print( self.current_angle )
            msg_f = Float32()
            msg_f.data = float(self.current_angle)

            self.rotate_pub.publish( msg_f )
            rospy.sleep(3)
            rospy.wait_for_service('/detect_color_pattern')
            try:
                client = rospy.ServiceProxy('/detect_color_pattern', DetectColorPattern)
                resp = client()
                if resp.tileX != 0 and resp.tileY != 0:
                    self.x_tile = resp.tileX
                    self.y_tile = resp.tileY
                    rospy.loginfo(f"Tile X: {self.x_tile}")
                    rospy.loginfo(f"Tile Y: {self.y_tile}")
                    if self.y_tile > 4:
                        self.state = X_MOVE_TO_PICK_POS1
                    elif self.y_tile > 1:
                        self.state = Y_MOVE_TO_PICK_POS
                    else:
                        self.state = X_MOVE_TO_PICK_POS2
                else:
                    self.current_angle += 90.0
                    if self.current_angle > 270:
                        self.current_angle -= 360
            except rospy.ServiceException as e:
                print("Pattern call failed: %s"%e)

        elif self.state == X_MOVE_TO_PICK_POS1:
            rospy.loginfo("Moving to x pick pos 1")
            self.color_target_to_align = "rojo"
            current_odom_y = self.nav_odom.pose.pose.position.y
            target_odom_y = current_odom_y + (4 - self.x_tile) * 0.27
            sign = 1
            print(f"Current odom x: {current_odom_y}")
            print(f"Max odom x: {target_odom_y}")
            
            # While no cube is detected and the limit has not been reached, move back
            while abs(current_odom_y - target_odom_y) > 0.07:
                try:
                    sign = (target_odom_y-current_odom_y) / abs(target_odom_y-current_odom_y)
                except:
                    sign = 1
                msg = Twist()
                msg.linear.y = self.speed_nav * sign
                self.cmd_vel_pub.publish(msg)
                current_odom_y = self.nav_odom.pose.pose.position.y
            
            while self.color_target_dis == 5000:
                print("Waiting for color target dis")
                pass

            while abs(self.color_target_dis) > self.align_tolerance:
                msg = Twist()
                msg.linear.y = - self.speed_align * self.color_target_dis / abs(self.color_target_dis)
                self.cmd_vel_pub.publish(msg)
                #print(f"Color target dis: {self.color_target_dis}")

            msg = Twist()
            self.cmd_vel_pub.publish(msg) # stop
            self.color_target_dis = 5000
            rospy.sleep(1)
            self.x_tile = 4
            self.state = Y_MOVE_TO_PICK_POS

        elif self.state == Y_MOVE_TO_PICK_POS:
            rospy.loginfo("Moving to y pick pos")
            current_odom_x = self.nav_odom.pose.pose.position.x
            target_odom_x = current_odom_x + (self.y_tile-2) * 0.29
            print(f"Current odom y: {current_odom_x}")
            print(f"Max odom y: {target_odom_x}")
            
            # While no cube is detected and the limit has not been reached, move back
            while target_odom_x - current_odom_x > 0.05:
                msg = Twist()
                msg.linear.x = self.speed_nav 
                self.cmd_vel_pub.publish(msg)
                current_odom_x = self.nav_odom.pose.pose.position.x
            
            msg = Twist()
            self.cmd_vel_pub.publish(msg) # stop
            rospy.sleep(1)

            """self.mechanismCommandSvr('approach', 1)
            rospy.sleep(3)
            self.mechanismCommandSvr('approach', 0)

            msg = Twist()
            self.cmd_vel_pub.publish(msg) # stop
            rospy.sleep(2)"""

            self.y_tile = 1
            self.state = X_MOVE_TO_PICK_POS2

        elif self.state == X_MOVE_TO_PICK_POS2:
            rospy.loginfo("Moving to x pick pos 2")
            self.color_target_to_align = "azul"
            current_odom_y = self.nav_odom.pose.pose.position.y
            target_odom_y = current_odom_y + (2 - self.x_tile) * 0.33
            sign = 1
            print(f"Current odom y: {current_odom_y}")
            print(f"Max odom y: {target_odom_y}")
            
            # While no cube is detected and the limit has not been reached, move back
            while abs(current_odom_y - target_odom_y) > 0.07:
                try:
                    sign = (target_odom_y-current_odom_y) / abs(target_odom_y-current_odom_y)
                except:
                    sign = 1
                msg = Twist()
                msg.linear.y = self.speed_nav * sign
                self.cmd_vel_pub.publish(msg)
                current_odom_y = self.nav_odom.pose.pose.position.y
            
            """
            while self.color_target_dis == 5000:
                print("Waiting for color target dis")
                pass

            while abs(self.color_target_dis) > self.tolerance:
                msg = Twist()
                msg.linear.y = - 0.13 * self.color_target_dis / abs(self.color_target_dis)
                self.cmd_vel_pub.publish(msg)"""

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
            self.state = RESET_ELEVATOR


        elif self.state == RESET_ELEVATOR:
            self.mechanismCommandSvr('hard_stop', 1)
            rospy.sleep(1)

            rospy.loginfo("Resetting elevator")
            rospy.loginfo(f"Color stack: {self.color_stack}")
            rospy.loginfo(f"Aruco stack: {self.aruco_stack}")
            rospy.loginfo(f"Letter stack: {self.letter_stack}")
            self.mechanismCommandSvr('elevator', 0)
            rospy.Rate(0.3).sleep()
            self.flag_selection.publish(1)
            self.flag_pub.publish(True)
            self.state = PICK_CUBE_TARGET

        elif self.state == PICK_CUBE_TARGET:
            rospy.sleep(0.5)
            rospy.loginfo("Searching for Target")
            self.search_tries+=1

            if self.target_success[0] or self.target_success[1] or self.target_success[2]:
                
                point_x_min_id = -1
                y_lowest = -70
                for i in range(3):
                    if self.detected_targets[i] == objectDetection():
                        continue
                    
                    c_y = (self.detected_targets[i].ymin + self.detected_targets[i].ymax) / 2

                    if c_y>= y_lowest:
                        y_lowest = c_y
                        point_x_min_id = i

                """if point_x_min_id != -1:
                    self.selected_target = self.detected_targets[point_x_min_id]
                    rospy.loginfo("Target Selected")
                    print(f"Going to {self.selected_target.labelText}")
                    self.search_tries = 0
                    self.state = DRIVE_TO_TARGET
                    return"""
                if point_x_min_id != -1:
                    self.selected_target = self.detected_targets[point_x_min_id]
                    print(f"Detected {self.selected_target.labelText}")

                    start_time_d = self.current_time
                    detections_total = 0
                    detections_success = 0

                    while( rospy.get_time() - start_time_d ) < 1.5:
                        detections_total += 1

                        point_x_min_id = -1
                        y_lowest = -70
                        for i in range(3):
                            if self.detected_targets[i] == objectDetection():
                                continue
                        
                            c_y = (self.detected_targets[i].ymin + self.detected_targets[i].ymax) / 2
                            if c_y>= y_lowest:
                                y_lowest = c_y
                                point_x_min_id = i

                        if point_x_min_id != -1:
                            if self.selected_target.labelText == self.detected_targets[point_x_min_id].labelText:
                                detections_success += 1

                    if detections_success/detections_total > 0.4:
                        self.state = DRIVE_TO_TARGET
                        rospy.loginfo("Target Selected")

                    self.search_tries = 0
                    return
                else:
                    rospy.logwarn("No target found")

            elif self.search_tries>3 and not self.left_side_pick:
                # spin to back
                # align with tile 3
                # spin front
                # left pick pos true
                rospy.loginfo("No cube found, moving left")
                
                self.current_angle += 180.0
                if self.current_angle > 270:
                    self.current_angle -= 360                
                msg_f = Float32()
                msg_f.data = float(self.current_angle)

                self.rotate_pub.publish( msg_f )
                self.flag_selection.publish(0)
                rospy.sleep(2)

                try:
                    client = rospy.ServiceProxy('/detect_color_pattern', DetectColorPattern)
                    resp = client()
                    if resp.tileX != 0 and resp.tileY != 0:
                        self.x_tile = resp.tileX
                        self.y_tile = resp.tileY
                        rospy.loginfo(f"Tile X: {self.x_tile}")
                        rospy.loginfo(f"Tile Y: {self.y_tile}")
                except rospy.ServiceException as e:
                    print("Pattern call failed: %s"%e)

                self.color_target_to_align = "amarillo"
                current_odom_y = self.nav_odom.pose.pose.position.y
                target_odom_y = current_odom_y + (5 - self.x_tile) * 0.27
                sign = 1
            
                # While no cube is detected and the limit has not been reached, move back
                while abs(current_odom_y - target_odom_y) > 0.07:
                    try:
                        sign = (target_odom_y-current_odom_y) / abs(target_odom_y-current_odom_y)
                    except:
                        sign = 1
                    msg = Twist()
                    msg.linear.y = self.speed_nav * sign
                    self.cmd_vel_pub.publish(msg)
                    current_odom_y = self.nav_odom.pose.pose.position.y
            
                while self.color_target_dis == 5000:
                    print("Waiting for color target dis")
                    pass

                while abs(self.color_target_dis) > self.tolerance:
                    msg = Twist()
                    msg.linear.y = - self.speed_align * self.color_target_dis / abs(self.color_target_dis)
                    self.cmd_vel_pub.publish(msg)
                #print(f"Color target dis: {self.color_target_dis}")

                msg = Twist()
                self.cmd_vel_pub.publish(msg) # stop
                self.color_target_dis = 5000
                rospy.sleep(1)
                self.x_tile = 3
                
                self.current_angle += 180.0
                if self.current_angle > 270:
                    self.current_angle -= 360                
                msg_f = Float32()
                msg_f.data = float(self.current_angle)

                self.rotate_pub.publish( msg_f )
                self.flag_selection.publish(1)
                rospy.sleep(2)
                
                self.search_tries = 0
                self.left_side_pick = True
                
                """prev move left code--
                current_odom_y = self.nav_odom.pose.pose.position.y
                min_odom_y = current_odom_y - MOVE_LEFT_TO_DETECT_LIMIT
                print(f"Current odom y: {current_odom_y}")
                print(f"Max odom y: {min_odom_y}")
                
                self.target_success = [False, False, False]
                self.detected_targets = [objectDetection(), objectDetection(), objectDetection()]
                # While no cube is detected and the limit has not been reached, move back
                while not self.target_success[0] and not self.target_success[1] and not self.target_success[2] and current_odom_y > min_odom_y:
                    msg = Twist()
                    msg.linear.y = 0.2
                    self.cmd_vel_pub.publish(msg)
                    current_odom_y = self.nav_odom.pose.pose.position.y
                    print(f"Current odom y: {current_odom_y}")
                    print(f"Max odom y: {min_odom_y}")
                if self.target_success[0] or self.target_success[1] or self.target_success[2]:
                    rospy.logdebug("Cube detected")
                    self.state = RESET_ELEVATOR
                else:
                    rospy.logwarn("No cube detected, limit reached")
                    self.state = PICK_FINISH
                    self.pick_result = PARTIAL_FINISH
                msg = Twist()
                self.cmd_vel_pub.publish(msg) # stop
                self.target_success = [False, False, False]
                self.detected_targets = [objectDetection(), objectDetection(), objectDetection()]
                rospy.sleep(1)"""
            else:
                self.state = PICK_FINISH
                self.pick_result = PARTIAL_FINISH

        elif self.state == DRIVE_TO_TARGET:
            #self.followCubePub.publish(True)
            rospy.loginfo("Target sent")

            rospy.Rate(1).sleep()
            
            #tfIntake = self.tfBuffer.lookup_transform('intake', 'qbo1', rospy.Time())
            
            self.mechanismCommandSvr('intake', 1)

            #self.driveTargetClient.wait_for_server()

            target_point = Point()
            #target_point.x = tfIntake.transform.translation.x
            #target_point.y = tfIntake.transform.translation.y
            target_point.x = self.selected_target.point3D.x
            target_point.y = self.selected_target.point3D.z - self.dis_to_intake
            target_point.z = 0

            self.pick_drive_target_timeout = 0
            self.pick_drive_target_timeout_flag = False

            self.state = X_TARGET
            self.last_error_x = 0
            self.pick_cube_init_odom = self.nav_odom.pose.pose.position.x


        elif self.state == X_TARGET:
            self.driveNavTarget(0.0, self.selected_target.point3D.x, 0.2)

            self.state = Y_TARGET
            rospy.loginfo("X target reached")

        elif self.state == Y_TARGET:

            msg = Twist()
            """if self.intakePresenceData:
                self.cmd_vel_pub.publish(msg)
                rospy.sleep(0.5)
                self.state = PICK_CUBE
                return"""
            msg.linear.x = max( (self.selected_target.point3D.z) * self.ykP, 0.2)
            p_offset = - self.xkP * self.selected_target.point3D.x
            d_offset = - self.xkD * (self.selected_target.point3D.x - self.last_error_x)
            #print(f"P: {p_offset}, D: {d_offset}")
            if abs(self.selected_target.point3D.x) > self.tolerance and self.selected_target_seen:
                msg.linear.y = - 0.17 * self.selected_target.point3D.x / abs(self.selected_target.point3D.x) + p_offset + d_offset
                self.pick_drive_target_timeout_flag = False


            """if not self.selected_target_seen and not self.pick_drive_target_timeout_flag:
                self.pick_drive_target_timeout = self.current_time
                self.pick_drive_target_timeout_flag = True

            if self.pick_drive_target_timeout_flag and (self.current_time - self.pick_drive_target_timeout) > 3.0:
                rospy.sleep(0.5)
                self.mechanismCommandSvr('intake', 5)
                self.state = PICK_CUBE_TARGET
                self.selected_target = objectDetection()
                self.target_success = [False, False, False]
                self.detected_targets = [objectDetection(), objectDetection(), objectDetection()]"""

            self.last_error_x = self.selected_target.point3D.x
            self.cmd_vel_pub.publish(msg)

        elif self.state == PICK_CUBE:
            msg = Twist()
            msg.linear.x = 0.125
            self.cmd_vel_pub.publish(msg)
            rospy.sleep(0.5)
            msg = Twist()
            self.cmd_vel_pub.publish(msg)
            self.mechanismCommandSvr('hard_stop', 1)
            rospy.sleep(2)
            rospy.loginfo("Target reached")
            self.flag_pub.publish(False)
            rospy.loginfo("Cube picked")
            self.state = ACTIVATE_ELEVATOR_TO_STORE

        elif self.state == ACTIVATE_ELEVATOR_TO_STORE:
            msg = Twist()
            self.cmd_vel_pub.publish(msg)
            print(f"Category is {self.selected_target.category}")
            if self.selected_target.category == str('color'):
                self.mechanismCommandSvr('elevator', 1)
            elif self.selected_target.category == str('aruco'):
                self.mechanismCommandSvr('elevator', 2)
            elif self.selected_target.category == str('letter'):
                self.mechanismCommandSvr('elevator', 3)

            rospy.sleep(3)
            rospy.loginfo("Elevator activated")
            self.state = STORE_CUBE

        elif self.state == STORE_CUBE:
            self.mechanismCommandSvr('intake', 2)
            rospy.sleep(2)
            

            # feedback if it has been stored (in progress)
            if not self.intakePresenceData:
                rospy.loginfo("Cube stored")
                if self.selected_target.category == str('color'):
                    self.color_stack.append(self.selected_target.labelText)
                elif self.selected_target.category == str('aruco'):
                    self.aruco_stack.append(self.selected_target.labelText)
                elif self.selected_target.category == str('letter'):
                    self.letter_stack.append(self.selected_target.labelText)
            else:
                rospy.loginfo("Cube not stored")
                self.mechanismCommandSvr('intake', 4) # drop cube
                rospy.Rate(1).sleep()
            
            # moving elevator a small step down to avoid cube blocking the intake
            self.mechanismCommandSvr('elevator', ELEVATOR_SMALLSTEPDOWN_COMMAND)
            rospy.sleep(2)
        
            rospy.loginfo(f"Number of cubes stored: {len(self.color_stack) + len(self.aruco_stack) + len(self.letter_stack)}")
            self.selected_target = objectDetection()
            if (len(self.color_stack) + len(self.aruco_stack) + len(self.letter_stack) >= NUMBER_OF_CUBES):
                rospy.loginfo("All cubes stored")
                self.state = PICK_FINISH
                self.pick_result = FINISH
                self.flag_pub.publish(True)
                self.target_success = [False, False, False]
                self.detected_targets = [objectDetection(), objectDetection(), objectDetection()]
            else:
                #self.target_nav_back = self.nav_odom.pose.pose.position.x - 0.3
                self.flag_pub.publish(True)
                self.target_success = [False, False, False]
                self.detected_targets = [objectDetection(), objectDetection(), objectDetection()]
                self.state = MOVE_BACK_TO_DETECT

        elif self.state == MOVE_BACK_TO_DETECT:
            current_odom_x = self.nav_odom.pose.pose.position.x
            min_odom_x = current_odom_x - MOVE_BACK_TO_DETECT_LIMIT
            #target_odom_x = cur.to_sec()rent_odom_x - MOVE_BACK_TO_DETECT_MINIMUM
            #target_odom_x = self.pick_cube_init_odom + 0.2
            target_odom_x = current_odom_x - 0.6
            print(f"Current odom x: {current_odom_x}")
            print(f"Max odom x: {min_odom_x}")
            
            while current_odom_x > target_odom_x:
                msg = Twist()
                msg.linear.x = -0.2
                self.cmd_vel_pub.publish(msg)
                current_odom_x = self.nav_odom.pose.pose.position.x
            
            # While no cube is detected and the limit has not been reached, move back
            """while not self.target_success[0] and not self.target_success[1] and not self.target_success[2] and current_odom_x > min_odom_x:
                msg = Twist()
                msg.linear.x = -0.2
                self.cmd_vel_pub.publish(msg)
                current_odom_x = self.nav_odom.pose.pose.position.x
                #print(f"Current odom x: {current_odom_x}")
                #print(f"Max odom x: {min_odom_x}")
            if self.target_success[0] or self.target_success[1] or self.target_success[2]:
                rospy.logdebug("Cube detected")
            else:
                rospy.logwarn("No cube detected, limit reached")"""
            msg = Twist()
            self.cmd_vel_pub.publish(msg) # stop
            self.target_success = [False, False, False]
            self.detected_targets = [objectDetection(), objectDetection(), objectDetection()]
            rospy.sleep(1)
            self.state = RESET_ELEVATOR

        
        elif self.state == PICK_FINISH:
            if self.pick_result == PARTIAL_PICK_FINISH:
                rospy.loginfo("Partial pick finish")
            else:
                rospy.loginfo("Pick finish")
            rospy.loginfo(f"Color stack: {self.color_stack}")
            rospy.loginfo(f"Aruco stack: {self.aruco_stack}")
            rospy.loginfo(f"Letter stack: {self.letter_stack}")

            # send message to rotate 180 degrees to face unload areas
            self.current_angle += 180
            if self.current_angle > 270:
                self.current_angle -= 360

            rotate_msg = Float32()
            rotate_msg.data = float(self.current_angle)
            self.rotate_pub.publish(rotate_msg)
            self.flag_selection.publish(0)
            rospy.sleep(3)
            
            """self.mechanismCommandSvr('approach', 1)
            rospy.sleep(3)
            self.mechanismCommandSvr('approach', 0)
            msg = Twist()
            self.cmd_vel_pub.publish(msg) # stop
            rospy.sleep(3)"""

            self.state = FIND_UPLOAD_TILE 

        elif self.state == FIND_UPLOAD_TILE:
            print("pattern")
            self.flag_pub.publish(True)
            self.mechanismCommandSvr('elevator', 1)
            rospy.sleep(4)
            rospy.wait_for_service('/detect_color_pattern')
            try:
                client = rospy.ServiceProxy('/detect_color_pattern', DetectColorPattern)
                resp = client()
                if resp.tileX != 0 and resp.tileY != 0:
                    self.x_tile = resp.tileX
                    self.y_tile = resp.tileY
                    rospy.loginfo(f"Tile X: {self.x_tile}")
                    rospy.loginfo(f"Tile Y: {self.y_tile}")
                    self.state = PICK_UNLOAD_TARGET
            except rospy.ServiceException as e:
                print("Pattern call failed: %s"%e)

        elif self.state == PICK_UNLOAD_TARGET:
            # Obtain selected target from the color stack
            if len(self.color_stack) > 0:
                self.color_target_to_align = self.color_stack.pop()
                rospy.loginfo(f"Cube to unload: {self.color_target_to_align}")
            else:
                self.state = FIND_TILE_SHELF
                rospy.loginfo("No more color cubes to unload")
                return
            # Wait for detections
            rospy.sleep(1)
            self.state = MOVE_X_UPLOAD_TILE 
            
        elif self.state == MOVE_X_UPLOAD_TILE:
            color2tile = {'rojo': 4, 'verde': 1, 'azul': 2, 'amarillo': 3}

            rospy.loginfo("Moving to x pick pos 1")
            self.driveNavTarget(0.0, -(color2tile[self.color_target_to_align] - self.x_tile) * 0.25, 0.2)
            
            print("Waiting for color target dis")
            while self.color_target_dis == 5000:
                pass

            self.driveNavTarget(0.0, self.color_target_dis, 0.2)
            
            self.color_target_dis = 5000
            self.x_tile = color2tile[self.color_target_to_align]
            
            self.state = MOVE_Y_UPLOAD_TILE

        elif self.state == MOVE_Y_UPLOAD_TILE:
            self.color_target_dis = 5000
            rospy.loginfo("Moving to y pick pos")
            
            print("Waiting for color target dis")
            while self.color_target_dis == 5000:
                pass

            print(self.color_target_dis)
            self.driveNavTarget(self.color_target_dis-0.63, 0.0, 0.2)

            self.color_target_dis = 5000
            
            self.y_tile = 1
            self.state = UNLOAD_CUBE 
        
        elif self.state == UNLOAD_CUBE:
            print("Starting approach")
            self.mechanismCommandSvr('approach', 1)
            rospy.sleep(3)
            self.mechanismCommandSvr('approach', 0)

            msg = Twist()
            self.cmd_vel_pub.publish(msg) # stop
            self.mechanismCommandSvr('hard_stop', 1)
            rospy.sleep(1)

            self.mechanismCommandSvr('intake', 3)
            rospy.sleep(0.5)
            self.mechanismCommandSvr('warehouse', 1)
            rospy.sleep(1)
            self.mechanismCommandSvr('intake', 4)
            rospy.sleep(1)
            
            self.driveNavTarget(-0.2, 0.0, 0.2)

            self.y_tile = 2
            self.state = PICK_UNLOAD_TARGET
            

        elif self.state == FIND_TILE_SHELF:
            self.flag_pub.publish(True)
            rospy.sleep(2)
            rospy.wait_for_service('/detect_color_pattern')
            try:
                client = rospy.ServiceProxy('/detect_color_pattern', DetectColorPattern)
                resp = client()
                if resp.tileX != 0 and resp.tileY != 0:
                    #self.x_tile = resp.tileX
                    #self.y_tile = resp.tileY
                    rospy.loginfo(f"Tile X: {self.x_tile}")
                    rospy.loginfo(f"Tile Y: {self.y_tile}")
                self.state = MOVE_X_SHELF_INIT
            except rospy.ServiceException as e:
                print("Pattern call failed: %s"%e)
        
        elif self.state == MOVE_X_SHELF_INIT:
            rospy.loginfo("Moving to x pick pos 1")
            self.color_target_to_align = "azul"
            self.driveNavTarget(0.0, -(2-self.x_tile) * 0.26, 0.2)
            
            while self.color_target_dis == 5000:
                print("Waiting for color target dis")
                pass

            self.driveNavTarget(0.0, self.color_target_dis, 0.2)

            self.color_target_dis = 5000
            
            self.x_tile = 2
            self.current_shelf = 2
            self.state = MOVE_Y_SHELF_INIT

        elif self.state == MOVE_Y_SHELF_INIT:
            rospy.loginfo("Moving to y shelf pos")
            self.color_target_to_align = "azul"
            
            print("Waiting for color target dis")
            while self.color_target_dis == 5000:
                pass

            print(self.color_target_dis)
            self.driveNavTarget(self.color_target_dis-1.57, 0.0, 0.2)

            self.color_target_dis = 5000
            rospy.sleep(1)

            self.y_tile = 5
            self.state = GET_SHELF_UNLOAD_TARGET

        elif self.state == GET_SHELF_UNLOAD_TARGET:
            # Obtain selected target from the color stack
            target_cube = ''
            if len(self.letter_stack) > 0:
                self.category_to_unload = self.categoryDict['letter']
                target_cube = self.letter_stack.pop()
                self.shelf_target = (ord(target_cube) - ord('A') ) % 3 + 1
                self.shelf_target_height = (ord(target_cube) - ord('A') ) // 3 + 1
            elif len(self.aruco_stack) > 0:
                self.category_to_unload = self.categoryDict['aruco']
                target_cube = self.aruco_stack.pop()
                self.shelf_target = (ord(target_cube) - ord('1') ) % 3 + 1
                self.shelf_target_height = (ord(target_cube) - ord('1') ) // 3 + 1
            else:
                self.state = FINISH
                return
            
            # Wait for detections
            rospy.loginfo(f"Cube to unload: {target_cube}")
            rospy.loginfo(f"Shelf target: {self.shelf_target}")
            rospy.loginfo(f"Shelf target height: {self.shelf_target_height}")

            self.mechanismCommandSvr('elevator', 0)
            rospy.sleep(3)
            self.state = MOVE_X_SHELF_UNLOAD

        elif self.state == MOVE_X_SHELF_UNLOAD:
            rospy.loginfo("Moving to x shelf")
            self.shelf_color_dict = {1: 'verde', 2: 'azul', 3: 'amarillo', 5: 'amarillo', 6: 'azul', 7: 'verde'}
            self.color_target_to_align = self.shelf_color_dict[ self.shelf_target ]

            self.driveNavTarget(0.0, -(self.shelf_target - self.current_shelf) * 0.25, 0.2)
            # TODO offset para aruco
            
            while self.color_target_dis == 5000:
                print("Waiting for color target dis")
                pass

            self.driveNavTarget(0.0, self.color_target_dis, 0.2)

            self.color_target_dis = 5000
            rospy.sleep(1)
            
            self.current_angle += 180
            if self.current_angle > 270:
                self.current_angle -= 360

            rotate_msg = Float32()
            rotate_msg.data = float(self.current_angle)
            self.rotate_pub.publish(rotate_msg)
            rospy.sleep(3)

            self.mechanismCommandSvr('hard_stop', 1)
            msg = Twist()
            self.cmd_vel_pub.publish(msg) # stop
            rospy.sleep(0.5)

            self.current_shelf = self.shelf_target
            self.mechanismCommandSvr('elevator', self.category_to_unload+4)
            rospy.sleep(2)
            self.mechanismCommandSvr('intake', 3)
            self.mechanismCommandSvr('warehouse', self.category_to_unload+1)
            rospy.sleep(3)
            self.mechanismCommandSvr('elevator', self.shelf_target_height+6)
            rospy.sleep(3)
            self.state = MOVE_Y_SHELF_UNLOAD

        elif self.state == MOVE_Y_SHELF_UNLOAD:
            rospy.loginfo("Moving to y shelf")

            self.mechanismCommandSvr('approach', 1)
            print("approach finished")
            rospy.sleep(3)
            self.mechanismCommandSvr('approach', 0)
            msg = Twist()
            self.cmd_vel_pub.publish(msg) # stop
            self.mechanismCommandSvr('hard_stop', 1)
            rospy.sleep(0.5)
            self.mechanismCommandSvr('intake', 4)
            rospy.sleep(1.5)
            rospy.logdebug("Cube unloaded")

            self.driveNavTarget(-0.18, 0.0, 0.2)
            
            self.current_angle += 180
            if self.current_angle > 270:
                self.current_angle -= 360

            rotate_msg = Float32()
            rotate_msg.data = float(self.current_angle)
            self.rotate_pub.publish(rotate_msg)
            rospy.sleep(3)

            self.mechanismCommandSvr('hard_stop', 1)
            msg = Twist()
            self.cmd_vel_pub.publish(msg) # stop
            rospy.sleep(0.5)
            
            self.state = GET_SHELF_UNLOAD_TARGET

        elif self.state == PARTIAL_FINISH:
            rospy.loginfo("Partial finish")
            while (1):
                pass
        
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
    rate = rospy.Rate(50) # 10hz

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
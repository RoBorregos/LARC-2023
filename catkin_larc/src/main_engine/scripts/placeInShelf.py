#!/usr/bin/env python3

import rospy
import tf_conversions
import tf2_ros
import actionlib
from vision.msg import objectDetection, objectDetectionArray
import geometry_msgs.msg
from std_msgs.msg import Int32, Bool, Float32
from geometry_msgs.msg import Point, Pose, Twist
from main_engine.srv import MechanismCommand, MechanismCommandResponse
from nav_main.msg import Drive2TargetAction, Drive2TargetGoal, Drive2TargetResult, Drive2TargetFeedback
from nav_msgs.msg import Odometry

FIND_INIT_POS = "find_init_pos"
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
DRIVE_TO_COLOR_AREA = "drive_to_color_area"
MOVE_BACK_TO_UNLOAD_COLOR = "move_back_to_unload_color"
PARTIAL_FINISH = "partial_finish"
FINISH = "finish"

X_TARGET = "x_target"
Y_TARGET = "y_target"

ELEVATOR_SMALLSTEPDOWN_COMMAND = 50

MOVE_BACK_TO_DETECT_LIMIT = 0.5 # Xcm maximum to go back from cube
MOVE_BACK_TO_DETECT_MINIMUM = 0.4 # Xcm minimum to go back from cube
MOVE_LEFT_TO_DETECT_LIMIT = 1.0 # 1m maximum to go left when not finding a cube

NUMBER_OF_CUBES = 2



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
        self.xkP = 0.15
        self.xkD = 2.0
        self.last_error_x = 0
        self.ykP = 0.35
        self.target_nav_back = 0.0

        self.search_tries = 0

        self.categoryDict = { 'color': 0, 'aruco': 1, 'letter': 2 }
        self.color_stack = []
        self.aruco_stack = ['5']
        self.letter_stack = []
        self.nav_odom = Odometry()


        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.Subscriber('/vision/color_detect', objectDetectionArray, self.colorDetectCb)
        rospy.Subscriber('/vision/aruco_detect', objectDetectionArray, self.arucoDetectCb)
        rospy.Subscriber('/vision/letter_detect', objectDetectionArray, self.lettersDetectCb)
        rospy.Subscriber('/intake_presence', Bool, self.intakePresenceCb)
        self.intakePresenceData = False
        rospy.Subscriber('/odom', Odometry, self.odomCb)
        self.flag_pub = rospy.Publisher('/flag', Bool, queue_size=10)
        self.rotate_pub = rospy.Publisher('/rotate', Float32, queue_size=10)

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
            rospy.Rate(0.5).sleep()
            self.state = PICK_CUBE
            #self.flag_pub.publish(False)

    def cubeTargetsHandler(self, data):
        if self.state == PICK_CUBE_TARGET or self.state == MOVE_BACK_TO_DETECT:
            sz = len(data.detections)
            if sz == 0:
                return
            category_id = self.categoryDict[ data.detections[0].category ]
            point_x_min_id = 0
            y_lowest = data.detections[0].ymax
            y_lowest_id = 0
            for i in range(sz):
                if data.detections[i].ymax - y_lowest >= 100:
                    y_lowest = data.detections[i].ymax
                    point_x_min_id = i
                elif abs(data.detections[i].ymax - y_lowest) < 100:
                    if( data.detections[i].point3D.x > data.detections[point_x_min_id].point3D.x ):
                        point_x_min_id = i
            
            self.target_success[category_id] = True
            self.detected_targets[category_id] = data.detections[point_x_min_id]

        if self.state == X_TARGET or self.state == Y_TARGET:
            # check if target is still in sight, based on the bounding pixels, then obtain the target's 3D position
            sz = len(data.detections)
            if sz == 0:
                return
            for i in range(sz):
                if data.detections[i].labelText == self.selected_target.labelText:
                    self.selected_target = data.detections[i]
                    return
        
        if self.state == PICK_UNLOAD_TARGET:
             # check if target is still in sight, based on the bounding pixels, then obtain the target's 3D position
            sz = len(data.detections)
            if sz == 0:
                return
            for i in range(sz):
                if data.detections[i].labelText == self.selected_target.labelText:
                    self.selected_target = data.detections[i]
                    return

        

    def colorDetectCb(self, data):
        self.cubeTargetsHandler(data)
                
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


    def run(self):
        self.current_time = rospy.Time.now()

        if self.state == RESET_ELEVATOR:
            rospy.loginfo("Resetting elevator")
            rospy.loginfo(f"Color stack: {self.color_stack}")
            rospy.loginfo(f"Aruco stack: {self.aruco_stack}")
            rospy.loginfo(f"Letter stack: {self.letter_stack}")
            self.mechanismCommandSvr('elevator', 0)
            rospy.Rate(0.3).sleep()
            self.flag_pub.publish(True)
            self.state = PICK_CUBE_TARGET

        if self.state == PICK_CUBE_TARGET:
            rospy.Rate(1).sleep()
            rospy.loginfo("Searching for Target")
            self.search_tries+=1

            if self.target_success[0] or self.target_success[1] or self.target_success[2]:
                
                point_x_min_id = -1
                y_lowest = -70
                y_lowest_id = 0
                for i in range(3):
                    if self.detected_targets[i] == objectDetection():
                        continue
                    if self.detected_targets[i].ymax - y_lowest >= 100:
                        y_lowest = self.detected_targets[i].ymax
                        point_x_min_id = i
                    elif abs(self.detected_targets[i].ymax - y_lowest) < 100:
                        if( self.detected_targets[i].point3D.x > self.detected_targets[point_x_min_id].point3D.x ):
                            point_x_min_id = i

                if point_x_min_id != -1:
                    self.selected_target = self.detected_targets[point_x_min_id]
                    rospy.loginfo("Target Selected")
                    print(f"Going to {self.selected_target.labelText}")
                    self.search_tries = 0
                    self.state = DRIVE_TO_TARGET
                    return
                else:
                    rospy.logwarn("No target found")

            elif self.search_tries>3:
                rospy.loginfo("No cube found, moving left")
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
                rospy.sleep(1)

        elif self.state == DRIVE_TO_TARGET:
            #self.followCubePub.publish(True)
            rospy.loginfo("Target sent")

            rospy.Rate(0.5).sleep()
            
            #tfIntake = self.tfBuffer.lookup_transform('intake', 'qbo1', rospy.Time())
            
            self.mechanismCommandSvr('intake', 1)

            #self.driveTargetClient.wait_for_server()

            target_point = Point()
            #target_point.x = tfIntake.transform.translation.x
            #target_point.y = tfIntake.transform.translation.y
            target_point.x = self.selected_target.point3D.x
            target_point.y = self.selected_target.point3D.z - self.dis_to_intake
            target_point.z = 0

            """self.targetPointFbPub.publish( target_point )

            goal = Drive2TargetGoal( target=target_point )
            print(target_point)
            print(self.selected_target.labelText)

            self.driveTargetClient.send_goal(goal)
            self.driveTargetClient.wait_for_result()
            print(self.driveTargetClient.get_result())"""

            self.state = X_TARGET
            self.last_error_x = 0


        elif self.state == X_TARGET:
            msg = Twist()
            #print(self.selected_target.point3D.x)
            p_offset = - self.xkP * self.selected_target.point3D.x
            d_offset = - self.xkD * (self.selected_target.point3D.x - self.last_error_x)
            #print(f"P: {p_offset}, D: {d_offset}")
            if self.selected_target.point3D.x != 0:
                msg.linear.y = - 0.11 * self.selected_target.point3D.x / abs(self.selected_target.point3D.x) + p_offset + d_offset
            self.last_error_x = self.selected_target.point3D.x
            self.cmd_vel_pub.publish(msg)
            if abs(self.selected_target.point3D.x) < self.tolerance:
                self.state = Y_TARGET
                rospy.loginfo("X target reached")
                msg_z = Twist()
                self.cmd_vel_pub.publish(msg_z)
                rospy.Rate(0.4).sleep()

        elif self.state == Y_TARGET:
            msg = Twist()
            msg.linear.x = max( (self.selected_target.point3D.z) * self.ykP, 0.11)
            p_offset = - self.xkP * self.selected_target.point3D.x
            d_offset = - self.xkD * (self.selected_target.point3D.x - self.last_error_x)
            #print(f"P: {p_offset}, D: {d_offset}")
            if self.selected_target.point3D.x != 0:
                msg.linear.y = - 0.12 * self.selected_target.point3D.x / abs(self.selected_target.point3D.x) + p_offset + d_offset
            self.last_error_x = self.selected_target.point3D.x
            self.cmd_vel_pub.publish(msg)

        elif self.state == PICK_CUBE:
            msg = Twist()
            self.cmd_vel_pub.publish(msg)
            rospy.loginfo("Target reached")
            self.flag_pub.publish(False)
            rospy.sleep(4)
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
            if self.intakePresenceData:
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
                rospy.Rate(0.2).sleep()
            
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
            """self.followCubePub.publish(False)
            target_point = Point()
            target_point.x = 0
            # listen to topic odom and return to 0 position
            target_point.y = -self.nav_odom.pose.pose.position.x
            target_point.z = 0
            goal = Drive2TargetGoal( target=target_point )
            
            rospy.loginfo("Moving back to detect")
            self.driveTargetClient.send_goal(goal)
            self.driveTargetClient.wait_for_result()
            print(self.driveTargetClient.get_result())
"""         
            """
            rospy.loginfo("Moving back to detect")
            msg = Twist()
            msg.linear.x = -0.15
            self.cmd_vel_pub.publish(msg)
            rospy.sleep(4)

            #if self.nav_odom.pose.pose.position.x - self.target_nav_back < 0.05:
            msg = Twist()
            self.cmd_vel_pub.publish(msg)
            self.target_success = [False, False, False]
            self.detected_targets = [objectDetection(), objectDetection(), objectDetection()]
            self.state = RESET_ELEVATOR
            """
            current_odom_x = self.nav_odom.pose.pose.position.x
            min_odom_x = current_odom_x - MOVE_BACK_TO_DETECT_LIMIT
            target_odom_x = current_odom_x - MOVE_BACK_TO_DETECT_MINIMUM
            print(f"Current odom x: {current_odom_x}")
            print(f"Max odom x: {min_odom_x}")
            
            while current_odom_x > target_odom_x:
                msg = Twist()
                msg.linear.x = -0.2
                self.cmd_vel_pub.publish(msg)
                current_odom_x = self.nav_odom.pose.pose.position.x
            
            # While no cube is detected and the limit has not been reached, move back
            while not self.target_success[0] and not self.target_success[1] and not self.target_success[2] and current_odom_x > min_odom_x:
                msg = Twist()
                msg.linear.x = -0.2
                self.cmd_vel_pub.publish(msg)
                current_odom_x = self.nav_odom.pose.pose.position.x
                #print(f"Current odom x: {current_odom_x}")
                #print(f"Max odom x: {min_odom_x}")
            if self.target_success[0] or self.target_success[1] or self.target_success[2]:
                rospy.logdebug("Cube detected")
            else:
                rospy.logwarn("No cube detected, limit reached")
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
            rotate_msg = Float32()
            rotate_msg.data = 180
            self.rotate_pub.publish(rotate_msg)
            rospy.sleep(2)
            self.state = PICK_UNLOAD_TARGET
            
        elif self.state == PICK_UNLOAD_TARGET:
            # Obtain selected target from the color stack
            self.selected_target = objectDetection()
            if len(self.color_stack) > 0:
                self.selected_target.category = 'color'
                self.selected_target.labelText = self.color_stack.pop()
                rospy.loginfo(f"Cube to unload: {self.selected_target.labelText}")
            else:
                self.state = FINISH
                return
            # Wait for detections
            rospy.sleep(1)
            self.state = DRIVE_TO_COLOR_AREA

            
        elif self.state == DRIVE_TO_COLOR_AREA:
            # Obtain 3D point of area
            target_point = Point()
            #target_point.x = tfIntake.transform.translation.x
            #target_point.y = tfIntake.transform.translation.y
            target_point.x = self.selected_target.point3D.x
            target_point.y = self.selected_target.point3D.z - self.dis_to_intake
            target_point.z = 0
            
            current_odom_y = self.nav_odom.pose.pose.position.y
            target_odom_y = current_odom_y + target_point.y
            
            print(f"Current odom y: {current_odom_y}")
            print(f"Target odom y: {target_odom_y}")

            # if target is left
            while current_odom_y < target_odom_y:
                msg = Twist()
                msg.linear.y = 0.2
                self.cmd_vel_pub.publish(msg)
                current_odom_y = self.nav_odom.pose.pose.position.y
            while current_odom_y > target_odom_y:
                msg = Twist()
                msg.linear.y = -0.2
                self.cmd_vel_pub.publish(msg)
                current_odom_y = self.nav_odom.pose.pose.position.y
            
            msg = Twist()
            self.cmd_vel_pub.publish(msg) # stop
            rospy.loginfo("Reached Y target of Unloading area")
            rospy.sleep(1)

            current_odom_x = self.nav_odom.pose.pose.position.x
            target_odom_x = current_odom_x + target_point.x - 0.3 # 0.15 from half the square, and 0.15 distance from center of robot to intake

            while current_odom_x < target_odom_x:
                msg = Twist()
                msg.linear.x = -0.2
                self.cmd_vel_pub.publish(msg)
                current_odom_x = self.nav_odom.pose.pose.position.x
                print(f"Current odom x: {current_odom_x}")
                print(f"Target odom x: {target_odom_x}")
                print("------------------")
            while current_odom_x > target_odom_x:
                msg = Twist()
                msg.linear.x = 0.2
                self.cmd_vel_pub.publish(msg)
                current_odom_x = self.nav_odom.pose.pose.position.x
                print(f"Current odom x: {current_odom_x}")
                print(f"Target odom x: {target_odom_x}")
                print("------------------")
            msg = Twist()
            self.cmd_vel_pub.publish(msg) # stop
            rospy.loginfo("Reached X target of Unloading area")
            rospy.sleep(5)
            self.state = MOVE_BACK_TO_UNLOAD_COLOR
            
        if self.state == MOVE_BACK_TO_UNLOAD_COLOR:
            current_odom_x = self.nav_odom.pose.pose.position.x
            target_odom_x = current_odom_x -1.0

            while current_odom_x > target_odom_x:
                msg = Twist()
                msg.linear.x = -0.2
                self.cmd_vel_pub.publish(msg)
                current_odom_x = self.nav_odom.pose.pose.position.x
            msg = Twist()
            self.cmd_vel_pub.publish(msg) # stop
            rospy.loginfo("Picking unloading area again")
            rospy.sleep(1)
            self.state = PICK_UNLOAD_TARGET



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
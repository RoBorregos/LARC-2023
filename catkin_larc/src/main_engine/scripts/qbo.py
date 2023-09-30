#!/usr/bin/env python3

import rospy
import tf_conversions
import tf2_ros
import actionlib
from vision.msg import objectDetection, objectDetectionArray
import geometry_msgs.msg
from std_msgs.msg import Int32, Bool
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
FINISH = "finish"

X_TARGET = "x_target"
Y_TARGET = "y_target"



class MainEngine:
    def __init__(self):
        rospy.loginfo("MainEngine init")
        self.current_time = rospy.Time.now()
        self.state = RESET_ELEVATOR
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

        self.color_stack = []
        self.aruco_stack = []
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
            self.flag_pub.publish(False)

    def colorDetectCb(self, data):
        if self.state == PICK_CUBE_TARGET:
            #NOTE inverted x and y
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

            self.target_success[0] = True
            self.detected_targets[0] = data.detections[point_x_min_id]
        
        if self.state == X_TARGET or self.state == Y_TARGET:
            # check if target is still in sight, based on the bounding pixels, then obtain the target's 3D position
            sz = len(data.detections)
            if sz == 0:
                return
            for i in range(sz):
                if data.detections[i].labelText == self.selected_target.labelText: #and abs(data.detections[i].point3D.x - self.selected_target.point3D.x) < 0.15 and abs(data.detections[i].point3D.z - self.selected_target.point3D.z) < 0.15:
                    self.selected_target = data.detections[i]
                    return
                
    def arucoDetectCb(self, data):
        if self.state == PICK_CUBE_TARGET:
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

            self.target_success[1] = True
            self.detected_targets[1] = data.detections[point_x_min_id]

        if self.state == X_TARGET or self.state == Y_TARGET:
            # check if target is still in sight, based on the bounding pixels, then obtain the target's 3D position
            sz = len(data.detections)
            if sz == 0:
                return
            for i in range(sz):
                if data.detections[i].labelText == self.selected_target.labelText:
                    self.selected_target = data.detections[i]
                    return
                
    def lettersDetectCb(self, data):
        if self.state == PICK_CUBE_TARGET:
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

            self.target_success[2] = True
            self.detected_targets[2] = data.detections[point_x_min_id]

        if self.state == X_TARGET or self.state == Y_TARGET:
            # check if target is still in sight, based on the bounding pixels, then obtain the target's 3D position
            sz = len(data.detections)
            if sz == 0:
                return
            for i in range(sz):
                if data.detections[i].labelText == self.selected_target.labelText:
                    self.selected_target = data.detections[i]
                    return

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
            if self.target_success[0] or self.target_success[1] or self.target_success[2]:
                point_x_min_id = -1
                y_lowest = -70
                y_lowest_id = 0
                for i in range(3):
                    if self.detected_targets[i] == objectDetection():
                        continue
                    if self.detected_targets[i].ymin - y_lowest >= 60:
                        y_lowest = self.detected_targets[i].ymin
                        point_x_min_id = i
                    elif abs(self.detected_targets[i].ymin - y_lowest) < 60:
                        if( abs(self.detected_targets[i].point3D.x) < abs(self.detected_targets[point_x_min_id].point3D.x)):
                            point_x_min_id = i

                if point_x_min_id != -1:
                    self.selected_target = self.detected_targets[point_x_min_id]
                    rospy.logdebug("Target Selected")
                    rospy.loginfo(self.selected_target)
                    self.state = DRIVE_TO_TARGET
                else:
                    rospy.logwarn("No target found")
                

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
                msg.linear.y = - 0.11 * self.selected_target.point3D.x / abs(self.selected_target.point3D.x) + p_offset + d_offset
            self.last_error_x = self.selected_target.point3D.x
            self.cmd_vel_pub.publish(msg)

        elif self.state == PICK_CUBE:
            rospy.loginfo("Target reached")
            rospy.Rate(0.5).sleep()
            rospy.loginfo("Cube picked")
            self.state = ACTIVATE_ELEVATOR_TO_STORE

        elif self.state == ACTIVATE_ELEVATOR_TO_STORE:
            print(f"Category is {self.selected_target.category}")
            if self.selected_target.category == str('color'):
                self.mechanismCommandSvr('elevator', 1)
            elif self.selected_target.category == str('aruco'):
                self.mechanismCommandSvr('elevator', 2)
            elif self.selected_target.category == str('letter'):
                self.mechanismCommandSvr('elevator', 3)

            rospy.Rate(0.2).sleep()
            rospy.loginfo("Elevator activated")
            self.state = STORE_CUBE

        elif self.state == STORE_CUBE:
            self.mechanismCommandSvr('intake', 2)
            rospy.Rate(0.2).sleep()
            
            if not self.intakePresenceData:
                rospy.logdebug("Cube stored")
                if self.selected_target.category == str('color'):
                    self.color_stack.append(self.selected_target.labelText)
                elif self.selected_target.category == str('aruco'):
                    self.aruco_stack.append(self.selected_target.labelText)
                elif self.selected_target.category == str('letter'):
                    self.letter_stack.append(self.selected_target.labelText)
            else:
                rospy.logwarn("Cube not stored")
                self.mechanismCommandSvr('intake', 4) # drop cube
                rospy.Rate(0.2).sleep()
            # feedback if it has been stored (in progress)
            self.selected_target = objectDetection()
            self.target_nav_back = self.nav_odom.pose.pose.position.x - 0.3
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
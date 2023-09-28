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

FIND_INIT_POS = "find_init_pos"
ROTATE_TO_CUBES = "rotate_to_cubes"
PICK_CUBE_TARGET = "pick_cube_target"
DRIVE_TO_TARGET = "drive_to_target"
PICK_CUBE = "pick_cube"
ACTIVATE_ELEVATOR_TO_STORE = "activate_elevator_to_store"
STORE_CUBE = "store_cube"
MOVE_BACK_TO_DETECT = "move_back_to_detect"
FINISH = "finish"

class MainEngine:
    def __init__(self):
        rospy.loginfo("MainEngine init")
        self.current_time = rospy.Time.now()
        self.state = PICK_CUBE_TARGET
        self.target_success = False
        self.selected_target = objectDetection()
        self.dis_to_intake = 0.13

        self.br = tf2_ros.TransformBroadcaster()
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        rospy.Subscriber('/color_detect', objectDetectionArray, self.colorDetectCb)
        self.cmdVelPub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.targetPointFbPub = rospy.Publisher('/target_point_fb', Point, queue_size=10)
        self.driveTargetClient = actionlib.SimpleActionClient("drive_to_target", Drive2TargetAction)


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
                if data.detections[i].ymin - y_lowest >= 80:
                    y_lowest = data.detections[i].ymin
                    y_lowest_id = i
                    point_x_min_id = i
                elif abs(data.detections[i].ymin - y_lowest) < 80:
                    if( abs(data.detections[i].point3D.x) < abs(data.detections[point_x_min_id].point3D.x)):
                        point_x_min_id = i

            self.target_success = True
            self.selected_target = data.detections[point_x_min_id]
        
        if self.state == DRIVE_TO_TARGET:
            # check if target is still in sight, based on the bounding pixels, then obtain the target's 3D position
            sz = len(data.detections)
            if sz == 0:
                return
            for i in range(sz):
                if data.detections[i].labelText == self.selected_target.labelText and abs(data.detections[i].point3D.x - self.selected_target.point3D.x) < 0.15 and abs(data.detections[i].point3D.z - self.selected_target.point3D.z) < 0.15:
                    self.selected_target = data.detections[i]
                    target_point_fb = Point()
                    target_point_fb.x = self.selected_target.point3D.x
                    target_point_fb.y = self.selected_target.point3D.z - self.dis_to_intake
                    target_point_fb.z = 0
                    self.targetPointFbPub.publish( target_point_fb )
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

        if self.state == PICK_CUBE_TARGET:
            self.mechanismCommandSvr('elevator', 0)
            if self.target_success:
                self.state = DRIVE_TO_TARGET
                rospy.loginfo("Target selected")

        elif self.state == DRIVE_TO_TARGET:
            t = geometry_msgs.msg.TransformStamped()

            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "cam"
            t.child_frame_id = "qbo1"
            t.transform.translation.x = self.selected_target.point3D.x
            t.transform.translation.y = self.selected_target.point3D.y
            t.transform.translation.z = self.selected_target.point3D.z
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            #self.br.sendTransform(t)
            rospy.loginfo("Target sent")

            rospy.Rate(0.5).sleep()
            
            #tfIntake = self.tfBuffer.lookup_transform('intake', 'qbo1', rospy.Time())
            
            self.mechanismCommandSvr('intake', 1)

            self.driveTargetClient.wait_for_server()

            target_point = Point()
            #target_point.x = tfIntake.transform.translation.x
            #target_point.y = tfIntake.transform.translation.y
            target_point.x = t.transform.translation.x
            target_point.y = t.transform.translation.z - self.dis_to_intake
            target_point.z = 0
            goal = Drive2TargetGoal( target=target_point )
            print(target_point)
            print(self.selected_target.labelText)

            self.driveTargetClient.send_goal(goal)
            self.driveTargetClient.wait_for_result()
            print(self.driveTargetClient.get_result())

            if self.driveTargetClient.get_result():
                self.state = PICK_CUBE

            rospy.loginfo("Target reached")

        elif self.state == PICK_CUBE:
            rospy.Rate(0.5).sleep()
            rospy.loginfo("Cube picked")
            self.state = ACTIVATE_ELEVATOR_TO_STORE

        elif self.state == ACTIVATE_ELEVATOR_TO_STORE:
            self.mechanismCommandSvr('elevator', 1)
            rospy.Rate(0.2).sleep()
            rospy.loginfo("Elevator activated")
            self.state = STORE_CUBE

        elif self.state == STORE_CUBE:
            self.mechanismCommandSvr('intake', 2)
            rospy.Rate(0.2).sleep()
            rospy.loginfo("Cube stored")
            self.state = MOVE_BACK_TO_DETECT

        elif self.state == MOVE_BACK_TO_DETECT:
            target_point = Point()
            target_point.x = 0
            target_point.y = -0.3
            target_point.z = 0
            goal = Drive2TargetGoal( target=target_point )
            
            rospy.loginfo("Moving back to detect")
            self.driveTargetClient.send_goal(goal)
            self.driveTargetClient.wait_for_result()
            print(self.driveTargetClient.get_result())

            self.target_success = False
            
            self.state = PICK_CUBE_TARGET

        
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
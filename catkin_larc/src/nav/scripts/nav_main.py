#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from nav.msg import Drive2TargetAction, Drive2TargetGoal, Drive2TargetResult, Drive2TargetFeedback

X_TARGET = "X_TARGET"
Y_TARGET = "Y_TARGET"
FOLLOW_SUCCESS = "FOLLOW_SUCCESS"

class NavMain:
    def __init__(self):
        self.drive_target_as = actionlib.SimpleActionServer("drive_to_target", Drive2TargetAction, execute_cb=self.ex_cb_drive_target, auto_start=False)
        self.drive_target_as.start()
        self.drive_target_feedback = Drive2TargetFeedback()
        self.drive_target_result = Drive2TargetResult()
        self.distance_acc_tolerance = 0.01
        self.x_feedback_tolerance = 0.04
        self.x_feedback_kP = 0.15
        self.x_feedback_kD = 2.0
        self.x_feedback_last_error = 0
        self.y_feedback_kP = 1.0
        
        self.min_speed = 0.125
        self.max_speed = 0.6
        self.SPEED = 0.175
        self.pos_kP = 2.0

        self.pos_kD = 1.0
        self.rate = rospy.Rate(10)
        self.nav_target_x = 0
        self.nav_target_y = 0
        self.init_nav_target_x = 0
        self.init_nav_target_y = 0
        self.intake_presence = False

        self.follow_state = X_TARGET
        self.following_cube = False

        self.pubCmdVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.subOdom = rospy.Subscriber('/odom', Odometry, self.odomCb)
        rospy.Subscriber('/follow_cube', Bool, self.followCubeCb)
        rospy.Subscriber('/target_point_fb', Point, self.targetPointFbCb)
        rospy.Subscriber('/intake_presence', Bool, self.intakePresenceCb)
        self.nav_odom = Odometry()

        self.run()

    def ex_cb_drive_target(self, goal):
        self.intake_presence = False
       
        #self.nav_target_x = - goal.target.x + self.nav_odom.pose.pose.position.y
        #if goal.target.y == -0.3:
        #    self.nav_target_y = 0.15
        distance_x = abs(goal.target.x)
        distance_y = abs(goal.target.y)
        last_error_x = 0
        last_error_y = 0
        
        self.init_nav_target_x = - goal.target.x
        self.init_nav_target_y = goal.target.y
        self.drive_target_feedback.distance_to_target = Point()
        self.drive_target_feedback.distance_to_target.x = distance_x
        self.drive_target_feedback.distance_to_target.y = distance_y

        success = True

        rospy.logwarn('Starting drive to target')

        while distance_x > self.distance_acc_tolerance:
            if self.drive_target_as.is_preempt_requested() or self.intake_presence:
                rospy.loginfo('Preempted')
                self.drive_target_as.set_preempted()
                msg = Twist()
                self.pubCmdVel.publish( msg )
                success = False
                break
            # nav_target_x should be 0.5, as the cube has to be in the middle of the robot
            error_x = self.init_nav_target_x - self.nav_odom.pose.pose.position.y
            #error_x = self.nav_target_x #- self.nav_odom.pose.pose.position.y
            msg = Twist()
            #msg.linear.y = error_x * self.pos_kP + (error_x - last_error_x) * self.pos_kD
            # get the sign of error and multiply with 0.4
            msg.linear.y = - self.SPEED * error_x / abs(error_x)

            self.pubCmdVel.publish( msg )
            # update init_nav_target_x
            
            distance_x = abs(error_x)
            self.drive_target_feedback.distance_to_target.x = distance_x
            self.drive_target_as.publish_feedback( self.drive_target_feedback )
            self.rate.sleep()

        msg = Twist()
        self.pubCmdVel.publish( msg )
        rospy.loginfo('Reached X target')
        rospy.sleep(1.0)

        while distance_y > self.distance_acc_tolerance:
            if self.drive_target_as.is_preempt_requested() or not success or self.intake_presence:
                rospy.loginfo('Preempted')
                self.drive_target_as.set_preempted()
                msg = Twist()
                self.pubCmdVel.publish( msg )
                success = False
                break
            error_y = self.init_nav_target_y - self.nav_odom.pose.pose.position.x
            #error_x = self.nav_target_x #- self.nav_odom.pose.pose.position.y
            msg = Twist()
            #msg.linear.x = error_y * self.pos_kP + (error_y - last_error_y) * self.pos_kD
            # get the sign of error and multiply with speed
            msg.linear.x = self.SPEED * error_y / abs(error_y)

            self.pubCmdVel.publish( msg )

            distance_y = abs(error_y)
            self.drive_target_feedback.distance_to_target.y = distance_y
            self.drive_target_as.publish_feedback( self.drive_target_feedback )
            self.rate.sleep()

        rospy.loginfo('Reached Y target')
        msg = Twist()
        self.pubCmdVel.publish( msg )

        if success:
            self.drive_target_result.success = True
            self.drive_target_as.set_succeeded( self.drive_target_result )
            rospy.loginfo('Reached target')

    def odomCb(self, data):
        self.nav_odom = data
    
    def intakePresenceCb(self, data):
        self.intake_presence = data.data
        if data.data == True:
            print("intake presence")
            msg = Twist()
            msg.linear.x = 0
            msg.linear.y = 0
            msg.linear.z = 0
            msg.angular.x = 0
            msg.angular.y = 0
            msg.angular.z = 0
            self.pubCmdVel.publish(msg)
            rospy.Rate(0.5).sleep()
            self.state = FOLLOW_SUCCESS

    def followCubeCb(self, data):
        self.following_cube = data.data


    def targetPointFbCb(self, data):
        self.nav_target_x = data.x #+ self.nav_odom.pose.pose.position.y
        self.nav_target_y = data.y


    def run(self):
        pass
            


if __name__ == '__main__':
    rospy.init_node('main_nav')
    nav_main = NavMain()

    while not rospy.is_shutdown():
        nav_main.run()
        rospy.Rate(10).sleep()
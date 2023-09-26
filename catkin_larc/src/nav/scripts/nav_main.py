#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from nav.msg import Drive2TargetAction, Drive2TargetGoal, Drive2TargetResult, Drive2TargetFeedback

class NavMain:
    def __init__(self):
        
        self.drive_target_as = actionlib.SimpleActionServer("drive_to_target", Drive2TargetAction, execute_cb=self.ex_cb_drive_target, auto_start=False)
        self.drive_target_as.start()
        self.drive_target_feedback = Drive2TargetFeedback()
        self.drive_target_result = Drive2TargetResult()
        self.distance_acc_tolerance = 0.01
        self.pos_kP = 2.0
        self.rate = rospy.Rate(10)

        self.pubCmdVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.subOdom = rospy.Subscriber('/odom', Odometry, self.odomCb)
        self.nav_odom = Odometry()

    def ex_cb_drive_target(self, goal):
        nav_target_x = goal.target.x + self.nav_odom.pose.pose.position.x
        nav_target_y = goal.target.y + self.nav_odom.pose.pose.position.y
        distance_x = abs(goal.target.x)
        distance_y = abs(goal.target.y)
        
        self.drive_target_feedback.distance_to_target = Point()
        self.drive_target_feedback.distance_to_target.x = distance_x
        self.drive_target_feedback.distance_to_target.y = distance_y

        success = True

        rospy.loginfo('Starting drive to target')

        while distance_x > self.distance_acc_tolerance:
            if self.drive_target_as.is_preempt_requested():
                rospy.loginfo('Preempted')
                self.drive_target_as.set_preempted()
                success = False
                break
            error_x = nav_target_x - self.nav_odom.pose.pose.position.x
            msg = Twist()
            msg.linear.y = error_x * self.pos_kP
            self.pubCmdVel.publish( msg )

            distance_x = abs(error_x)
            self.drive_target_feedback.distance_to_target.x = distance_x
            self.drive_target_as.publish_feedback( self.drive_target_feedback )
            self.rate.sleep()

        msg = Twist()
        self.pubCmdVel.publish( msg )
        rospy.loginfo('Reached X target')

        while distance_y > self.distance_acc_tolerance:
            if self.drive_target_as.is_preempt_requested() or not success:
                rospy.loginfo('Preempted')
                self.drive_target_as.set_preempted()
                success = False
                break
            error_y = nav_target_y - self.nav_odom.pose.pose.position.y
            msg = Twist()
            msg.linear.x = error_y * self.pos_kP
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


    def odomCb(self, data):
        self.nav_odom = data


if __name__ == '__main__':
    rospy.init_node('main_nav')
    nav_main = NavMain()
    rospy.spin()
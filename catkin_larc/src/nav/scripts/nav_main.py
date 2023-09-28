#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from nav.msg import Drive2TargetAction, Drive2TargetGoal, Drive2TargetResult, Drive2TargetFeedback

class NavMain:
    def __init__(self):
        
        self.drive_target_as = actionlib.SimpleActionServer("drive_to_target", Drive2TargetAction, execute_cb=self.ex_cb_drive_target, auto_start=False)
        self.drive_target_as.start()
        self.drive_target_feedback = Drive2TargetFeedback()
        self.drive_target_result = Drive2TargetResult()
        self.distance_acc_tolerance = 0.01
        self.min_speed = 0.15
        self.max_speed = 0.6
        self.SPEED = 0.175
        self.pos_kP = 0.03
        self.pos_kD = 1.0
        self.rate = rospy.Rate(10)
        self.nav_target_x = 0
        self.nav_target_y = 0
        self.init_nav_target_x = 0
        self.init_nav_target_y = 0
        self.intake_presence = False

        self.pubCmdVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.subOdom = rospy.Subscriber('/odom', Odometry, self.odomCb)
        rospy.Subscriber('/target_point_fb', Point, self.targetPointFbCb)
        rospy.Subscriber('/intake_presence', Bool, self.intakePresenceCb)
        self.nav_odom = Odometry()

        self.run()

    def ex_cb_drive_target(self, goal):
        self.intake_presence = False
        #self.nav_target_x = - goal.target.x + self.nav_odom.pose.pose.position.y
        self.nav_target_x = - goal.target.x
        self.nav_target_y = goal.target.y + self.nav_odom.pose.pose.position.x
        if goal.target.y == -0.3:
            self.nav_target_y = 0.15
        distance_x = abs(goal.target.x)
        distance_y = abs(goal.target.y)
        last_error_x = 0
        last_error_y = 0
        
        self.init_nav_target_x = self.nav_target_x
        self.init_nav_target_y = self.nav_target_y
        self.drive_target_feedback.distance_to_target = Point()
        self.drive_target_feedback.distance_to_target.x = distance_x
        self.drive_target_feedback.distance_to_target.y = distance_y

        success = True

        rospy.loginfo('Starting drive to target')

        while distance_x > self.distance_acc_tolerance:
            if self.drive_target_as.is_preempt_requested() or self.intake_presence:
                rospy.loginfo('Preempted')
                self.drive_target_as.set_preempted()
                msg = Twist()
                self.pubCmdVel.publish( msg )
                success = False
                break
            error_x = self.nav_target_x #- self.nav_odom.pose.pose.position.y
            msg = Twist()
            #msg.linear.y = error_x * self.pos_kP + (error_x - last_error_x) * self.pos_kD
            # get the sign of error and multiply with 0.4
            msg.linear.y = self.SPEED * error_x / abs(error_x)
            if abs(msg.linear.y) < self.min_speed:
                msg.linear.y = self.min_speed * msg.linear.y / abs(msg.linear.y)
            if abs(msg.linear.y) > self.max_speed:
                msg.linear.y = self.max_speed * msg.linear.y / abs(msg.linear.y)
            last_error_x = error_x
            self.pubCmdVel.publish( msg )

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
            error_y = self.nav_target_y - self.nav_odom.pose.pose.position.x
            error_x = self.nav_target_x #- self.nav_odom.pose.pose.position.y
            msg = Twist()
            #msg.linear.x = error_y * self.pos_kP + (error_y - last_error_y) * self.pos_kD
            # get the sign of error and multiply with 0.4
            msg.linear.x = self.SPEED * error_y / abs(error_y) 
            msg.linear.y = error_x * self.pos_kP * self.SPEED * error_x / abs(error_x)
            if abs(msg.linear.x) < self.min_speed:
                msg.linear.x = self.min_speed * msg.linear.x / abs(msg.linear.x)
            if abs(msg.linear.x) > self.max_speed:
                msg.linear.x = self.max_speed * msg.linear.x / abs(msg.linear.x)
            last_error_y = error_y
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
            self.drive_target_as.set_preempted()
            print("intake presence")
            msg = Twist()
            msg.linear.x = 0
            msg.linear.y = 0
            msg.linear.z = 0
            msg.angular.x = 0
            msg.angular.y = 0
            msg.angular.z = 0
            self.pubCmdVel.publish(msg)


    def targetPointFbCb(self, data):
        #if abs(-data.x + self.nav_odom.pose.pose.position.y - self.init_nav_target_x) > 0.1 or abs(data.y + self.nav_odom.pose.pose.position.x - self.init_nav_target_y) > 0.1:
            #return
        self.nav_target_x = - data.x #+ self.nav_odom.pose.pose.position.y
        self.nav_target_y = data.y + self.nav_odom.pose.pose.position.x


    def run(self):
        try:
            while not rospy.is_shutdown():
                self.rate.sleep()
        except KeyboardInterrupt:
            rospy.logwarn("Keyboard interrupt detected, stopping listener")


if __name__ == '__main__':
    rospy.init_node('main_nav')
    nav_main = NavMain()
//ros basic imports
#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include "vision/objectDetectionArray.h"
#include "vision/objectDetection.h"

using namespace std;

class MainEngine{
    private:
        ros::NodeHandle nh;
        ros::Publisher pub_cmd_vel;
        ros::Publisher pub_intake;
        ros::Publisher pub_elevator;
        ros::Publisher pub_warehouse;
        ros::Publisher pub_reset_odom;

        ros::Subscriber sub_odom;
        ros::Suscriber sub_color_detect;
        bool target_reached = true;
        pair<float, float> target_position;
        pair<float, float> current_position;
        float tolerance = 0.03;
        int task = 1;
        float imu_setpoint = 0.0;
        int state = 0;

        ros::Time last_spin_time;
        bool color_sequence_detected = false;


    public:
        MainEngine(){ //constructor
            //init node
            nh = ros::NodeHandle();
            //init publisher
            pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
            pub_intake = nh.advertise<std_msgs::Int32>("intake", 10);
            pub_elevator = nh.advertise<std_msgs::Int32>("elevator", 10);
            pub_warehouse = nh.advertise<std_msgs::Int32>("warehouse", 10);
            pub_reset_odom = nh.advertise<std_msgs::Bool>("reset_odom", 5);

            //init subscriber
            sub_odom = nh.subscribe("odom", 10, &MainEngine::odomCallback, this);
            sub_color_detect = nh.subscribe("color_detect", 100, )

            target_reached = true;
            state = 0;

            last_spin_time = ros::Time::now();
        }

        void run(){
            switch( state ){
                case 0:
                    timed_spin();
                    if( color_sequence_detected ){
                        state = 1;
                    }
                break;
            }
        }

        void timedSpin(){
            if( ros::Time::now().toSec() - last_spin_time.toSec() >= ros::Duration(5) ){
                geometry_msgs::Twist msg;
                msg.linear.x = 0;
                msg.linear.y = 0;
                msg.linear.z = 0;
                msg.angular.x = 0;
                msg.angular.y = 0;
                msg.angular.z = -0.5;
                pub_cmd_vel.publish(msg);
                last_spin_time = ros::Time::now();
            }
        }


        void followTarget(){
            if( target_reached ){
                target_position = readTarget();
                target_reached = false;
            }
            else{
                if( abs(current_position.first - target_position.first) <= tolerance && abs(current_position.second - target_position.second) <= tolerance ){
                    target_reached = true;
                    //rosinfo target reached
                    ROS_INFO("Target reached");
                    //stop
                    geometry_msgs::Twist msg;
                    msg.linear.x = 0;
                    msg.linear.y = 0;
                    msg.linear.z = 0;
                    msg.angular.x = 0;
                    msg.angular.y = 0;
                    msg.angular.z = 0;
                    pub_cmd_vel.publish(msg);
                }
                else {
                    //move to target
                    //rosinfo target
                    ROS_INFO("Moving to target from current (X,Y): ");
                    ROS_INFO("X: %f", current_position.first);
                    ROS_INFO("Y: %f", current_position.second);

                    geometry_msgs::Twist msg;
                    msg.linear.x = (target_position.first - current_position.first) * 0.7;
                    msg.linear.y = (target_position.second - current_position.second) * 0.7;
                    msg.linear.z = 0;
                    msg.angular.x = 0;
                    msg.angular.y = 0;
                    msg.angular.z = 0;

                    pub_cmd_vel.publish(msg);
                }
            } 
        }

        pair<float, float> readTarget(){
            //rosinfo target
            ROS_INFO("Enter target (X,Y): ");
            float x, y;
            cin>>x;
            cin>>y;
            return make_pair(x, y);
        }

        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
            //get current position
            current_position = make_pair(msg->pose.pose.position.x, msg->pose.pose.position.y);
        }

        void colorDetectCallback(const vision::objectDetectionArray::ConstPtr& msg){
            float detection_max_size = 0;
            float detection_max_y = 0;
            for( auto detection : msg->detections ){
                if( detection.xmax - detection.xmin > detection_max_size){
                    detection_max_size = detection.xmax - detection.xmin;
                    detection_max_y = (detection.ymax + detection.ymin) / 2;
                }
            }
            //find sequential detections with similar y as max_y
            float detection_max_y_tolerance = 30;
            float detection_x_distance_tolerance = 30;
            int detection_count = 0;
            for( auto detection : msg->detections ){
                if( abs(detection_max_y - (detection.ymax + detection.ymin)/2) <= detection_max_y_tolerance ){
                    detection_count++;
                }
            }
            if( detection_count >= 3 ){
                color_sequence_detected = true;
            }
            else{
                color_sequence_detected = false;
            }
        }

        void manualCommand(){
            char key;
            int command;
            std_msgs::Int32 msg;
            cin>>key;
            cin>>command;

            switch(key){
                case 'Q':
                    msg.data = command;
                    pub_intake.publish(msg);
                    break;
                case 'q':
                    msg.data = command;
                    pub_intake.publish(msg);
                    break;
                case 'E':
                    msg.data = command;
                    pub_elevator.publish(msg);
                    break;
                case 'e':
                    msg.data = command;
                    pub_elevator.publish(msg);
                    break;
                case 'W':
                    msg.data = command;
                    pub_warehouse.publish(msg);
                    break;
                case 'w':
                    msg.data = command;
                    pub_warehouse.publish(msg);
                    break;
                case 'r':
                    std_msgs::Bool msgB;
                    msgB.data = true;
                    pub_reset_odom.publish(msgB);
                    break;
            }
        }
};

int main(int argc, char **argv){
    //init ros
    ros::init(argc, argv, "MainEngine");
    MainEngine mainEngine;

    while (ros::ok()){
        mainEngine.run();
        ros::Rate(10).sleep();
        ros::spinOnce(); 
    }
    return 0;
}
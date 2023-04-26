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
        ros::Publisher pub_global_setpoint;

        ros::Subscriber sub_odom;
        ros::Subscriber sub_color_detect;
        bool target_reached = true;
        pair<float, float> target_position;
        pair<float, float> current_position;
        float tolerance = 0.03;
        int task = 1;
        float imu_setpoint = 0.0;
        int state = 0;

        ros::Time last_spin_time;
        ros::Time state_time;
        bool color_sequence_detected = false;
        bool color_detected[4] = {false, false, false, false};
        int seq_id = 0;
        float detection_max_size = 0;
        float detection_max_y = 0;
        float detection_y_pos[5] = {0, 0, 0, 0, 0};
        string static_color_seq = "gbyrybg";

        geometry_msgs::Twist twist_msg;
        std_msgs::Int32 int_msg;
        std_msgs::Bool bool_msg;

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
            pub_global_setpoint = nh.advertise<std_msgs::Bool>("global_setpoint", 10);

            //init subscriber
            sub_odom = nh.subscribe("odom", 10, &MainEngine::odomCallback, this);
            sub_color_detect = nh.subscribe("color_detect", 20, &MainEngine::colorDetectCallback, this);

            target_reached = true;
            state = 0;

            last_spin_time = ros::Time::now();
        }

        void run(){
            switch( state ){
                case 0:
                    timedSpin(-1);
                    if( color_sequence_detected ){
                        ROS_INFO("0");
                        bool_msg.data = true;
                        pub_global_setpoint.publish(bool_msg);
                        state_time = ros::Time::now();
                        state = 1;
                    } 
                    break;
                case 1:
                    if( ros::Time::now() - state_time >= ros::Duration(3) ){ 
                        ROS_INFO("1");          
                        bool_msg.data = true;
                        pub_reset_odom.publish(bool_msg);

                        target_position = make_pair(0, -0.5);
                        target_reached = false;
                        state = 2;
                    }
                    break;
                case 2:
                    followTarget();
                    if( target_reached ){
                        ROS_INFO("2");
                        twist_msg.linear.x = 0;
                        twist_msg.linear.y = 0;
                        twist_msg.linear.z = 0;
                        twist_msg.angular.x = 0;
                        twist_msg.angular.y = 0;
                        twist_msg.angular.z = 0;
                        pub_cmd_vel.publish(twist_msg);
                        state_time = ros::Time::now();

                        state = 3;
                    }
                    break;
                case 3:
                    if( ros::Time::now() - state_time >= ros::Duration(3) ){                        
                        bool_msg.data = true;
                        pub_reset_odom.publish(bool_msg);
                        int_msg.data = 1;
                        pub_intake.publish(int_msg);

                        target_position = make_pair(-0.2, 0);
                        target_reached = false;
                        state = 4;
                    }
                    break;
            }
        }

        void timedSpin(int dir){
            if( ros::Time::now() - last_spin_time >= ros::Duration(4) ){
                geometry_msgs::Twist msg;
                msg.linear.x = 0;
                msg.linear.y = 0;
                msg.linear.z = 0;
                msg.angular.x = 0;
                msg.angular.y = 0;
                msg.angular.z = 0.5 * dir;
                pub_cmd_vel.publish(msg);
                last_spin_time = ros::Time::now();
            }
        }


        void followTarget(){
            if( !target_reached ){
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
            int sz = msg->detections.size();
            if( sz == 0 )
                return;
            float y_min_first = msg->detections[0].ymin;
            float x_last_max = msg->detections[0].xmax;
            string color_seq = "";
            for( int i=1; i<sz; i++ ){
                if( abs( msg->detections[i].ymin - y_min_first ) >= 50 || abs( msg->detections[i].xmin - x_last_max ) >= 50 )
                    break;
                if( msg->detections[i].labelText == "rojo" )
                    color_seq += "r";
                else if( msg->detections[i].labelText == "verde" )
                    color_seq += "g";
                else if( msg->detections[i].labelText == "azul" )
                    color_seq += "b";
                else if( msg->detections[i].labelText == "amarillo" )
                    color_seq += "y";

                x_last_max = msg->detections[i].xmax;
            }

            //check if color sequence is subsequence of static color sequence
            if( static_color_seq.find(color_seq) != string::npos && color_seq.size() >= 2 ){
                color_sequence_detected = true;
            }

            ROS_INFO("colors: %s", color_seq.c_str());
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
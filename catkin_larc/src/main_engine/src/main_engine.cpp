//ros basic imports
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>

#include "vision/objectDetectionArray.h"
#include "vision/objectDetection.h" 

bool redDetected = false;
float redX = 0;

void colorCallback(const vision::objectDetectionArray::ConstPtr& msg){
    ROS_INFO("I heard");
    for( auto detection : msg->detections){
        if(detection.labelText == "rojo"){
            redDetected = true;
            redX = detection.point3D.x;
        }
    }
}

int main(int argc, char **argv){
    //init ros
    ros::init(argc, argv, "main_engine");
    ros::NodeHandle n;
    //init publisher
    //ros::Publisher pub = n.advertise<std_msgs::Float64>("motor", 1000);
    ros::Publisher pub_drive = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    //init subscriber
    //ros::Subscriber sub = n.subscribe("cmd_vel", 1000, &cmd_velCallback);
    ros::Subscriber sub_colors = n.subscribe("color_detect", 5, &colorCallback);
    //init rate
    ros::Rate loop_rate(10);
    //init msg
    //std_msgs::Float64 msg;
    //init msg data
    //msg.data = 0;
    //init counter
    int count = 0;

    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;
    //main loop
    while (ros::ok()){
        //publish msg
        if( redDetected ){
            if( redX < 0.2 ){
                msg.angular.z = 0.2;
            }else if( redX > 0.2 ){
                msg.angular.z = -0.2;
            } else {
                msg.angular.z = 0;
            }
        } else {
            msg.angular.z = 0.5;
        }
        pub_drive.publish(msg);
        //spin
        ros::spinOnce();
        //sleep
        loop_rate.sleep();
        //increment counter
        ++count;
    }
    return 0;
}
//ros basic imports
#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

using namespace std;

class MainEngine{
    public:
        MainEngine(){ //constructor
            //init node
            nh = ros::NodeHandle();
            //init publisher
            pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
            pub_intake = nh.advertise<std_msgs::Int32>("intake", 10);
            pub_elevator = nh.advertise<std_msgs::Int32>("elevator", 10);
            pub_warehouse_m = nh.advertise<std_msgs::Int32>("warehouse_m", 10);

            //init subscriber
            sub_odom = nh.subscribe("odom", 10, &MainEngine::odomCallback, this);

            target_reached = true;

        }

        void run(){
            if( task == 1 ){
                manualCommand();
                return;
            }

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
                    pub_warehouse_m.publish(msg);
                    break;
                case 'w':
                    msg.data = command;
                    pub_warehouse_m.publish(msg);
                    break;
            }
        }
    private:
        ros::NodeHandle nh;
        ros::Publisher pub_cmd_vel;
        ros::Publisher pub_intake;
        ros::Publisher pub_elevator;
        ros::Publisher pub_warehouse_m;

        ros::Subscriber sub_odom;
        bool target_reached = true;
        pair<float, float> target_position;
        pair<float, float> current_position;
        float tolerance = 0.03;
        int task = 1;
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
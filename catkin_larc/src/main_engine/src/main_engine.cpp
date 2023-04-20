//ros basic imports
#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Int32.h>

using namespace std;

class MainEngine{
    public:
        MainEngine(){ //constructor
            //init node
            nh = ros::NodeHandle();
            //init publisher
            pub_intake = nh.advertise<std_msgs::Int32>("intake", 10);
            pub_elevator = nh.advertise<std_msgs::Int32>("elevator", 10);
            pub_warehouse_m = nh.advertise<std_msgs::Int32>("warehouse_m", 10);

            //init rate
        }
        void run(){
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
        ros::Publisher pub_intake;
        ros::Publisher pub_elevator;
        ros::Publisher pub_warehouse_m;
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
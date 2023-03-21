//ros basic imports
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

//ros main function to suscribe to cmd_vel and publish to motor
int main(int argc, char **argv)
{
    //init ros
    ros::init(argc, argv, "main_engine");
    ros::NodeHandle n;
    //init publisher
    ros::Publisher pub = n.advertise<std_msgs::Float64>("motor", 1000);
    //init subscriber
    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, &cmd_velCallback);
    //init rate
    ros::Rate loop_rate(10);
    //init msg
    std_msgs::Float64 msg;
    //init msg data
    msg.data = 0;
    //init counter
    int count = 0;
    //main loop
    while (ros::ok())
    {
        //publish msg
        pub.publish(msg);
        //spin
        ros::spinOnce();
        //sleep
        loop_rate.sleep();
        //increment counter
        ++count;
    }
    return 0;
}
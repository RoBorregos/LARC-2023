

//ros connection test
#include <ros.h>
#include <geometry_msgs/Twist.h>

void messageCb( const geometry_msgs::Twist& msg){
    Serial.println("I heard: ");
    Serial.println(msg.linear.x);
    Serial.println(msg.linear.y);
    Serial.println(msg.linear.z);
    Serial.println(msg.angular.x);
    Serial.println(msg.angular.y);
    Serial.println(msg.angular.z);
    digitalWrite(13, HIGH-digitalRead(13));
}

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );

void setup() {
    pinMode(13, OUTPUT);
    nh.initNode();
    nh.subscribe(sub);
    Serial.begin(9600);
}

void loop() {
//    Serial.println( digitalRead(13) ); 
    //Serial.println( Serial.read() );
    /*if( Serial.read() != -1 ){
        digitalWrite(13, HIGH);
    }
    else{
        digitalWrite(13, LOW);
    }*/
    nh.spinOnce();
    delay(1);
}

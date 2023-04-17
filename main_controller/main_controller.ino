#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>

#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#include "Constants.h"
#include "Drive.h"
#include "Motor.h"
#include "Intake.h"
#include "Elevator.h"
#include "LineSensor.h"
#include "Warehouse.h"

Drive mDrive;
Elevator mElevator;
Intake mIntake;
Warehouse mWarehouse;

unsigned long debug_time = 0;
float targetSpeed = 0.8;
int state = 0;
unsigned long state_time = 0;
unsigned long loop_time = 0;

float linear_x = 0;
float linear_y = 0;
float angular_z = 0;
float theta_error = 0;

Adafruit_VL53L0X sensor1;
Adafruit_VL53L0X sensor2;

void teleop( const geometry_msgs::Twist& msg){
    Serial.println("I heard: ");
    Serial.println(msg.linear.x);
    Serial.println(msg.linear.y);
    Serial.println(msg.linear.z);
    Serial.println(msg.angular.x);
    Serial.println(msg.angular.y);
    Serial.println(msg.angular.z);
    linear_x = msg.linear.x;
    linear_y = msg.linear.y;
    angular_z = msg.angular.z;
    digitalWrite(13, HIGH-digitalRead(13));
}

void imu_read( const geometry_msgs::Vector3& msg2){
    theta_error = msg2.z;
    //digitalWrite(13, HIGH-digitalRead(13));
}

//ros::NodeHandle nh;
//ros::Subscriber<geometry_msgs::Twist> sub_drive("cmd_vel", &teleop );
//ros::Subscriber<geometry_msgs::Vector3> sub_imu("imu_rpy", &imu_read );

unsigned long current_time = 0;

void setup(){
    current_time = millis();

    Wire1.begin();
    Wire2.begin();
    
    sensor1.begin(0x29, false, &Wire1, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT);
    sensor2.begin(0x29, false, &Wire2, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT);

    mDrive.init(&theta_error);
    mWarehouse.init(current_time, &sensor2);
    mElevator.setSpeed(1500);
    Serial.begin(9600);

    //Serial.write("<target>");
    attachInterrupt(digitalPinToInterrupt(Constants::kFrontLeftEncoder), interruptFL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Constants::kFrontRightEncoder), interruptFR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Constants::kBackLeftEncoder), interruptBL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Constants::kBackRightEncoder), interruptBR, CHANGE);

    state_time = current_time;
    loop_time = current_time;
    debug_time = current_time;
    //mElevator.setPosition(ElevatorPosition::SecondWarehouse);

    //nh.initNode();
    //nh.subscribe(sub_drive);
    //nh.subscribe(sub_imu);
}

void loop(){

    current_time = millis();

    if( current_time - loop_time > 10 ){
        mDrive.periodicIO(current_time);
        mIntake.periodicIO(current_time);
        //nh.spinOnce();
        loop_time = current_time;
    }

    mDrive.setSpeed(linear_x, linear_y, angular_z);
    //mDrive.setSpeed(0.5, 0, 0);
    //mDrive.periodicIO();

    //mElevator.setPosition(ElevatorPosition::SecondWarehouse);
    //mElevator.periodicIO();

    // Plot (TODO: make a library for this)
    if( current_time - debug_time > 50 ){
        //mWarehouse.periodicIO(current_time);
        //Serial.println(mDrive.getSpeed(MotorID::FrontLeft));
        //plotData(mDrive.getSpeed(MotorID::FrontLeft), mDrive.getSpeed(MotorID::FrontRight), mDrive.getSpeed(MotorID::BackLeft), mDrive.getSpeed(MotorID::BackRight), targetSpeed);
        Pose2d pose = mDrive.getPosition();
        Serial.print(pose.x);
        Serial.print(" ");
        Serial.print(pose.y);
        Serial.print(" ");
        Serial.println(pose.theta);
        debug_time = current_time;
    }
    //delay(10);
}

void plotData(float data1, float data2, float data3, float data4, float data5){
    const byte *byteData1 = (byte *)(&data1);
    const byte *byteData2 = (byte *)(&data2);
    const byte *byteData3 = (byte *)(&data3);
    const byte *byteData4 = (byte *)(&data4);
    const byte *byteData5 = (byte *)(&data5);

    const byte buf[20] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                          byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                          byteData3[0], byteData3[1], byteData3[2], byteData3[3],
                          byteData4[0], byteData4[1], byteData4[2], byteData4[3],
                          byteData5[0], byteData5[1], byteData5[2], byteData5[3]};

    //Serial.write(buf, 20);
    for(int i=0; i<20; i++){
        Serial.print(buf[i]);
        Serial.print(" ");
    }
    Serial.println();
}

void interruptFL(){
    mDrive.encoderInterrupt(MotorID::FrontLeft);
}
void interruptFR(){
    mDrive.encoderInterrupt(MotorID::FrontRight);
}
void interruptBL(){
    mDrive.encoderInterrupt(MotorID::BackLeft);
}
void interruptBR(){
    mDrive.encoderInterrupt(MotorID::BackRight);
}
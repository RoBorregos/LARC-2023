#include "Constants.h"
#include "Drive.h"
#include "Motor.h"

Drive mDrive;

void setup(){
    mDrive.init();
    Serial.begin(9600);
    attachInterrupt(digitalPinToInterrupt(Constants::kFrontLeftEncoder), interruptFL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Constants::kFrontRightEncoder), interruptFR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Constants::kBackLeftEncoder), interruptBL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Constants::kBackRightEncoder), interruptBR, CHANGE);
}

void loop(){
    mDrive.setSpeed(0.8, 0, 0);
    mDrive.periodicIO();
    Serial.println(mDrive.getSpeed(MotorID::BackRight));
    delay(50);
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
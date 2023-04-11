#include "Drive.h"

void Drive::init(){
    frontLeft.init(Constants::kFrontLeftA, Constants::kFrontLeftB, Constants::kFrontLeftEncoder);
    frontRight.init(Constants::kFrontRightA, Constants::kFrontRightB, Constants::kFrontRightEncoder);
    backLeft.init(Constants::kBackLeftA, Constants::kBackLeftB, Constants::kBackLeftEncoder);
    backRight.init(Constants::kBackRightA, Constants::kBackRightB, Constants::kBackRightEncoder);
}

void Drive::setSpeed(float linearX, float linearY, float angularZ, float error){
    float wheelPosX = Constants::kWheelBase/2;
    float wheelPosY = Constants::kWheelTrack/2; 
    float frontLeftSpeed = linearX - linearY - angularZ*(wheelPosX + wheelPosY) - error;
    float frontRightSpeed = linearX + linearY + angularZ*(wheelPosX + wheelPosY) - error;
    float backLeftSpeed = linearX + linearY - angularZ*(wheelPosX + wheelPosY) - error;
    float backRightSpeed = linearX - linearY + angularZ*(wheelPosX + wheelPosY) - error;

    frontLeft.setSpeed(frontLeftSpeed);
    frontRight.setSpeed(frontRightSpeed);
    backLeft.setSpeed(backLeftSpeed);
    backRight.setSpeed(backRightSpeed);
}

void Drive::stop(){
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
}

void Drive::encoderInterrupt(MotorID motorID){
    switch(motorID){
        case MotorID::FrontLeft:
            frontLeft.encoderInterrupt();
            break;
        case MotorID::FrontRight:
            frontRight.encoderInterrupt();
            break;
        case MotorID::BackLeft:
            backLeft.encoderInterrupt();
            break;
        case MotorID::BackRight:
            backRight.encoderInterrupt();
            break;
    }
}

float Drive::getSpeed(MotorID motorID){
    switch(motorID){
        case MotorID::FrontLeft:
            return frontLeft.getSpeed();
            break;
        case MotorID::FrontRight:
            return frontRight.getSpeed();
            break;
        case MotorID::BackLeft:
            return backLeft.getSpeed();
            break;
        case MotorID::BackRight:
            return backRight.getSpeed();
            break;
    }
    return -1;
}

long Drive::getTicks(MotorID motorID){
    switch(motorID){
        case MotorID::FrontLeft:
            return frontLeft.getTicks();
            break;
        case MotorID::FrontRight:
            return frontRight.getTicks();
            break;
        case MotorID::BackLeft:
            return backLeft.getTicks();
            break;
        case MotorID::BackRight:
            return backRight.getTicks();
            break;
    }
    return -1;
}

void Drive::periodicIO(){
    frontLeft.periodicIO();
    frontRight.periodicIO();
    backLeft.periodicIO();
    backRight.periodicIO();
}
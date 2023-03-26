#include "Drive.h"

Drive::Drive(){
    frontLeft = Motor(Constants::kFrontLeftA, Constants::kFrontLeftB, Constants::kFrontLeftEncoder);
    frontRight = Motor(Constants::kFrontRightA, Constants::kFrontRightB, Constants::kFrontRightEncoder);
    backLeft = Motor(Constants::kBackLeftA, Constants::kBackLeftB, Constants::kBackLeftEncoder);
    backRight = Motor(Constants::kBackRightA, Constants::kBackRightB, Constants::kBackRightEncoder);
    kinematics = Kinematics(Constants::kWheelTrack, Constants::kWheelBase);
}

void Drive::setSpeed(float linearX, float linearY, float angularZ){
    wheelSpeeds = kinematics.getWheelSpeeds(linearX, linearY, angularZ);
    frontLeft.setSpeed(wheelSpeeds.frontLeft);
    frontRight.setSpeed(wheelSpeeds.frontRight);
    backLeft.setSpeed(wheelSpeeds.backLeft);
    backRight.setSpeed(wheelSpeeds.backRight);
    Serial.println();
}

void Drive::stop(){
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
}
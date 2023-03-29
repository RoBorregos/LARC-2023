#include "Drive.h"

void Drive::init(){
    frontLeft.init(Constants::kFrontLeftA, Constants::kFrontLeftB, Constants::kFrontLeftEncoder);
    frontRight.init(Constants::kFrontRightA, Constants::kFrontRightB, Constants::kFrontRightEncoder);
    backLeft.init(Constants::kBackLeftA, Constants::kBackLeftB, Constants::kBackLeftEncoder);
    backRight.init(Constants::kBackRightA, Constants::kBackRightB, Constants::kBackRightEncoder);
}

void Drive::setSpeed(float linearX, float linearY, float angularZ){
    float wheelPosX = Constants::kWheelBase/2;
    float wheelPosY = Constants::kWheelTrack/2; 
    float frontLeftSpeed = linearX - linearY - angularZ*(wheelPosX + wheelPosY);
    float frontRightSpeed = linearX + linearY + angularZ*(wheelPosX + wheelPosY);
    float backLeftSpeed = linearX + linearY - angularZ*(wheelPosX + wheelPosY);
    float backRightSpeed = linearX - linearY + angularZ*(wheelPosX + wheelPosY);

    frontLeft.setSpeed(frontLeftSpeed);
    frontRight.setSpeed(frontRightSpeed);
    backLeft.setSpeed(backLeftSpeed);
    backRight.setSpeed(backRightSpeed);
}

void Drive::periodicIO(){
    frontLeft.periodicIO();
    frontRight.periodicIO();
    backLeft.periodicIO();
    backRight.periodicIO();
}
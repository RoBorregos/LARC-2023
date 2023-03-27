#include "Drive.h"

void Drive::init(){
    frontLeft = Motor(Constants::kFrontLeftA, Constants::kFrontLeftB, Constants::kFrontLeftEncoder);
    frontRight = Motor(Constants::kFrontRightA, Constants::kFrontRightB, Constants::kFrontRightEncoder);
    backLeft = Motor(Constants::kBackLeftA, Constants::kBackLeftB, Constants::kBackLeftEncoder);
    backRight = Motor(Constants::kBackRightA, Constants::kBackRightB, Constants::kBackRightEncoder);
}

void Drive::setSpeed(float linearX, float linearY, float angularZ){
    float frontLeftSpeed = linearX + linearY + angularZ;
    float frontRightSpeed = linearX - linearY - angularZ;
    float backLeftSpeed = linearX - linearY + angularZ;
    float backRightSpeed = linearX + linearY - angularZ;

    frontLeft.setSpeed(frontLeftSpeed);
    frontRight.setSpeed(frontRightSpeed);
    backLeft.setSpeed(backLeftSpeed);
    backRight.setSpeed(backRightSpeed);
}
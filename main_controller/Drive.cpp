#include "Drive.h"

void Drive::init(){
    frontLeft.init(Constants::kFrontLeftPWM, Constants::kFrontLeftA, Constants::kFrontLeftB, Constants::kFrontLeftEncoder);
    frontRight.init(Constants::kFrontRightPWM, Constants::kFrontRightA, Constants::kFrontRightB, Constants::kFrontRightEncoder);
    backLeft.init(Constants::kBackLeftPWM, Constants::kBackLeftA, Constants::kBackLeftB, Constants::kBackLeftEncoder);
    backRight.init(Constants::kBackRightPWM, Constants::kBackRightA, Constants::kBackRightB, Constants::kBackRightEncoder);
}

void Drive::setSpeed(float linearX, float linearY, float angularZ){
    float error = (angle-setpoint) * Constants::kDriveKP;
    float wheelPosX = Constants::kWheelBase/2;
    float wheelPosY = Constants::kWheelTrack/2;
    if(abs(angularZ) > 0.2)
        setpoint = angle;
    
    float frontLeftSpeed = linearX - linearY - angularZ*(wheelPosX + wheelPosY) - error;
    float frontRightSpeed = linearX + linearY + angularZ*(wheelPosX + wheelPosY) + error;
    float backLeftSpeed = linearX + linearY - angularZ*(wheelPosX + wheelPosY) - error;
    float backRightSpeed = linearX - linearY + angularZ*(wheelPosX + wheelPosY) + error;

    frontLeft.setSpeed(frontLeftSpeed);
    frontRight.setSpeed(frontRightSpeed);
    backLeft.setSpeed(backLeftSpeed);
    backRight.setSpeed(backRightSpeed);
}

void Drive::setAngle(float angle){
    this->angle = angle;
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

Pose2d Drive::getChassisSpeeds(){
    return velocity;
}

Pose2d Drive::getPosition(){
    return position;
}

void Drive::resetOdometry(){
    frontLeft.resetEncoder();
    frontRight.resetEncoder();
    backLeft.resetEncoder();
    backRight.resetEncoder();
    
    velocity.x = 0;
    velocity.y = 0;
    velocity.theta = 0;
    position.x = 0;
    position.y = 0;
    position.theta = 0;
}

void Drive::periodicIO(unsigned long current_time){
    if( current_time - last_time < loop_time)
        return;

    frontLeft.periodicIO(current_time);
    frontRight.periodicIO(current_time);
    backLeft.periodicIO(current_time);
    backRight.periodicIO(current_time);

    velocity.x = (frontLeft.getSpeed() + frontRight.getSpeed() + backLeft.getSpeed() + backRight.getSpeed())/4;
    velocity.y = (-frontLeft.getSpeed() + frontRight.getSpeed() + backLeft.getSpeed() - backRight.getSpeed())/4;
    velocity.theta = (-frontLeft.getSpeed() + frontRight.getSpeed() - backLeft.getSpeed() + backRight.getSpeed())/(4* (Constants::kWheelBase/2 + Constants::kWheelTrack/2));

    unsigned long delta_time = current_time - last_time;
    position.x += (velocity.x * cos(angle) + velocity.y * sin(angle)) * (delta_time * 0.001);
    position.y += (velocity.x * sin(angle) + velocity.y * cos(angle)) * (delta_time * 0.001);
    position.theta = angle;

    last_time = current_time;
}
#include "Drive.h"

void Drive::init(){
    frontLeft.init(Constants::kFrontLeftA, Constants::kFrontLeftB, Constants::kFrontLeftEncoder);
    frontRight.init(Constants::kFrontRightA, Constants::kFrontRightB, Constants::kFrontRightEncoder);
    backLeft.init(Constants::kBackLeftA, Constants::kBackLeftB, Constants::kBackLeftEncoder);
    backRight.init(Constants::kBackRightA, Constants::kBackRightB, Constants::kBackRightEncoder);
    line_move = false;
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

// setSpeed with a time counter
void Drive::setSpeed(float linearX, float linearY, float angularZ, unsigned long current_time){

    if( current_time - speed_last_time < pid_time)
        return;
    
    float wheelPosX = Constants::kWheelBase/2;
    float wheelPosY = Constants::kWheelTrack/2;
    
    float frontLeftSpeed = linearX - linearY - angularZ*(wheelPosX + wheelPosY);
    float frontRightSpeed = linearX + linearY + angularZ*(wheelPosX + wheelPosY);
    float backLeftSpeed = linearX + linearY - angularZ*(wheelPosX + wheelPosY);
    float backRightSpeed = linearX - linearY + angularZ*(wheelPosX + wheelPosY);

    frontLeft.setSpeed(frontLeftSpeed, current_time);
    frontRight.setSpeed(frontRightSpeed, current_time);
    backLeft.setSpeed(backLeftSpeed, current_time);
    backRight.setSpeed(backRightSpeed, current_time);

    speed_last_time = current_time;
}

void Drive::setAngle(float angle){
    if(angle == 500){
        this->angle = setpoint;
        return;
    }
    this->angle = angle - global_setpoint;
    if(this->angle > 180)
        this->angle -= 360;
    else if(this->angle < -180)
        this->angle += 360;
}

void Drive::setGlobalSetpoint(){
    this->global_setpoint = angle;
    setpoint = 0;
}

void Drive::stop(){
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
}

void Drive::hardStop(){
    frontLeft.hardStop();
    frontRight.hardStop();
    backLeft.hardStop();
    backRight.hardStop();
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
    float angleRad = angle * PI/180;
    // Serial.print("X velocity: "); Serial.print(velocity.x); Serial.print(" Y velocity: "); Serial.print(velocity.y); Serial.print(" Theta velocity: "); Serial.println(velocity.theta);
    // Serial.print("FL PWM: "); Serial.print(frontLeft.getPWM()); Serial.print(" FR PWM: "); Serial.print(frontRight.getPWM()); Serial.print(" BL PWM: "); Serial.print(backLeft.getPWM()); Serial.print(" BR PWM: "); Serial.println(backRight.getPWM());
    position.x += velocity.x  * (delta_time * 0.001);
    position.y += velocity.y  * (delta_time * 0.001);
    position.theta = angle;

    error = (angle - setpoint);
    //map error to -180 to 180
    if(error > 180){
        error -= 360;
    }else if(error < -180){
        error += 360;
    }
    error = error * Constants::kDriveKP + (error - last_error)/delta_time * Constants::kDriveKD;

    last_time = current_time;
}

void Drive::setVerbose(bool verbose){
    this->verbose = verbose;
    // set motors to verbose
    frontLeft.setVerbose(verbose);
    frontRight.setVerbose(verbose);
    backLeft.setVerbose(verbose);
    backRight.setVerbose(verbose);
}
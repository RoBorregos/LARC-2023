#include "Drive.h"

void Drive::init( LineSensor *lineSensor ){
    frontLeft.init(Constants::kFrontLeftPWM, Constants::kFrontLeftA, Constants::kFrontLeftB, Constants::kFrontLeftEncoder);
    frontRight.init(Constants::kFrontRightPWM, Constants::kFrontRightA, Constants::kFrontRightB, Constants::kFrontRightEncoder);
    backLeft.init(Constants::kBackLeftPWM, Constants::kBackLeftA, Constants::kBackLeftB, Constants::kBackLeftEncoder);
    backRight.init(Constants::kBackRightPWM, Constants::kBackRightA, Constants::kBackRightB, Constants::kBackRightEncoder);
    this->lineSensor = lineSensor;
    line_move = false;
}

void Drive::setSpeed(float linearX, float linearY, float angularZ){
    if( line_move ){
        if( line_state==1 ){
            if( lineSensor->lineDetected(SensorID::FrontLeft1) ){
                frontLeft.setSpeed(0.7);
                frontRight.setSpeed(0.7);
                backLeft.setSpeed(0.7);
                backRight.setSpeed(0.7);
            }
        }
        if( line_state==2 ){
            if( !lineSensor->lineDetected(SensorID::FrontLeft1) ){
                frontLeft.setSpeed(0.6);
                frontRight.setSpeed(0.6);
                backLeft.setSpeed(0.6);
                backRight.setSpeed(0.6);
            }
        }
        return;
    }

    float wheelPosX = Constants::kWheelBase/2;
    float wheelPosY = Constants::kWheelTrack/2;
    if(abs( angularZ ) > 0.2 && !spin_flag){
        if( angularZ < 0 )
            setpoint += 90;
        else
            setpoint -= 90;
        spin_flag = true;
    } else if( abs(angularZ) == 0 && spin_flag ){
        spin_flag = false;
    }

    if( abs(linearY) > 0.2 && !line_move ){
        line_move = true;
        line_state = 1;
        return;
    }
    
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
    this->angle = angle - global_setpoint;
}

void Drive::setGlobalSetpoint(){
    this->global_setpoint = setpoint;
    setpoint = 0;
    //180 to -180
    if(this->angle > 180)
        this->angle -= 360;
    else if(this->angle < -180)
        this->angle += 360;
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

    if( line_move ){
        if( line_state==1 ){
            if( !lineSensor->lineDetected(SensorID::FrontLeft1) )
                line_state = 2;
        }
        if( line_state==2 ){
            if( lineSensor->lineDetected(SensorID::FrontLeft1) ){
                frontLeft.stop();
                frontRight.stop();
                backLeft.stop();
                backRight.stop();
                line_state = 0;
                line_move = false;
            }
        }
    }

    frontLeft.periodicIO(current_time);
    frontRight.periodicIO(current_time);
    backLeft.periodicIO(current_time);
    backRight.periodicIO(current_time);

    velocity.x = (frontLeft.getSpeed() + frontRight.getSpeed() + backLeft.getSpeed() + backRight.getSpeed())/4;
    velocity.y = (-frontLeft.getSpeed() + frontRight.getSpeed() + backLeft.getSpeed() - backRight.getSpeed())/4;
    velocity.theta = (-frontLeft.getSpeed() + frontRight.getSpeed() - backLeft.getSpeed() + backRight.getSpeed())/(4* (Constants::kWheelBase/2 + Constants::kWheelTrack/2));

    unsigned long delta_time = current_time - last_time;
    float angleRad = angle * PI/180;
    position.x += (velocity.x * cos(angleRad) + velocity.y * sin(angleRad)) * (delta_time * 0.001);
    position.y += (velocity.x * sin(angleRad) + velocity.y * cos(angleRad)) * (delta_time * 0.001);
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
#include "Drive.h"

void Drive::init(BNO *bno, LineSensor *lineSensor){
    pidControllerBNO.set(BNOKP, BNOKI, BNOKD, BNOKImax, BNOKout_min, BNOKout_max);
    frontLeft.init(Constants::kFrontLeftA, Constants::kFrontLeftB, Constants::kFrontLeftEncoder);
    frontRight.init(Constants::kFrontRightA, Constants::kFrontRightB, Constants::kFrontRightEncoder);
    backLeft.init(Constants::kBackLeftA, Constants::kBackLeftB, Constants::kBackLeftEncoder);
    backRight.init(Constants::kBackRightA, Constants::kBackRightB, Constants::kBackRightEncoder);
    pinMode(Constants::kLimitSwitch, INPUT);
    this->bno = bno;
    this->lineSensor = lineSensor;
    bno->init();
    robot_angle = bno->getOrientation().x;
    line_move = false;
}

void Drive::restart(){
    frontLeft.init(Constants::kFrontLeftA, Constants::kFrontLeftB, Constants::kFrontLeftEncoder);
    frontRight.init(Constants::kFrontRightA, Constants::kFrontRightB, Constants::kFrontRightEncoder);
    backLeft.init(Constants::kBackLeftA, Constants::kBackLeftB, Constants::kBackLeftEncoder);
    backRight.init(Constants::kBackRightA, Constants::kBackRightB, Constants::kBackRightEncoder);
    bno->init();
    robot_angle = bno->getOrientation().x;
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

float Drive::getAngleX(){
    return curr_angle_x;
}

// setSpeed with a time counter and BNO feedback
void Drive::setSpeedOriented(float linearX, float linearY, float angularZ, unsigned long current_time){

    if( (current_time - speed_last_time < pid_time))
        return;
        
    if (digitalRead(intake_presence)){
        linearY = 0;   
    }
    if ((digitalRead(intake_presence) && current_time - presence_detection_time > Constants::kIntakePushTime) && !shelf_approach){
        hardStop();
        linearX = 0;
        linearY = 0;
        angularZ = 0;
        flag = false;
    }
    if (shelf_approach){
        linearX = 0.2;
        linearY = 0;
        angularZ = 0;
        // read line sensors and stop if back line is detected
        // back sensors are 5, 6, 7 and 8. Have to detect one of 5 or 6, and one from 7 or 8
        if (digitalRead(Constants::kLimitSwitch)){
            hardStop();
            linearX = 0;
            linearY = 0;
            angularZ = 0;
            shelf_approach = false;
        }
        if (lineSensor->lineDetected(5) || lineSensor->lineDetected(6)){
            if (lineSensor->lineDetected(7) || lineSensor->lineDetected(8)){
                hardStop();
                linearX = 0;
                linearY = 0;
                angularZ = 0;
                shelf_approach = false;
            }
        }
    }

    // set speeds to 0 if they are less than treshold
    if(abs(linearX) < kMinSpeed)
        linearX = 0;
    if(abs(linearY) < kMinSpeed)
        linearY = 0;
    if(abs(angularZ) < kMinSpeed)
        angularZ = 0;
    
    // to help continuous loops (beyond 180) and stability around 0
    float current_angle;
    if (robot_angle < 150){
        current_angle = bno->getOrientation().x;
    }
    else{
        current_angle = bno->getOrientation0to360().x;
    }
    this->curr_angle_x = current_angle;
    /*if (current_angle > 180 || current_angle < -180){
        frontLeft.setSpeed(frontLeftSpeed, current_time);
        frontRight.setSpeed(frontRightSpeed, current_time);
        backLeft.setSpeed(backLeftSpeed, current_time);
        backRight.setSpeed(backRightSpeed, current_time);
    }*/
    // if an angular speed is set, update the angle
    if(angularZ != 0){
        robot_angle = curr_angle_x;
    }
    // get angular speed to compensate angle error, using PID
    else {
        // if difference is higher than a threshold, correct with angular speed
        float angle_difference = current_angle - robot_angle;
        if(abs(angle_difference) > Constants::kAngleTolerance){
            float angular_speed = pidControllerBNO.calculate(robot_angle, current_angle, pid_time);
            angularZ = -angular_speed;
            Serial.print("Angle error: "); Serial.print(current_angle - robot_angle);
            Serial.print("Angular speed: "); Serial.println(angular_speed);
        }
    }
    
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
    /*if(angle == 500){
        this->angle = setpoint;
        return;
    }
    this->angle = angle - global_setpoint;
    if(this->angle > 180)
        this->angle -= 360;
    else if(this->angle < -180)
        this->angle += 360;*/
    robot_angle = angle;
}

void Drive::setApproachShelf(bool approach){
    shelf_approach = approach;
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
    hard_stop_current = hard_stop_time;
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

    bool presence = digitalRead(Constants::kIntakePresence);
    if( presence && !flag ){
        presence_detection_time = current_time;
        flag = true;
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
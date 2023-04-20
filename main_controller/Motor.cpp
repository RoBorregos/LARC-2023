#include "Motor.h"

Motor::Motor(){
    this->motorA = 0;
    this->motorB = 0;
    this->encoder = 0;
}

void Motor::init(int motorA, int motorB, int encoder){
    this->motorA = motorA;
    this->motorB = motorB;
    this->encoder = encoder;

    pinMode(motorA, OUTPUT);
    pinMode(motorB, OUTPUT);
    pinMode(encoder, INPUT_PULLUP);
}

void Motor::encoderInterrupt(){
    if( io.direction )
        io.ticks++;
    else
        io.ticks--;
}

void Motor::periodicIO(unsigned long current_time){
    analogWrite(motorA, io.direction? io.demand : 0);
    analogWrite(motorB, io.direction? 0 : io.demand);

    //overflow
    if( abs(io.ticks) > 2147483647){
        io.ticks = 0;
        io.last_ticks = 0;
    }
    io.delta_time = (current_time - io.last_time) / 1000.0;
    io.delta_ticks = io.ticks - io.last_ticks;
    io.speed = (io.delta_ticks / io.delta_time) * (Constants::kWheelDiameter * PI / (Constants::kEncoderTicksPerRevolution/2) );

    io.last_ticks = io.ticks;
    io.last_time = current_time;
}

// Set the speed of the motor in m/s
void Motor::setSpeed(float speed){
    if( abs(speed) < 0.05 ){
        stop();
        return;
    }
    io.target_speed = speed;
    float current_speed = io.speed + pidController.calculate(speed, io.speed, io.delta_time);
    float pwm = current_speed / getMaxVelocity() * 255;
    setPWM(pwm);
}

// Set the speed of the motor in PWM
void Motor::setPWM(int pwm){
    io.direction = pwm > 0;
    io.demand = min( max( abs(pwm), Constants::kMotorMinPWM), 255);
}

void Motor::stop(){
    io.demand = 0;
    analogWrite(motorA, 0);
    analogWrite(motorB, 0);
}

float Motor::getMaxVelocity(){
    return Constants::kWheelDiameter * PI * Constants::kMotorsRPM / 60;
}

long Motor::getTicks(){
    return io.ticks;
}

float Motor::getSpeed(){
    return io.speed;
}
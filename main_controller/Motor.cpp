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
    io.ticks++;
}

void Motor::periodicIO(){
    analogWrite(motorA, io.direction? io.demand : 0);
    analogWrite(motorB, io.direction? 0 : io.demand);
    Serial.print(io.demand);
    Serial.print(" ");

    unsigned long current_time = millis();
    float delta_time = (current_time - io.last_time) / 1000.0;
    io.delta_ticks = io.ticks - io.last_ticks;
    io.speed = (io.delta_ticks / delta_time) * (Constants::kWheelDiameter * PI / Constants::kEncoderTicksPerRevolution);
}

// Set the speed of the motor in m/s
void Motor::setSpeed(float speed){
   float pwm = speed / getMaxVelocity() * 255;
   setPWM(pwm);
}

// Set the speed of the motor in PWM
void Motor::setPWM(int pwm){
    io.direction = pwm > 0;
    io.demand = abs(pwm);
}

void Motor::stop(){
    analogWrite(motorA, 0);
    analogWrite(motorB, 0);
}

float Motor::getMaxVelocity(){
    return Constants::kWheelDiameter * Constants::kMotorsRPM * PI / 60;
}

long Motor::getTicks(){
    return io.ticks;
}

float Motor::getSpeed(){
    return io.speed;
}
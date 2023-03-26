#include "Motor.h"

Motor::Motor(int motorA, int motorB, int encoder){
    this->motorA = motorA;
    this->motorB = motorB;
    this->encoder = encoder;

    pinMode(motorA, OUTPUT);
    pinMode(motorB, OUTPUT);
    pinMode(encoder, INPUT);
}

float Motor::getMaxVelocity(){
    return Constants::kWheelDiameter * Constants::kMotorsRPM * PI / 60;
}

// Set the speed of the motor in m/s
void Motor::setSpeed(float speed){
   float pwm = speed / getMaxVelocity() * 255;
   setPWM(pwm); 
}

// Set the speed of the motor in PWM
void Motor::setPWM(int pwm){
    bool dir = pwm > 0;
    analogWrite(motorA, dir ? pwm : 0);
    analogWrite(motorB, dir ? 0 : -pwm);
    Serial.print(pwm);
    Serial.print(" ");
}

void Motor::stop(){
    analogWrite(motorA, 0);
    analogWrite(motorB, 0);
}
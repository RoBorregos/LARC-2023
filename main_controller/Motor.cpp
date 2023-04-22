#include "Motor.h"

Motor::Motor(){
    this->pinPWM = 0;
    this->pinA = 0;
    this->pinB = 0;
    this->encoder = 0;
}

void Motor::init(int pinPWM, int pinA, int pinB, int encoder){
    this->pinPWM = pinPWM;
    this->pinA = pinA;
    this->pinB = pinB;
    this->encoder = encoder;

    pinMode(pinPWM, OUTPUT);
    pinMode(pinA, OUTPUT);
    pinMode(pinB, OUTPUT);
    pinMode(encoder, INPUT_PULLUP);
}

void Motor::encoderInterrupt(){
    if( io.direction )
        io.ticks++;
    else
        io.ticks--;
}

void Motor::periodicIO(unsigned long current_time){
    analogWrite(pinPWM, io.demand);
    digitalWrite(pinA, io.direction);
    digitalWrite(pinB, !io.direction);

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
    analogWrite(pinA, 0);
    analogWrite(pinB, 0);
}

float Motor::getMaxVelocity(){
    return Constants::kWheelDiameter * PI * Constants::kMotorsRPM / 60;
}

void Motor::resetEncoder(){
    io.ticks = 0;
    io.last_ticks = 0;
    io.delta_ticks = 0;
    io.speed = 0;
}

long Motor::getTicks(){
    return io.ticks;
}

float Motor::getSpeed(){
    return io.speed;
}
#include "Motor.h"

Motor::Motor(){
    this->pinA = 0;
    this->pinB = 0;
    this->encoder = 0;
}

void Motor::init(int pinA, int pinB, int encoder){
    this->pinA = pinA;
    this->pinB = pinB;
    this->encoder = encoder;

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
    if (io.direction){
        analogWrite(pinA, io.demand);
        analogWrite(pinB, 0);
    } else {
        analogWrite(pinA, 0);
        analogWrite(pinB, io.demand);
    }

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

    //Serial.println("target speed: " + String(speed) + " m/s");
    io.direction = speed > 0;
    
    //Serial.print("PID from motor "); Serial.print(pinA); Serial.print(" ");
    float current_speed = pidController.calculate(abs(speed), io.speed, io.delta_time);
    Serial.print("current speed: "); Serial.println(current_speed);
    float pwm = current_speed / getMaxVelocity() * 255;
    setPWM(pwm);
}

void Motor::setSpeed(float speed, unsigned long current_time){
    if( abs(speed) < 0.05 ){
        stop();
        return;
    }

    //Serial.println("target speed: " + String(speed) + " m/s");
    io.direction = speed > 0;
    //Serial.print("PID from motor "); Serial.print(pinA); Serial.print(" ");
    float current_speed = pidController.calculate(abs(speed), abs(io.speed), (current_time - io.pid_last_time));
    float pwm = current_speed / getMaxVelocity() * 255;
    setPWM(pwm);
    io.pid_last_time = current_time;
}

// Set the speed of the motor in PWM
void Motor::setPWM(int pwm){
    // io.direction = pwm > 0;
    io.demand = min( max( abs(pwm), Constants::kMotorMinPWM), 255);
}

// Get the motor current PWM
int Motor::getPWM(){
    return io.demand;
}

void Motor::stop(){
    io.demand = 0;
    analogWrite(pinA, 0);
    analogWrite(pinB, 0);
}

void Motor::hardStop(){
    io.demand = 0;
    analogWrite(pinA, 255);
    analogWrite(pinB, 255);
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

float Motor::getTargetSpeed(){
    return io.target_speed;
}

void Motor::setVerbose(bool verbose){
    this->verbose = verbose;
}
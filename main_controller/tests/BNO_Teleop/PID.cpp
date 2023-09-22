#include "PID.h"

PID::PID(){
}

void PID::set(float kP, float kI, float kD, float kImax, float out_min, float out_max){
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    this->kImax = kImax;
    this->out_min = out_min;
    this->out_max = out_max;
}

float PID::calculate(float setpoint, float input, float dt){
    float error = setpoint - input;
    integral += error*dt;
    if (integral > kImax)
        integral = kImax;
    else if (integral < -kImax)
        integral = -kImax;
    float derivative = (error - last_error)/dt;
    float output = kP*error + kI*integral + kD*derivative;
    // Serial.print("P: "); Serial.print(kP*error); Serial.print(" I: "); Serial.print(kI*integral); Serial.print(" D: "); Serial.println(kD*derivative);
    last_error = error;
    if (output > out_max)
        output = out_max;
    else if (output < out_min)
        output = out_min;
    return output;
}
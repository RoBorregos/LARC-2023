#include "PID.h"

PID::PID(){

}

float PID::calculate(float setpoint, float input, float dt){
    float error = setpoint - input;
    integral += error*dt;
    float derivative = (error - last_error)/dt;
    float output = kP*error + kI*integral + kD*derivative;
    last_error = error;
    return output;
}
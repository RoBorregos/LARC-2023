#include "PID.h"

PID::PID(){

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
    if (output < 0)
        output = 0;
    return output;
}
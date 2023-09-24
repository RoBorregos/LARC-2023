#ifndef PID_h
#define PID_h

#include "Arduino.h"
#include "Constants.h"

class PID{
    private:
        float last_error = 0;
        float integral = 0;
        float kP = Constants::kMotorKP;
        float kI = Constants::kMotorKI;
        float kD = Constants::kMotorKD;
        float kImax = Constants::kMotorKImax;
    public:
        PID();
        float calculate(float setpoint, float input, float dt);
};


#endif
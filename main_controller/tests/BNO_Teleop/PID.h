#ifndef PID_h
#define PID_h

#include "Arduino.h"
#include "Constants.h"

class PID{
    private:
        float last_error = 0;
        float integral = 0;
        float kP;
        float kI;
        float kD;
        float kImax;
        float out_max;
        float out_min;
    public:
        PID();
        void set(float kP, float kI, float kD, float kImax, float out_min, float out_max);
        float calculate(float setpoint, float input, float dt);
};


#endif
#ifndef Motor_h
#define Motor_h

#include "Constants.h"

class Motor{
    private:
        //A(fwd) and B(rev) pins
        int motorA;
        int motorB;
        int encoder;
    public:
        Motor(int motorA, int motorB, int encoder);
        void setSpeed(float speed);
        void setPWM(int pwm);
        void stop();
        float getMaxVelocity();
};

#endif
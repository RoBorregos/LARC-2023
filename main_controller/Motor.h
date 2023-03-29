#ifndef Motor_h
#define Motor_h

#include "Constants.h"

class Motor{
    private:
        //A(fwd) and B(rev) pins
        int motorA;
        int motorB;
        int encoder;

        struct periodicIO{
            //INPUT
            volatile long ticks = 0;
            long last_ticks = 0;
            unsigned long last_time = 0;
            float delta_ticks = 0;
            float speed = 0;
            //OUTPUT
            float demand = 0;
            bool direction = 1; //1 = forward, 0 = reverse
        };
        periodicIO io;
        
    public:
        Motor();
        void init(int motorA, int motorB, int encoder);
        void setSpeed(float speed);
        void setPWM(int pwm);
        void stop();
        void periodicIO();
        float getMaxVelocity();
        void encoderInterrupt();
        long getTicks();
        float getSpeed();
};

#endif
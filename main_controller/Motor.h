#ifndef Motor_h
#define Motor_h

#include "Arduino.h"
#include "Constants.h"
#include "PID.h"

class Motor{
    private:
        //A(fwd) and B(rev) pins
        int pinPWM;
        int pinA;
        int pinB;
        int encoder;
        PID pidController;

        struct periodicIO{
            //INPUT
            volatile long ticks = 0;
            long last_ticks = 0;
            unsigned long last_time = 0;
            float delta_ticks = 0;
            float delta_time = 0;
            float speed = 0;
            //OUTPUT
            float target_speed = 0;
            float demand = 0;
            bool direction = 1; //1 = forward, 0 = reverse
        };
        periodicIO io;
        
    public:
        Motor();
        void init(int pinPWM, int pinA, int pinB, int encoder);
        void setSpeed(float speed);
        void setPWM(int pwm);
        void stop();
        void periodicIO(unsigned long current_time);
        float getMaxVelocity();
        void encoderInterrupt();
        void resetEncoder();
        long getTicks();
        float getSpeed();
};

#endif
#ifndef Motor_h
#define Motor_h

#include "Arduino.h"
#include "Constants.h"
#include "PID.h"

class Motor{
    private:
        //A(fwd) and B(rev) pins
        int pinA;
        int pinB;
        int encoder;
        PID pidController;
        bool verbose = false;

        struct periodicIO{
            //INPUT
            volatile long ticks = 0;
            long last_ticks = 0;
            unsigned long last_time = 0;
            unsigned long pid_last_time = 0;
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
        void init(int pinA, int pinB, int encoder);
        void setSpeed(float speed);
        void setSpeed(float speed, unsigned long current_time);
        void setPWM(int pwm);
        int getPWM();
        void stop();
        void hardStop();
        void periodicIO(unsigned long current_time);
        float getMaxVelocity();
        float getTargetSpeed();
        void encoderInterrupt();
        void resetEncoder();
        long getTicks();
        float getSpeed();
        void setVerbose(bool verbose);
};

#endif
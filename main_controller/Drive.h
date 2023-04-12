#ifndef Drive_h
#define Drive_h

#include "Arduino.h"
#include "Constants.h"
#include "Motor.h"

enum MotorID{
    FrontLeft,
    FrontRight,
    BackLeft,
    BackRight
};

class Drive{
    private:
        Motor frontLeft;
        Motor frontRight;
        Motor backLeft;
        Motor backRight;
        float angle;
        float* imu_ptr;
    public:
        void init(float* theta);
        void setSpeed(float linearX, float linearY, float angularZ);
        void stop();
        void periodicIO();
        void encoderInterrupt(MotorID motorID);
        float getSpeed(MotorID motorID);
        long getTicks(MotorID motorID);
};

#endif
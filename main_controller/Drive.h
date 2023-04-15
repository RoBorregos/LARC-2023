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

struct Pose2d{
    float x = 0;
    float y = 0;
    float theta = 0;
};

class Drive{
    private:
        Motor frontLeft;
        Motor frontRight;
        Motor backLeft;
        Motor backRight;
        Pose2d velocity;
        Pose2d position;
        float angle;
        float* imu_ptr;
        unsigned long last_time = 0;
    public:
        void init(float* theta);
        void setSpeed(float linearX, float linearY, float angularZ);
        void stop();
        void periodicIO(unsigned long current_time);
        void encoderInterrupt(MotorID motorID);
        float getSpeed(MotorID motorID);
        long getTicks(MotorID motorID);
        Pose2d getChassisSpeeds();
        Pose2d getPosition();
};

#endif
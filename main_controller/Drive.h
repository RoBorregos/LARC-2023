/*
https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
*/

#ifndef Drive_h
#define Drive_h

#include "Arduino.h"
#include "Constants.h"
#include "Motor.h"
#include "LineSensor.h"

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
        constexpr static float loop_time = 10;
        Motor frontLeft;
        Motor frontRight;
        Motor backLeft;
        Motor backRight;
        Pose2d velocity;
        Pose2d position;
        float angle;
        float setpoint;
        float global_setpoint = 0;
        float error;
        float last_error;
        unsigned long last_time = 0;
        bool spin_flag = false;
        bool line_move = false;
        int line_state = 0;
        LineSensor *lineSensor;
    public:
        void init(LineSensor *lineSensor);
        void setSpeed(float linearX, float linearY, float angularZ);
        void setAngle(float angle);
        void setGlobalSetpoint();
        void stop();
        void periodicIO(unsigned long current_time);
        void encoderInterrupt(MotorID motorID);
        float getSpeed(MotorID motorID);
        long getTicks(MotorID motorID);
        Pose2d getChassisSpeeds();
        Pose2d getPosition();
        void resetOdometry();
};

#endif
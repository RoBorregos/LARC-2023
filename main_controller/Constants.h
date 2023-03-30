#ifndef Constants_h
#define Constants_h

#include <Arduino.h>

class Constants{
    private:
    /* data */
    public:
        // Drive constants //
        static constexpr float kWheelDiameter = 0.054; //meters
        static constexpr float kWheelTrack = 0.23; //meters
        static constexpr float kWheelBase = 0.155; //meters
        
        //Motors (A: fwd, B: rev)
        static constexpr float kMotorsRPM = 380; //RPM
        static constexpr float kMotorMinPWM = 140; //PWM (0-255)
        static constexpr int kFrontLeftA = 3;
        static constexpr int kFrontLeftB = 2;
        static constexpr int kFrontRightA = 4;
        static constexpr int kFrontRightB = 5;
        static constexpr int kBackLeftA = 6;
        static constexpr int kBackLeftB = 7;
        static constexpr int kBackRightA = 0;
        static constexpr int kBackRightB = 1;

        //Encoders
        static constexpr float kEncoderTicksPerRevolution = 979.62;
        static constexpr int kFrontLeftEncoder = 12;
        static constexpr int kFrontRightEncoder = 23;
        static constexpr int kBackLeftEncoder = 26;
        static constexpr int kBackRightEncoder = 27;

        //PID
        static constexpr float kP = 5;
        static constexpr float kI = 0.0;
        static constexpr float kD = 0.0;

        // Line sensor //
        static constexpr int kLineSensorS0 = 18;
        static constexpr int kLineSensorS1 = 19;
        static constexpr int kLineSensorS2 = 20;
        static constexpr int kLineSensorS3 = 21;
        static constexpr int kLineSensorSignal = 22;

        // Intake //
        static constexpr int kIntakeMotor1A = 8;
        static constexpr int kIntakeMotor1B = 9;
        static constexpr int kIntakeMotor2A = 10;
        static constexpr int kIntakeMotor2B = 11;
        static constexpr int kIntakePickSpeed = 255;
        static constexpr int kIntakeDropSpeed = -255;
};

#endif
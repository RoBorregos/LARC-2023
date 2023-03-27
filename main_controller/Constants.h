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
        static constexpr float kMotorMinPWM = 150; //PWM (0-255)
        static constexpr int kFrontLeftA = 2;
        static constexpr int kFrontLeftB = 3;
        static constexpr int kFrontRightA = 7;
        static constexpr int kFrontRightB = 6;
        static constexpr int kBackLeftA = 5;
        static constexpr int kBackLeftB = 4;
        static constexpr int kBackRightA = 0;
        static constexpr int kBackRightB = 1;

        //Encoders
        static constexpr float kEncoderTicksPerRevolution = 48;
        static constexpr int kFrontLeftEncoder = 12;
        static constexpr int kFrontRightEncoder = 26;
        static constexpr int kBackLeftEncoder = 27;
        static constexpr int kBackRightEncoder = 23;

        // Line sensor //
        static constexpr int kLineSensorS0 = 18;
        static constexpr int kLineSensorS1 = 19;
        static constexpr int kLineSensorS2 = 20;
        static constexpr int kLineSensorS3 = 21;
        static constexpr int kLineSensorSignal = 22;
};

#endif
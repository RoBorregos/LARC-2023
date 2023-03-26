#ifndef Constants_h
#define Constants_h

#include <Arduino.h>

class Constants{
    private:
    /* data */
    public:
        // Drive constants //
        static const float kWheelDiameter = 0.054; //meters
        static const float kWheelTrack = 0.23; //meters
        static const float kWheelBase = 0.155; //meters
        
        //Motors (A: fwd, B: rev)
        static const float kMotorsRPM = 380; //RPM
        static const int kFrontLeftA = 2;
        static const int kFrontLeftB = 3;
        static const int kFrontRightA = 6;
        static const int kFrontRightB = 7;
        static const int kBackLeftA = 5;
        static const int kBackLeftB = 4;
        static const int kBackRightA = 1;
        static const int kBackRightB = 0;

        //Encoders
        static const float kEncoderTicksPerRevolution = 48;
        static const int kFrontLeftEncoder = 12;
        static const int kFrontRightEncoder = 26;
        static const int kBackLeftEncoder = 27;
        static const int kBackRightEncoder = 23;

        // Line sensor //
        static const int kLineSensorS0 = 18;
        static const int kLineSensorS1 = 19;
        static const int kLineSensorS2 = 20;
        static const int kLineSensorS3 = 21;
        static const int kLineSensorSignal = 22;
};

#endif
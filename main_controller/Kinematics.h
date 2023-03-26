/*Mecanum Drive kinematics*/

#ifndef Kinematics_h
#define Kinematics_h

#include <Arduino.h>
#include <iostream>

class Kinematics{
    private:
        float inverseKinematics[4][3];
        float input[3][1];
        float output[4][1];
    public:
        struct WheelSpeeds{
            float frontLeft;
            float frontRight;
            float backLeft;
            float backRight;
        };
        Kinematics(float wheelTrack, float wheelBase);
        WheelSpeeds getWheelSpeeds(float linearX, float linearY, float angularZ);
//        float[][] matrixMult(float[][] matrixA, float[][] matrixB)
};

#endif
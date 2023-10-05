#ifndef LineSensor_h
#define LineSensor_h

#include <Arduino.h>
#include "Constants.h"

enum SensorID{
    FrontLeft1 = 0,
    FrontLeft2 = 1,
    FrontRight1 = 2,
    FrontRight2 = 3,
    RightFront1 = 8,
    RightFront2 = 9,
    RightBack1 = 10,
    RightBack2 = 11,
    BackRight1 = 4,
    BackRight2 = 5,
    BackLeft1 = 6,
    BackLeft2 = 7,
    LeftBack1 = 12,
    LeftBack2 = 13,
    LeftFront1 = 14,
    LeftFront2 = 15
};

class LineSensor{
    private:
    /* data */
        int averages[16] = {0};
    public:
        LineSensor();
        void calibrate();
        bool lineDetected(SensorID sensor);
        bool lineDetected(int sensor);
        int getData();
};

#endif
#ifndef LineSensor_h
#define LineSensor_h

#include <Arduino.h>
#include "Constants.h"

enum SensorID{
    FrontLeft1 = 8,
    FrontLeft2 = 9,
    FrontRight1 = 0,
    FrontRight2 = 1,
    RightFront1 = 4,
    RightFront2 = 5,
    RightBack1 = 10,
    RightBack2 = 11,
    BackRight1 = 6,
    BackRight2 = 7,
    BackLeft1 = 2,
    BackLeft2 = 3,
    LeftBack1 = 12,
    LeftBack2 = 13,
    LeftFront1 = 14,
    LeftFront2 = 15
};

class LineSensor{
    private:

    public:
        LineSensor();
        bool lineDetected(SensorID sensor);
        int getData();
};

#endif
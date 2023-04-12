#ifndef LineSensor_h
#define LineSensor_h

#include <Arduino.h>
#include "Constants.h"

enum SensorID{
    FrontLeft1 = 0,
    FrontLeft2 = 1,
    FrontRight1 = 2,
    FrontRight2 = 3,
    RightFront1 = 4,
    RightFront2 = 5,
    RightBack1 = 6,
    RightBack2 = 7,
    BackRight1 = 8,
    BackRight2 = 9,
    BackLeft1 = 10,
    BackLeft2 = 11,
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
};

#endif
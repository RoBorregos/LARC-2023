#ifndef Drive_h
#define Drive_h

#include "Motor.h"
#include "Constants.h"
#include "Kinematics.h"

class Drive{
    private:
        Motor frontLeft;
        Motor frontRight;
        Motor backLeft;
        Motor backRight;
        Kinematics kinematics;
        Kinematics::WheelSpeeds wheelSpeeds;
    public:
        Drive();
        void setSpeed(float linearX, float linearY, float angularZ);
        void stop();
};

#endif
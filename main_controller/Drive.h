#ifndef Drive_h
#define Drive_h

#include "Constants.h"
#include "Motor.h"

class Drive{
    private:
        Motor frontLeft;
        Motor frontRight;
        Motor backLeft;
        Motor backRight;
    public:
        void init();
        void setSpeed(float linearX, float linearY, float angularZ);
        void periodicIO();
};

#endif
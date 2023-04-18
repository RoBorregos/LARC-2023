#ifndef RosBridge_h
#define RosBridge_h

#include "Arduino.h"
#include "Drive.h"

class RosBridge{
    private:
        Drive *_drive;
    public:
        void init(Drive *drive);
        void spin();
};

#endif
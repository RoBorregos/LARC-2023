#ifndef Intake_h
#define Intake_h

#include "Arduino.h"
#include "Constants.h"

class Intake{
    private:
        bool presence = false;
        bool flag = false;
        unsigned long presence_detection_time = 0;
    public:
        Intake();
        void pick();
        void in();
        void out(unsigned long current_time);
        void drop();
        void stop();
        bool getPresence();
        void periodicIO(unsigned long current_time);
};

#endif
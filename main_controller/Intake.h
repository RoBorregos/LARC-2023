#ifndef Intake_h
#define Intake_h

#include "Arduino.h"
#include "Constants.h"

enum IntakeActions{
    In,
    Out,
    Pick,
    Drop,
    Stop
};

class Intake{
    private:
        constexpr static float loop_time = 10;
        bool presence = false;
        bool flag = false;
        unsigned long presence_detection_time = 0;
        unsigned long last_time = 0;
        IntakeActions action = Stop;
    public:
        Intake();
        void pick();
        void in();
        void out(unsigned long current_time);
        void drop();
        void stop();
        void setAction(IntakeActions action);
        bool getPresence();
        void periodicIO(unsigned long current_time);
};

#endif
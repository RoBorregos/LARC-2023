#ifndef Intake_h
#define Intake_h

#include "Arduino.h"
#include "Constants.h"

class Intake{
    private:

    public:
        Intake();
        void pick();
        void drop();
        void stop();
};

#endif
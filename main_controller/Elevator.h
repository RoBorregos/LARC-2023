#ifndef Elevator_h
#define Elevator_h

#include "Arduino.h"
#include "Constants.h"

enum ElevatorPosition{
    Pick = 0,
    FirstWarehouse = 10000,
    SecondWarehouse = 22500,
    ThirdWarehouse = 24000,
    FirstShelf = 5000,
    SecondShelf = 17500,
    ThirdShelf = 30000,
};

class Elevator{
    private:
        struct Stepper{
            int directionPin = 0;
            int stepPin = 0;
            int direction = 0;
            unsigned long step_delay = 0;
            unsigned long last_step_time = 0;
            int number_of_steps = 0;
            int step = 0;
            int steps_left = 0;

        };
        Stepper mStepper;
        unsigned long current_time = 0;
        int current_position = 0;
    public:
        Elevator();
        void setSpeed(long speed);
        void setSteps(int step);
        void periodicIO();
        void setPosition(ElevatorPosition position);
        bool positionReached();
};
#endif
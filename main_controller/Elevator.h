#ifndef Elevator_h
#define Elevator_h

#include "Arduino.h"
#include "Constants.h"

enum ElevatorPosition{
    PickPos = 0,
    FirstIn = 19000,
    SecondIn = 42500,
    ThirdIn = 65500,
    FirstOut = 5000,
    SecondOut = 17500,
    ThirdOut = 30000,
    FirstShelf = 10000,
    SecondShelf = 22500,
    ThirdShelf = 35000
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
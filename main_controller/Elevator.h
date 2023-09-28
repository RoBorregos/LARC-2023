#ifndef Elevator_h
#define Elevator_h

#include "Arduino.h"
#include "Constants.h"
#include <Stepper.h>

enum ElevatorPosition{
    PickPos = 0,
    FirstIn = 21000,
    SecondIn = 44500,
    ThirdIn = 67500,
    FirstOut = 20000,
    SecondOut = 43500,
    ThirdOut = 66500,
    FirstShelf = 5000,
    SecondShelf = 41500,
    ThirdShelf = 80000
};

class Elevator{
    private:
        struct StepperC{
            int directionPin = 0;
            int stepPin = 0;
            int direction = 0;
            unsigned long step_delay = 0;
            unsigned long last_step_time = 0;
            long int number_of_steps = 0;
            long int step = 0;
            long int steps_left = 0;
        };
        StepperC mStepper;
        Stepper* stepperPtr;
        unsigned long current_time = 0;
        long int current_position = 0;
        int steps_queued = 0;
    public:
        Elevator();
        void init(Stepper *stepper);
        void setSpeed(long speed);
        void setSteps(long int step);
        void periodicIO();
        void setPosition(ElevatorPosition position);
        bool positionReached();
};
#endif
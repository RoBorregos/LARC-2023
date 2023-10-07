#ifndef Elevator_h
#define Elevator_h

#include "Arduino.h"
#include "Constants.h"
#include "Adafruit_VL53L0X.h"
#include <Stepper.h>

enum ElevatorPosition{
    PickPos = Constants::kElevatorLevel0InSteps,
    FirstIn = Constants::kElevatorLevel1InSteps,
    SecondIn = Constants::kElevatorLevel2InSteps,
    ThirdIn = Constants::kElevatorLevel3InSteps,
    FirstOut = Constants::kElevatorLevel1OutSteps,
    SecondOut = Constants::kElevatorLevel2OutSteps,
    ThirdOut = Constants::kElevatorLevel3OutSteps,
    FirstShelf = Constants::kElevatorShelf1Steps,
    SecondShelf = Constants::kElevatorShelf2Steps,
    ThirdShelf = Constants::kElevatorShelf3Steps
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
        uint8_t goal_level = 0;
        uint8_t curr_level = 200;
        bool goal_reached = true;
        int steps_queued = 0;
        Adafruit_VL53L0X *vlx;
        const uint16_t kElevatorStepsPerMM = Constants::kElevatorStepsPerMM;
        const uint16_t kElevatorStepsVLX = Constants::kElevatorStepsVLX;
        const uint16_t kElevatorLevels[4] = {Constants::kElevatorLevel0Height, Constants::kElevatorLevel1Height, Constants::kElevatorLevel2Height, Constants::kElevatorLevel3Height};
        const int kElevatorSmallStepDown = Constants::kElevatorLevelSmallStepDown;
        const uint8_t kElevatorTolerance = Constants::kElevatorTolerance;
        int current_steps = 0;
    public:
        Elevator();
        void init(Stepper *stepper, Adafruit_VL53L0X *vlx);
        void setSpeed(long speed);
        void setSteps(long int step);
        void periodicIO();
        void setPosition(ElevatorPosition position);
        bool positionReached();
        void setLevel(uint8_t goal_level);
        void setSteps(ElevatorPosition position);
        void smallStepDown();
        uint16_t readVLX();
};
#endif
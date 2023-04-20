#include "Elevator.h"
#include "Arduino.h"

Elevator::Elevator(){
    mStepper.directionPin = Constants::kStepperDirectionPin;
    mStepper.stepPin = Constants::kStepperStepPin;
    mStepper.direction = 0;
    mStepper.number_of_steps = Constants::kStepperSteps;
    mStepper.step = 0;
    mStepper.step_delay = 60L * 1000L * 1000L / mStepper.number_of_steps / Constants::kStepperSpeed;

    pinMode(mStepper.directionPin, OUTPUT);
    pinMode(mStepper.stepPin, OUTPUT);
}

void Elevator::setSpeed(long speed){
    mStepper.step_delay = 60L * 1000L * 1000L / mStepper.number_of_steps / speed;
}

void Elevator::setSteps(int step){
    mStepper.steps_left = abs(step);
    mStepper.direction = step > 0 ? 1 : 0;
    periodicIO();
}

void Elevator::setPosition(ElevatorPosition position){
    setSteps(-position - current_position);
}

bool Elevator::positionReached(){
    return mStepper.steps_left == 0;
}

void Elevator::periodicIO(){
    while(mStepper.steps_left > 0){
        current_time = micros();
        if( current_time - mStepper.last_step_time > mStepper.step_delay){
            mStepper.last_step_time = current_time;
            if(mStepper.direction == 1){
                mStepper.step++;
                current_position++;
                if(mStepper.step == mStepper.number_of_steps){
                    mStepper.step = 0;
                }
            }else{
                if(mStepper.step == 0){
                    mStepper.step = mStepper.number_of_steps;
                }
                mStepper.step--;
                current_position--;
            }
            mStepper.steps_left--;

            int action = mStepper.step % 4;
            switch( action ){
                case 0: //01
                    digitalWrite(mStepper.directionPin, LOW);
                    digitalWrite(mStepper.stepPin, HIGH);
                    break;
                case 1: //11
                    digitalWrite(mStepper.directionPin, HIGH);
                    digitalWrite(mStepper.stepPin, HIGH);
                    break;
                case 2: //10
                    digitalWrite(mStepper.directionPin, HIGH);
                    digitalWrite(mStepper.stepPin, LOW);
                    break;
                case 3: //00
                    digitalWrite(mStepper.directionPin, LOW);
                    digitalWrite(mStepper.stepPin, LOW);
                    break;
            }
        }
    }
}

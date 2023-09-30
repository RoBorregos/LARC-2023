#include "Elevator.h"
#include "Arduino.h"
#include "Constants.h"
#include "Adafruit_VL53L0X.h"

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

void Elevator::init(Stepper *stepper, Adafruit_VL53L0X *vlx){
    this->vlx = vlx;
    stepperPtr = stepper;
    stepperPtr->setSpeed(Constants::kStepperSpeed);
}

void Elevator::setSpeed(long speed){
    mStepper.step_delay = 60L * 1000L * 1000L / mStepper.number_of_steps / speed;
}

void Elevator::setSteps(long int step){
    long int temp_step = abs(step);
    steps_queued = int(temp_step / 32000);
    mStepper.steps_left = temp_step % 32000;
    mStepper.direction = step > 0 ? 1 : 0;
}

void Elevator::setPosition(ElevatorPosition position){
    //Serial.println(-position - current_position);
    setSteps(-position - current_position);
}

bool Elevator::positionReached(){
    return mStepper.steps_left == 0;
}

// function to read and filter the value from the vlx
uint16_t Elevator::readVLX(){
    uint16_t range = vlx->readRange();
    while (range > 1000){
        range = vlx->readRange();
    }
    return range;
}

void Elevator::setLevel(uint8_t goal_level){

    if (goal_level != curr_level){
        uint16_t curr_height = readVLX();
        uint16_t goal_height = kElevatorLevels[goal_level];
        
        int steps = -(goal_height - curr_height) * kElevatorStepsPerMM;
        stepperPtr->step(steps);

        //curr_height = readVLX();
        // correct with vlx, not accurate
        /*while (abs(goal_height - curr_height) > kElevatorTolerance){
            if (goal_height > curr_height){
                stepperPtr->step(-steps);
            }
            else{
                stepperPtr->step(steps);
            }
            curr_height = readVLX();
        }*/

        // send that the elevator has finished movement
        // -----inprogress
        curr_level = goal_level;
    }
}

void Elevator::setSteps(ElevatorPosition steps){
    int required_steps = steps - current_steps;
    //Serial.print("Required steps: "); Serial.println(required_steps);
    stepperPtr->step(-required_steps);
    current_steps += required_steps;
}

void Elevator::periodicIO(){
    

    /*
    if (!goal_reached){
        if (goal_level == curr_level){
            goal_reached = true;
        }
        uint16_t curr_height = readVLX();
        uint16_t goal_height = kElevatorLevels[goal_level];
        
        int steps = -(goal_height - curr_height) * kElevatorStepsPerMM;
        stepperPtr->step(steps);

        curr_height = readVLX();
        // correct with vlx, not accurate
        while (abs(goal_height - curr_height) > kElevatorTolerance){
            if (goal_height > curr_height){
                stepperPtr->step(-steps);
            }
            else{
                stepperPtr->step(steps);
            }
            curr_height = readVLX();
        }

        // send that the elevator has finished movement
        // -----inprogress
        curr_level = goal_level;
        goal_reached = true;
    }*/
    /*while( mStepper.steps_left > 0 ){
        stepperPtr->step( (mStepper.direction? 1:-1) * mStepper.steps_left);
        current_position += (mStepper.direction? 1:-1) * mStepper.steps_left;
        mStepper.steps_left = 0;
        if( steps_queued > 0 ){
            mStepper.steps_left = 32000;
            steps_queued--;
        }
    }*/
    /*while(mStepper.steps_left > 0){
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
        } else {
            yield();
        }
    }
    if( steps_queued > 0 ){
        mStepper.steps_left = 32000;
        steps_queued--;
        delay(1000);
    }*/
}

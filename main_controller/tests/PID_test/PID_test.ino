#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Stepper.h>

#include "Constants.h"
#include "Drive.h"
#include "Motor.h"
#include "Plot.h"


Drive mDrive;
// create pointer to the drive object
Drive *drive = &mDrive;
Plot mPlot(drive);

bool ENABLE_ROS = true;

unsigned long debug_time = 0;
int state = -1;
unsigned long state_time = 0;
unsigned long loop_time = 0;
unsigned long current_time = 0;

void setup(){
    current_time = millis();

    mDrive.init();
    Serial.begin(115200);

    //Serial.write("<target>");
    attachInterrupt(digitalPinToInterrupt(Constants::kFrontLeftEncoder), interruptFL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Constants::kFrontRightEncoder), interruptFR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Constants::kBackLeftEncoder), interruptBL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Constants::kBackRightEncoder), interruptBR, CHANGE);

    state_time = current_time;
    loop_time = current_time;
    debug_time = current_time;

    delay(3000);
    mPlot.startSequence();

    //mIntake.setAction(IntakeActions::Pick);
    //mElevator.setPosition(ElevatorPosition::FirstIn);
    //mWarehouse.cubeOut(LevelPosition::Upper, current_time);
}

bool FRONT_TEST = false;
bool DRIVE_TEST = true; //move forward, right, backward, left
long unsigned int DRIVE_TEST_MOVEMENT_TIME = 1500;
int STATE = 0;

bool logged1 = false;
bool logged2 = false;
bool logged3 = false;
bool logged4 = false;
bool logged5 = false;

float SPEED = 0.5;

void loop(){

    if (FRONT_TEST){
        float speed = SPEED;
        current_time = millis();
        mDrive.periodicIO(current_time);
        mDrive.setSpeed(-speed, 0, 0, current_time);
        mPlot.plotTargetandCurrent(speed);
    }

    else if (DRIVE_TEST){
        float xspeed = 0, yspeed = 0;
        // ADD STOP TIMES IN BWTEEN EACH
        if (STATE == 0){
            xspeed = SPEED;
            yspeed = 0;
        }
        else if (STATE == 1){
            xspeed = 0;
            yspeed = 0;
        }
        else if (STATE == 2){
            xspeed = 0;
            yspeed = SPEED;
        }
        else if (STATE == 3){
            xspeed = 0;
            yspeed = 0;
        }
        else if (STATE == 4){
            xpeed = -SPEED;
            yspeed = 0;
        }
        else if (STATE == 5){
            xspeed = 0;
            yspeed = 0;
        }
        else if (STATE == 6){
            xspeed = 0;
            yspeed = -SPEED;
        }
        else if (STATE == 7){
            xspeed = 0;
            yspeed = 0;
        }
        else {
            xspeed = 0;
            yspeed = 0;
        }
        
        current_time = millis();
        mDrive.periodicIO(current_time);
        mDrive.setSpeed(xspeed, yspeed, 0, current_time);
        //mPlot.plotTargetandCurrent(xspeed);

        if (current_time - state_time > DRIVE_TEST_MOVEMENT_TIME){
            state_time = current_time;
            STATE++;
            if (STATE > 7){
                STATE = 0;
            }
            if (STATE == 0){
                Serial.println("Moving forward");
            }
            else if (STATE == 2){
                Serial.println("Moving right");
            }
            else if (STATE == 4){
                Serial.println("Moving backward");
            }
            else if (STATE == 6){
                Serial.println("Moving left");
            }
            else if (STATE == 8){
                Serial.println("Stopping");
            }
        }
        
    }

}
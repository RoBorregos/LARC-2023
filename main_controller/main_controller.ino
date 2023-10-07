#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_BNO055.h>
#include <Stepper.h>

#include "Constants.h"
#include "RosBridge.h"
#include "Drive.h"
#include "Motor.h"
#include "Intake.h"
#include "Elevator.h"
#include "LineSensor.h"
#include "Warehouse.h"
#include "VLX.h"


Drive mDrive;
Elevator mElevator;
Intake mIntake;
Warehouse mWarehouse;
LineSensor mLineSensor;
RosBridge ros;
VLX vlxs;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire2);
BNO mbno = BNO(&bno);

bool ENABLE_ROS = true;

unsigned long debug_time = 0;
int state = -1;
unsigned long state_time = 0;
unsigned long loop_time = 0;

Adafruit_VL53L0X vlx[4];
Stepper mStepper(Constants::kStepperSteps, Constants::kStepperDirectionPin, Constants::kStepperStepPin);

unsigned long current_time = 0;

void setup(){
    current_time = millis();

    Wire.begin();
    Serial.begin(115200);
    vlxs.initSensors(&vlx[0], &vlx[1], &vlx[2], &vlx[3]);
    mDrive.init(&mbno, &mLineSensor);
    mElevator.init(&mStepper, &vlx[3]);
    mWarehouse.init(current_time, &vlx[0], &vlx[1], &vlx[2]);

    //Serial.write("<target>");
    attachInterrupt(digitalPinToInterrupt(Constants::kFrontLeftEncoder), interruptFL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Constants::kFrontRightEncoder), interruptFR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Constants::kBackLeftEncoder), interruptBL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Constants::kBackRightEncoder), interruptBR, CHANGE);

    state_time = current_time;
    loop_time = current_time;
    debug_time = current_time;
    if( ENABLE_ROS )
        ros.init(&mDrive, &mIntake, &mElevator, &mWarehouse, &mLineSensor, &vlxs);

    //mIntake.setAction(IntakeActions::Pick);
    //mElevator.setPosition(ElevatorPosition::FirstIn);
    //mWarehouse.cubeOut(LevelPosition::Upper, current_time);
}

void loop(){

    current_time = millis();
    if( ENABLE_ROS )
        ros.spin(current_time);
    mDrive.periodicIO(current_time);
    mElevator.periodicIO();
    mIntake.periodicIO(current_time);
    mWarehouse.periodicIO(current_time);

}
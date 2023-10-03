#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_BNO055.h>
#include <Stepper.h>

#include "Constants.h"
#include "Drive.h"
#include "Motor.h"
#include "Plot.h"
#include "BNO.h"
#include "VLX.h"

#include <Stepper.h> 

#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X vlx[4];
VLX vlxs;

#define STEPS 3200

#define INTAKEPRESENCE 12
struct motor{
    int pwmA;
    int pwmB;
};

struct elevatorLevel{
    int level0;
    int level1;
    int level2;
    int level3;
};

// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver
Stepper stepper(STEPS, 30, 31); // Pin 2 connected to DIRECTION & Pin 3 connected to STEP Pin of Driver
// Elevator heights: Pick: 70mm; Level1: 155; Level2: 225; Level3: 295
struct elevatorLevel elevatorLevel = {61, 143, 212, 286};
struct elevatorLevel elevatorSteps = {0, 27000, 49000, 72000};
int current_steps = 0;
int elevatorTolerance = 1;
int STEPS_PER_MM = 315;
int INTAKESPEED = 255;
#define motorInterfaceType 1

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28,&Wire2);
BNO mbno = BNO(&bno);

Drive mDrive;
// create pointer to the drive object
Drive *drive = &mDrive;
Plot mPlot(drive);

struct motor intake1 = {22, 23};
struct motor intake2 = {29, 28};
struct motor level1 = {8, 9};
struct motor level2 = {10, 11};
struct motor level3 = {29, 28};
int WAREHOUSE_SPEED = 255;

bool TELEOP = true;
bool FRONT_TEST = false;
bool DRIVE_TEST = false; //move forward, right, backward, left

long unsigned int DRIVE_TEST_MOVEMENT_TIME = 1500;

float SPEED = 0.35;
long STEPPER_SPEED = 750;
long STEPPER_STEPS = 1;


unsigned long debug_time = 0;
int state = -1;
unsigned long state_time = 0;
unsigned long loop_time = 0;
unsigned long current_time = 0;

//moves the elevator to a step
void moveElevatortoStep(int steps){
    int required_steps = steps - current_steps;
    //Serial.print("Required steps: "); Serial.println(required_steps);
    stepper.step(-required_steps);
    current_steps += required_steps;
}

//moves the elevator to a height
void moveElevatorToHeight_MM(uint height, uint tolerance){
  
  uint current_height = readVLX(3);
  Serial.print("Current height: ");
  Serial.println(current_height);

  int steps = -(height - current_height) * STEPS_PER_MM;
  Serial.println("Starting movement");
  stepper.step(steps);

  current_height = readVLX(3);

  steps = STEPS_PER_MM/2;
  /*while (abs(height - current_height) > tolerance){
    if (height > current_height){
      stepper.step(-steps);
    }
    else{
      stepper.step(steps);
    }
    current_height = readVLX(3);
    Serial.print("Initializing, current height: ");
    Serial.println(current_height);
  }*/
}

// function to read and filter the value from the vlx
uint readVLX(uint sensorID){
    uint range = vlx[sensorID].readRange();
    while (range > 1000){
        range = vlx[sensorID].readRange();
    }
    return range;
}

void initRobot(){
    current_time = millis();
    mDrive.init(&mbno);
    pinMode(INTAKEPRESENCE, INPUT);

    stepper.setSpeed(STEPPER_SPEED);
    vlxs.initSensors(&vlx[0], &vlx[1], &vlx[2], &vlx[3]);

    state_time = current_time;
    loop_time = current_time;
    debug_time = current_time;
}

void setup(){
   
    Serial.begin(115200);
    //Serial.write("<target>");
    attachInterrupt(digitalPinToInterrupt(Constants::kFrontLeftEncoder), interruptFL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Constants::kFrontRightEncoder), interruptFR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Constants::kBackLeftEncoder), interruptBL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Constants::kBackRightEncoder), interruptBR, CHANGE);

    initRobot();
    delay(3000);

    //mIntake.setAction(IntakeActions::Pick);
    //mElevator.setPosition(ElevatorPosition::FirstIn);
    //mWarehouse.cubeOut(LevelPosition::Upper, current_time);
}

int STATE = 0;

bool logged1 = false;
bool logged2 = false;
bool logged3 = false;
bool logged4 = false;
bool logged5 = false;
bool pick = false;
bool store = false;
bool out = false;

bool level1_active = false;
bool level2_active = false;
bool level3_active = false;


char c = 'x';

void loop(){

    // teleop, receives wasd to move, qe to rotate and x to stop
    if (TELEOP){
        float xspeed, yspeed, zspeed;

        if (Serial.available() > 0){
            c = Serial.read();
        }
        // hard stop
        else if (c=='X'){
            //mDrive.hardStop();
            xspeed = 0;
            yspeed = 0;
            zspeed = 0;
        }
        if (c=='w'){
            xspeed = SPEED;
            yspeed = 0;
            zspeed = 0;
        }
        else if (c=='s'){
            xspeed = -SPEED;
            yspeed = 0;
            zspeed = 0;
        }
        else if (c=='a'){
            xspeed = 0;
            yspeed = SPEED;
            zspeed = 0;
        }
        else if (c=='d'){
            xspeed = 0;
            yspeed = -SPEED;
            zspeed = 0;
        }
        else if (c=='q'){
            xspeed = 0;
            yspeed = 0;
            zspeed = -3*SPEED;
        }
        else if (c=='e'){
            xspeed = 0;
            yspeed = 0;
            zspeed = 3*SPEED;
        }
        else if (c=='x'){
            xspeed = 0;
            yspeed = 0;
            zspeed = 0;
        }
        // Intake
        else if (c=='i'){
            pick = true;
            store = false;
            out = false;
        }
        else if (c=='I'){
            pick = false;
            store = true;
            out = false;
        }
        else if (c=='o'){
            pick = false;
            store = false;
            out = true;
        }
        else if (c=='p'){
            pick = false;
            store = false;
            out = false;
        }
        // Stepper, U up, D down, S stop
        else if (c=='U'){
            stepper.step(-STEPS);
        }
        else if (c=='D'){
            stepper.step(STEPS);
        }
        // elevator to levels
        else if (c=='0'){
            moveElevatortoStep(elevatorSteps.level0);
            c = '-';
        }
        else if (c=='1'){
            moveElevatortoStep(elevatorSteps.level1);
            c = '-';
        }
        else if (c=='2'){
            moveElevatortoStep(elevatorSteps.level2);
            c = '-';
        }
        else if (c=='3'){
            moveElevatortoStep(elevatorSteps.level3);
            c = '-';
        }
        // warehouse motors
        else if (c=='z'){
            if (!level1_active){
                analogWrite(level1.pwmA, WAREHOUSE_SPEED);
                analogWrite(level1.pwmB, 0);
                level1_active = true;
            }
            else{
                analogWrite(level1.pwmA, 0);
                analogWrite(level1.pwmB, 0);
                level1_active = false;
            }
        }
        else if (c=='x'){
            if (!level1_active){
                analogWrite(level1.pwmA, 0);
                analogWrite(level1.pwmB, WAREHOUSE_SPEED);
                level2_active = true;
            }
            else{
                analogWrite(level1.pwmA, 0);
                analogWrite(level1.pwmB, 0);
                level2_active = false;
            }
        }
        else if (c=='c'){
            if (!level2_active){
                analogWrite(level2.pwmA, WAREHOUSE_SPEED);
                analogWrite(level2.pwmB, 0);
                level2_active = true;
            }
            else{
                analogWrite(level1.pwmA, 0);
                analogWrite(level1.pwmB, 0);
                level2_active = false;
            }
        }
        else if (c=='v'){
            if (!level2_active){
                analogWrite(level2.pwmA, 0);
                analogWrite(level2.pwmB, WAREHOUSE_SPEED);
                level2_active = true;
            }
            else{
                analogWrite(level2.pwmA, 0);
                analogWrite(level2.pwmB, 0);
                level2_active = false;
            }
        }
        else if (c=='b'){
            if (!level3_active){
                analogWrite(level3.pwmA, WAREHOUSE_SPEED);
                analogWrite(level3.pwmB, 0);
                level3_active = true;
            }
            else{
                analogWrite(level3.pwmA, 0);
                analogWrite(level3.pwmB, 0);
                level3_active = false;
            }
        }
        else if (c=='n'){
            if (!level3_active){
                analogWrite(level3.pwmA, 0);
                analogWrite(level3.pwmB, WAREHOUSE_SPEED);
                level3_active = true;
            }
            else{
                analogWrite(level3.pwmA, 0);
                analogWrite(level3.pwmB, 0);
                level3_active = false;
            }
        }
        else if (c=='/'){
            //restart robot
            xspeed = 0;
            yspeed = 0;
            zspeed = 0;
            mDrive.setSpeedOriented(xspeed, yspeed, zspeed, current_time);
            initRobot();
        }
        // keep current settings
        else if (c=='-'){
            // do nothing
            xspeed = xspeed;
        }
        else {
            xspeed = 0;
            yspeed = 0;
            zspeed = 0;
        }
        if (pick){
            if (!digitalRead(INTAKEPRESENCE)){
                analogWrite(intake1.pwmA, INTAKESPEED);
                analogWrite(intake1.pwmB, 0);
                analogWrite(intake2.pwmA, INTAKESPEED);
                analogWrite(intake2.pwmB, 0);
            }
            else{
                analogWrite(intake1.pwmA, 0);
                analogWrite(intake1.pwmB, 0);
                analogWrite(intake2.pwmA, 0);
                analogWrite(intake2.pwmB, 0);
            }
        }
        else if (store){
            if (digitalRead(INTAKEPRESENCE)){
                analogWrite(intake1.pwmA, INTAKESPEED);
                analogWrite(intake1.pwmB, 0);
                analogWrite(intake2.pwmA, INTAKESPEED);
                analogWrite(intake2.pwmB, 0);
            }
            else{
                analogWrite(intake1.pwmA, 0);
                analogWrite(intake1.pwmB, 0);
                analogWrite(intake2.pwmA, 0);
                analogWrite(intake2.pwmB, 0);
            }
        }
        else if (out){
            analogWrite(intake1.pwmA, 0);
            analogWrite(intake1.pwmB, INTAKESPEED);
            analogWrite(intake2.pwmA, 0);
            analogWrite(intake2.pwmB, INTAKESPEED);
        }
        else{
            analogWrite(intake1.pwmA, 0);
            analogWrite(intake1.pwmB, 0);
            analogWrite(intake2.pwmA, 0);
            analogWrite(intake2.pwmB, 0);
        }
        

        current_time = millis();
        mDrive.periodicIO(current_time);
        mDrive.setSpeedOriented(xspeed, yspeed, zspeed, current_time);
    }

    else if (FRONT_TEST){
        float speed = SPEED;
        current_time = millis();
        mDrive.periodicIO(current_time);
        mDrive.setSpeedOriented(-speed, 0, 0, current_time);
        //mPlot.plotTargetandCurrent(speed);
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
            xspeed = -SPEED;
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
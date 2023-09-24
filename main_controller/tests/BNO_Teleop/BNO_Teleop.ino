#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_BNO055.h>
#include <Stepper.h>

#include "Constants.h"
#include "Drive.h"
#include "Motor.h"
#include "Plot.h"
#include "BNO.h"

#include <Stepper.h> 

#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X vlx[4];

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
struct elevatorLevel elevatorLevel = {61, 145, 215, 290};
int elevatorTolerance = 1;
int STEPS_PER_MM = 315;
int INTAKESPEED = 200;
#define motorInterfaceType 1

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28,&Wire2);
BNO mbno = BNO(&bno);

Drive mDrive;
// create pointer to the drive object
Drive *drive = &mDrive;
Plot mPlot(drive);

struct motor intake1 = {22, 23};
struct motor intake2 = {29, 28};

bool TELEOP = true;
bool FRONT_TEST = false;
bool DRIVE_TEST = false; //move forward, right, backward, left

long unsigned int DRIVE_TEST_MOVEMENT_TIME = 1500;

float SPEED = 0.3;
long STEPPER_SPEED = 500;
long STEPPER_STEPS = 1;


unsigned long debug_time = 0;
int state = -1;
unsigned long state_time = 0;
unsigned long loop_time = 0;
unsigned long current_time = 0;

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
  while (abs(height - current_height) > tolerance){
    if (height > current_height){
      stepper.step(-steps);
    }
    else{
      stepper.step(steps);
    }
    current_height = readVLX(3);
    Serial.print("Initializing, current height: ");
    Serial.println(current_height);
  }
}

// function to read and filter the value from the vlx
uint readVLX(uint sensorID){
    uint range = vlx[sensorID].readRange();
    while (range > 1000){
        range = vlx[sensorID].readRange();
    }
    return range;
}

void setup(){
    current_time = millis();

    mDrive.init(&mbno);
    Serial.begin(115200);
    pinMode(INTAKEPRESENCE, INPUT);

    //Serial.write("<target>");
    attachInterrupt(digitalPinToInterrupt(Constants::kFrontLeftEncoder), interruptFL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Constants::kFrontRightEncoder), interruptFR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Constants::kBackLeftEncoder), interruptBL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Constants::kBackRightEncoder), interruptBR, CHANGE);

    stepper.setSpeed(STEPPER_SPEED);
    vlxSetup();

    state_time = current_time;
    loop_time = current_time;
    debug_time = current_time;

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
            // only if intake is not occupied
            if (digitalRead(INTAKEPRESENCE) == LOW){
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
        else if (c=='I'){
            // only if intake is not occupied
            if (digitalRead(INTAKEPRESENCE) == HIGH){
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
        else if (c=='o'){
            analogWrite(intake1.pwmA, 0);
            analogWrite(intake1.pwmB, INTAKESPEED);
            analogWrite(intake2.pwmA, 0);
            analogWrite(intake2.pwmB, INTAKESPEED);
        }
        else if (c=='p'){
            analogWrite(intake1.pwmA, 0);
            analogWrite(intake1.pwmB, 0);
            analogWrite(intake2.pwmA, 0);
            analogWrite(intake2.pwmB, 0);
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
            moveElevatorToHeight_MM(elevatorLevel.level0, elevatorTolerance);
            c = '-';
        }
        else if (c=='1'){
            moveElevatorToHeight_MM(elevatorLevel.level1, elevatorTolerance);
            c = '-';
        }
        else if (c=='2'){
            moveElevatorToHeight_MM(elevatorLevel.level2, elevatorTolerance);
            c = '-';
        }
        else if (c=='3'){
            moveElevatorToHeight_MM(elevatorLevel.level3, elevatorTolerance);
            c = '-';
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
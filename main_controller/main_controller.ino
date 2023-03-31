#include "Constants.h"
#include "Drive.h"
#include "Motor.h"
#include "Intake.h"
#include "Elevator.h"

Drive mDrive;
Elevator mElevator;
Intake mIntake;

unsigned long debugTime = 0;
float targetSpeed = 0.8;
int state = 0;
unsigned long stateTime = 0;

void setup(){
    mDrive.init();
    mElevator.setSpeed(1000);
    Serial.begin(9600);
    //Serial.write("<target>");
    attachInterrupt(digitalPinToInterrupt(Constants::kFrontLeftEncoder), interruptFL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Constants::kFrontRightEncoder), interruptFR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Constants::kBackLeftEncoder), interruptBL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Constants::kBackRightEncoder), interruptBR, CHANGE);
    stateTime = millis();
    mElevator.setPosition(ElevatorPosition::SecondWarehouse);
}

void loop(){
    switch( state ){
        case 0:
            //mDrive.setSpeed(0.8, 0, 0);
            //if( millis() - stateTime > 1000 ){
            if( mElevator.positionReached() ){
                stateTime = millis();
                mIntake.drop();
                state = 1;
            }
            break;
        case 1:
            //mDrive.stop();
            break;  
    }
    //mDrive.setSpeed(0.8, 0, 0);
    //mDrive.periodicIO();

    //mElevator.setPosition(ElevatorPosition::SecondWarehouse);
    mElevator.periodicIO();

    // Plot (TODO: make a library for this)
    if( millis() - debugTime > 50 ){
        //Serial.println(mDrive.getSpeed(MotorID::FrontLeft));
        //plotData(mDrive.getSpeed(MotorID::FrontLeft), mDrive.getSpeed(MotorID::FrontRight), mDrive.getSpeed(MotorID::BackLeft), mDrive.getSpeed(MotorID::BackRight), targetSpeed);
        debugTime = millis();
    }
    //delay(10);
}

void plotData(float data1, float data2, float data3, float data4, float data5){
    const byte *byteData1 = (byte *)(&data1);
    const byte *byteData2 = (byte *)(&data2);
    const byte *byteData3 = (byte *)(&data3);
    const byte *byteData4 = (byte *)(&data4);
    const byte *byteData5 = (byte *)(&data5);

    const byte buf[20] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                          byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                          byteData3[0], byteData3[1], byteData3[2], byteData3[3],
                          byteData4[0], byteData4[1], byteData4[2], byteData4[3],
                          byteData5[0], byteData5[1], byteData5[2], byteData5[3]};

    //Serial.write(buf, 20);
    for(int i=0; i<20; i++){
        Serial.print(buf[i]);
        Serial.print(" ");
    }
    Serial.println();
}

void interruptFL(){
    mDrive.encoderInterrupt(MotorID::FrontLeft);
}
void interruptFR(){
    mDrive.encoderInterrupt(MotorID::FrontRight);
}
void interruptBL(){
    mDrive.encoderInterrupt(MotorID::BackLeft);
}
void interruptBR(){
    mDrive.encoderInterrupt(MotorID::BackRight);
}
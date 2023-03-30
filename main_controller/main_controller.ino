#include "Constants.h"
#include "Drive.h"
#include "Motor.h"

Drive mDrive;

unsigned long debugTime = 0;
float targetSpeed = 0.8;

void setup(){
    mDrive.init();
    Serial.begin(9600);
    attachInterrupt(digitalPinToInterrupt(Constants::kFrontLeftEncoder), interruptFL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Constants::kFrontRightEncoder), interruptFR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Constants::kBackLeftEncoder), interruptBL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Constants::kBackRightEncoder), interruptBR, CHANGE);
}

void loop(){
    mDrive.setSpeed(0.9, 0, 0);
    mDrive.periodicIO();


    // Plot (TODO: make a library for this)
    if( millis() - debugTime > 50 ){
        Serial.println(mDrive.getSpeed(MotorID::FrontLeft));
        //plotData(mDrive.getSpeed(MotorID::FrontLeft), mDrive.getSpeed(MotorID::FrontRight), mDrive.getSpeed(MotorID::BackLeft), mDrive.getSpeed(MotorID::BackRight), targetSpeed);
        debugTime = millis();
    }
    delay(10);
}

void plotData(float data1, float data2, float data3, float data4, float data5){
    byte byteData1[4];
    byte byteData2[4];
    byte byteData3[4];
    byte byteData4[4];
    byte byteData5[4];
    byteData1[0] = char((int)data1);
    byteData1[1] = char((int)(data1*10) % 10);
    byteData1[2] = char((int)(data1*100) % 100);
    byteData1[3] = char((int)(data1*1000) % 1000);
    byteData2[0] = char((int)data2);
    byteData2[1] = char((int)(data2*10) % 10);
    byteData2[2] = char((int)(data2*100) % 100);
    byteData2[3] = char((int)(data2*1000) % 1000);
    byteData3[0] = char((int)data3);
    byteData3[1] = char((int)(data3*10) % 10);
    byteData3[2] = char((int)(data3*100) % 100);
    byteData3[3] = char((int)(data3*1000) % 1000);
    byteData4[0] = char((int)data4);
    byteData4[1] = char((int)(data4*10) % 10);
    byteData4[2] = char((int)(data4*100) % 100);
    byteData4[3] = char((int)(data4*1000) % 1000);
    byteData5[0] = char((int)data5);
    byteData5[1] = char((int)(data5*10) % 10);
    byteData5[2] = char((int)(data5*100) % 100);
    byteData5[3] = char((int)(data5*1000) % 1000);

    const byte buf[20] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                          byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                          byteData3[0], byteData3[1], byteData3[2], byteData3[3],
                          byteData4[0], byteData4[1], byteData4[2], byteData4[3],
                          byteData5[0], byteData5[1], byteData5[2], byteData5[3]};

    Serial.write(buf, 20);
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
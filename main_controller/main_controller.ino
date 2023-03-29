#include "Constants.h"
#include "Drive.h"
#include "Motor.h"

Drive mDrive;

void setup(){
    mDrive.init();
    Serial.begin(9600);
}

void loop(){
    mDrive.setSpeed(0.8, 0, 0);
    mDrive.periodicIO();
    delay(50);
}
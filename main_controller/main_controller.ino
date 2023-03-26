#include "Constants.h"
#include "Drive.h"
#include "Motor.h"

//Drive mecanumDrive;
Motor motor1(Constants::kFrontLeftA, Constants::kFrontLeftB, Constants::kFrontLeftEncoder);

void setup(){
    Serial.begin(9600);
}

void loop(){
    motor1.setSpeed(0.8);
}
#include "Constants.h"
#include "Motor.h"

//Drive mecanumDrive;
Motor motor1(Constants::kFrontLeftA, Constants::kFrontLeftB, Constants::kFrontLeftEncoder);
Motor motor2(Constants::kFrontRightA, Constants::kFrontRightB, Constants::kFrontRightEncoder);

void encoderInterrupt1(){
    motor1.encoderInterrupt();
}
void encoderInterrupt2(){
    motor2.encoderInterrupt();
}

void setup(){
    Serial.begin(9600);
    attachInterrupt(digitalPinToInterrupt(Constants::kFrontLeftEncoder), encoderInterrupt1, RISING);
    attachInterrupt(digitalPinToInterrupt(Constants::kFrontRightEncoder), encoderInterrupt2, RISING);
}

void loop(){
    motor1.setSpeed(0.8);
    motor2.setSpeed(0.8);
    motor1.periodicIO();
    motor2.periodicIO();

    Serial.print(motor1.getTicks());
    Serial.print(" ");
    Serial.println(motor2.getTicks());
    delay(50);
}
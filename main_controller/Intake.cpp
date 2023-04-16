#include "Intake.h"

Intake::Intake(){
    pinMode(Constants::kIntakeMotor1A, OUTPUT);
    pinMode(Constants::kIntakeMotor1B, OUTPUT);
    pinMode(Constants::kIntakeMotor2A, OUTPUT);
    pinMode(Constants::kIntakeMotor2B, OUTPUT);
    pinMode(Constants::kIntakePresence, INPUT);
}

void Intake::pick(){
    if( presence ){
        stop();
        return;
    }
    analogWrite(Constants::kIntakeMotor1A, Constants::kIntakePickSpeed);
    analogWrite(Constants::kIntakeMotor1B, 0);
    analogWrite(Constants::kIntakeMotor2A, 0);
    analogWrite(Constants::kIntakeMotor2B, Constants::kIntakePickSpeed);
}

void Intake::in(){
    if( !presence ){
        stop();
        return;
    }
    analogWrite(Constants::kIntakeMotor1A, Constants::kIntakeInSpeed);
    analogWrite(Constants::kIntakeMotor1B, 0);
    analogWrite(Constants::kIntakeMotor2A, 0);
    analogWrite(Constants::kIntakeMotor2B, Constants::kIntakeInSpeed);
}

void Intake::out(unsigned long current_time){
    if( presence && current_time - presence_detection_time > 250 ){
        stop();
        return;
    }
    analogWrite(Constants::kIntakeMotor1A, 0);
    analogWrite(Constants::kIntakeMotor1B, Constants::kIntakeOutSpeed);
    analogWrite(Constants::kIntakeMotor2A, Constants::kIntakeOutSpeed);
    analogWrite(Constants::kIntakeMotor2B, 0);
}

void Intake::drop(){
    if( !presence ){
        stop();
        return;
    }
    analogWrite(Constants::kIntakeMotor1A, 0);
    analogWrite(Constants::kIntakeMotor1B, Constants::kIntakeDropSpeed);
    analogWrite(Constants::kIntakeMotor2A, Constants::kIntakeDropSpeed);
    analogWrite(Constants::kIntakeMotor2B, 0);
}

void Intake::stop(){
    analogWrite(Constants::kIntakeMotor1A, 0);
    analogWrite(Constants::kIntakeMotor1B, 0);
    analogWrite(Constants::kIntakeMotor2A, 0);
    analogWrite(Constants::kIntakeMotor2B, 0);
}

bool Intake::getPresence(){
    return presence;
}

void Intake::periodicIO(unsigned long current_time){
    presence = !digitalRead(Constants::kIntakePresence);
    if( presence && !flag ){
        presence_detection_time = current_time;
        flag = true;
    }
    if( !presence && flag){
        flag = false;
    }
}
#include "Intake.h"

Intake::Intake(){
    action = IntakeActions::Stop;
    pinMode(Constants::kIntakeMotor1PWM, OUTPUT);
    pinMode(Constants::kIntakeMotor1A, OUTPUT);
    pinMode(Constants::kIntakeMotor1B, OUTPUT);
    pinMode(Constants::kIntakeMotor2PWM, OUTPUT);
    pinMode(Constants::kIntakeMotor2A, OUTPUT);
    pinMode(Constants::kIntakeMotor2B, OUTPUT);
    pinMode(Constants::kIntakePresence, INPUT);
}

void Intake::pick(){
    if( presence ){
        setAction(Stop);
        return;
    }
    analogWrite(Constants::kIntakeMotor1PWM, Constants::kIntakePickSpeed);
    analogWrite(Constants::kIntakeMotor2PWM, Constants::kIntakePickSpeed); 
    digitalWrite(Constants::kIntakeMotor1A, 1);
    digitalWrite(Constants::kIntakeMotor1B, 0);
    digitalWrite(Constants::kIntakeMotor2A, 0);
    digitalWrite(Constants::kIntakeMotor2B, 1);
}

void Intake::in(){
    if( !presence ){
        setAction(Stop);
        return;
    }
    analogWrite(Constants::kIntakeMotor1PWM, Constants::kIntakeInSpeed);
    analogWrite(Constants::kIntakeMotor2PWM, Constants::kIntakeInSpeed);
    digitalWrite(Constants::kIntakeMotor1A, 1);
    digitalWrite(Constants::kIntakeMotor1B, 0);
    digitalWrite(Constants::kIntakeMotor2A, 0);
    digitalWrite(Constants::kIntakeMotor2B, 1);
}

void Intake::out(unsigned long current_time){
    if( presence && current_time - presence_detection_time > 350 ){
        setAction(Stop);
        return;
    }
    analogWrite(Constants::kIntakeMotor1PWM, Constants::kIntakeOutSpeed);
    analogWrite(Constants::kIntakeMotor2PWM, Constants::kIntakeOutSpeed);
    digitalWrite(Constants::kIntakeMotor1A, 0);
    digitalWrite(Constants::kIntakeMotor1B, 1);
    digitalWrite(Constants::kIntakeMotor2A, 1);
    digitalWrite(Constants::kIntakeMotor2B, 0);
}

void Intake::drop(unsigned long current_time){
    if( !presence && current_time - presence_detection_time > 1000){
        setAction(Stop);
        return;
    }
    analogWrite(Constants::kIntakeMotor1PWM, Constants::kIntakeDropSpeed);
    analogWrite(Constants::kIntakeMotor2PWM, Constants::kIntakeDropSpeed);
    digitalWrite(Constants::kIntakeMotor1A, 0);
    digitalWrite(Constants::kIntakeMotor1B, 1);
    digitalWrite(Constants::kIntakeMotor2A, 1);
    digitalWrite(Constants::kIntakeMotor2B, 0);
}

void Intake::stop(){
    digitalWrite(Constants::kIntakeMotor1A, 0);
    digitalWrite(Constants::kIntakeMotor1B, 0);
    digitalWrite(Constants::kIntakeMotor2A, 0);
    digitalWrite(Constants::kIntakeMotor2B, 0);
}

bool Intake::getPresence(){
    return presence;
}

void Intake::setAction(IntakeActions action){
    this->action = action;
}

void Intake::periodicIO(unsigned long current_time){
    if( current_time - last_time < loop_time )
        return;

    presence = !digitalRead(Constants::kIntakePresence);
    if( presence && !flag ){
        presence_detection_time = current_time;
        flag = true;
    }
    if( !presence && flag){
        flag = false;
        presence_detection_time = current_time;
    }

    switch(action){
        case In:
            in();
            break;
        case Out:
            out(current_time);
            break;
        case Pick:
            pick();
            break;
        case Drop:
            drop(current_time);
            break;
        case Stop:
            stop();
            break;
    }

    last_time = current_time;
}
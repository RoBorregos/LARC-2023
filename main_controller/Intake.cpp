#include "Intake.h"
#include "Constants.h"

Intake::Intake(){
    action = IntakeActions::Stop;
    pinMode(Constants::kIntakeMotor1A, OUTPUT);
    pinMode(Constants::kIntakeMotor1B, OUTPUT);
    pinMode(Constants::kIntakeMotor2A, OUTPUT);
    pinMode(Constants::kIntakeMotor2B, OUTPUT);
    pinMode(Constants::kIntakePresence, INPUT);
}

void Intake::pick(unsigned long current_time){
    if( presence && current_time - presence_detection_time > Constants::kIntakePickPresenceTime){
        setAction(Stop);
        return;
    }
    analogWrite(Constants::kIntakeMotor1A, Constants::kIntakePickSpeed);
    analogWrite(Constants::kIntakeMotor1B, 0);
    analogWrite(Constants::kIntakeMotor2A, Constants::kIntakePickSpeed);
    analogWrite(Constants::kIntakeMotor2B, 0);
}

void Intake::in(unsigned long current_time){
    if( !presence && current_time - presence_detection_time > Constants::kIntakeInPresenceTime){
        setAction(Stop);
        return;
    }
    analogWrite(Constants::kIntakeMotor1A, Constants::kIntakePickSpeed);
    analogWrite(Constants::kIntakeMotor1B, 0);
    analogWrite(Constants::kIntakeMotor2A, Constants::kIntakePickSpeed);
    analogWrite(Constants::kIntakeMotor2B, 0);
}

void Intake::out(unsigned long current_time){
    if( presence && current_time - presence_detection_time > 250 ){
        setAction(Stop);
        return;
    }
    analogWrite(Constants::kIntakeMotor1A, 0);
    analogWrite(Constants::kIntakeMotor1B, Constants::kIntakePickSpeed);
    analogWrite(Constants::kIntakeMotor2A, 0);
    analogWrite(Constants::kIntakeMotor2B, Constants::kIntakePickSpeed);
}

void Intake::drop(unsigned long current_time){
    if( !presence && current_time - presence_detection_time > Constants::kIntakeOutPresenceTime){
        setAction(Stop);
        return;
    }
    analogWrite(Constants::kIntakeMotor1A, 0);
    analogWrite(Constants::kIntakeMotor1B, Constants::kIntakePickSpeed);
    analogWrite(Constants::kIntakeMotor2A, 0);
    analogWrite(Constants::kIntakeMotor2B, Constants::kIntakePickSpeed);
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

void Intake::setAction(IntakeActions action){
    this->action = action;
}

void Intake::periodicIO(unsigned long current_time){
    if( current_time - last_time < loop_time )
        return;

    presence = digitalRead(Constants::kIntakePresence);
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
            in(current_time);
            break;
        case Out:
            out(current_time);
            break;
        case Pick:
            pick(current_time);
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
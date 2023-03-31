#include "Intake.h"

Intake::Intake(){
    pinMode(Constants::kIntakeMotor1A, OUTPUT);
    pinMode(Constants::kIntakeMotor1B, OUTPUT);
    pinMode(Constants::kIntakeMotor2A, OUTPUT);
    pinMode(Constants::kIntakeMotor2B, OUTPUT);
}

void Intake::pick(){
    analogWrite(Constants::kIntakeMotor1A, Constants::kIntakePickSpeed);
    analogWrite(Constants::kIntakeMotor1B, 0);
    analogWrite(Constants::kIntakeMotor2A, 0);
    analogWrite(Constants::kIntakeMotor2B, Constants::kIntakePickSpeed);
}

void Intake::drop(){
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
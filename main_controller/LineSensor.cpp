#include "LineSensor.h"

LineSensor::LineSensor(){
    pinMode(Constants::kLineSensorS0, OUTPUT);
    pinMode(Constants::kLineSensorS1, OUTPUT);
    pinMode(Constants::kLineSensorS2, OUTPUT);
    pinMode(Constants::kLineSensorS3, OUTPUT);
    pinMode(Constants::kLineSensorSignal, INPUT);
}

bool LineSensor::lineDetected(SensorID id){
    digitalWrite(Constants::kLineSensorS0, id & 1);
    digitalWrite(Constants::kLineSensorS1, id & 2);
    digitalWrite(Constants::kLineSensorS2, id & 4);
    digitalWrite(Constants::kLineSensorS3, id & 8);
    return analogRead(Constants::kLineSensorSignal) > Constants::kLineSensorValue;
}

int LineSensor::getData(){
    int data = 0;
    for( int i = 0; i < 16; i++ ){
        data |= lineDetected((SensorID)i) << i;
    }
    return data;
}
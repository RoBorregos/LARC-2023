#include "LineSensor.h"

LineSensor::LineSensor(){
    pinMode(Constants::kLineSensorS0, OUTPUT);
    pinMode(Constants::kLineSensorS1, OUTPUT);
    pinMode(Constants::kLineSensorS2, OUTPUT);
    pinMode(Constants::kLineSensorS3, OUTPUT);
    pinMode(Constants::kLineSensorSignal, INPUT);
    calibrate();
}

void LineSensor::calibrate(){
    long sums[16] = {0};

    for (int i = 0; i < 100; i++) {
        for (int j = 0; j < 16; j++) {
            digitalWrite(Constants::kLineSensorS0, j & 1);
            digitalWrite(Constants::kLineSensorS1, j & 2);
            digitalWrite(Constants::kLineSensorS2, j & 4);
            digitalWrite(Constants::kLineSensorS3, j & 8);
            sums[j] += analogRead(Constants::kLineSensorSignal);
        }
    }

    for (int i = 0; i < 16; i++) {
        averages[i] = sums[i] / 100;
    }
}

bool LineSensor::lineDetected(SensorID id){
    digitalWrite(Constants::kLineSensorS0, id & 1);
    digitalWrite(Constants::kLineSensorS1, id & 2);
    digitalWrite(Constants::kLineSensorS2, id & 4);
    digitalWrite(Constants::kLineSensorS3, id & 8);
    return analogRead(Constants::kLineSensorSignal) > averages[id]+Constants::kLineSensorTreshold;
}

bool LineSensor::lineDetected(int id){
    digitalWrite(Constants::kLineSensorS0, id & 1);
    digitalWrite(Constants::kLineSensorS1, id & 2);
    digitalWrite(Constants::kLineSensorS2, id & 4);
    digitalWrite(Constants::kLineSensorS3, id & 8);
    return analogRead(Constants::kLineSensorSignal) > averages[id]+Constants::kLineSensorTreshold;
}


int LineSensor::getData(){
    int data = 0;
    for( int i = 0; i < 16; i++ ){
        data |= lineDetected((SensorID)i) << i;
    }
    return data;
}
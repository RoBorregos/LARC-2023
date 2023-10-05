#include "LineSensor.h"

LineSensor lineSensor;

void setup(){
    Serial.begin(115200);
}

void loop(){
    for (int i = 0; i < 16; i++) {
        Serial.print(lineSensor.lineDetected(i));
        Serial.print(" ");
    }
    Serial.println();
    delay(10);
}
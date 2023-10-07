#ifndef BNO_h
#define BNO_h

#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

struct Orientation{
    float x;
    float y;
    float z;
};

class BNO{
    private:
        Adafruit_BNO055 *bno;
    public:
        BNO(Adafruit_BNO055 *bno);
        bool init();
        Orientation getOrientation();
};

#endif
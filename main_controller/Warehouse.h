#ifndef Warehouse_h
#define Warehouse_h

#include "Arduino.h"
#include "Constants.h"
#include "Wire.h"
#include "Adafruit_VL53L0X.h"

enum CubePosition{
    Empty = 250,
    One = 215,
    Two = 155,
    Three = 95,
    Four = 35
};

struct Level{
    int fwdPin;
    int revPin;
    int speed;
    int demand;
    bool dir;
    Adafruit_VL53L0X* tof;
    uint16_t distance;
    CubePosition cubeState;
};

class Warehouse{
    private:
        Level upper;
        Level mid;
        Level lower;
        Level* levels[3];
    public:
        void init(Adafruit_VL53L0X* tof1);//, Adafruit_VL53L0X* tof2, Adafruit_VL53L0X* tof3);
        void cubeOut(String level);
        CubePosition getCubeState(String level);
        void periodicIO(unsigned long current_time);
        int setSpeed(int a, int b, float speed);
};

#endif
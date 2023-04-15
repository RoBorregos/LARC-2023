#ifndef Warehouse_h
#define Warehouse_h

#include "Arduino.h"
#include "Constants.h"
#include "Wire.h"
#include "Adafruit_VLX53L0X.h"

enum CubePosition{
    Empty = 230,
    One = 180,
    Two = 130,
    Three = 80,
    Four = 30 
};

struct Level{
    int fwdPin;
    int revPin;
    int speed;
    Adafruit_VLX53L0X* tof;
    uint16_t distance;
    CubePosition cubeState;
};

class Warehouse{
    private:
        Level upper;
        Level mid;
        Level lower;
    public:
        void init();
        void cubeIn(string level);
        void cubeOut(string level);
        CubePosition getCubeState(string level);
        void periodicIO(unsigned long current_time);
        void setSpeed();
};

#endif
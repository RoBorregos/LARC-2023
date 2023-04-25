#ifndef Warehouse_h
#define Warehouse_h

#include "Arduino.h"
#include "Constants.h"
#include "Wire.h"
#include "Adafruit_VL53L0X.h"

enum CubePosition{
    Empty = 260,
    One = 225,
    Two = 165,
    Three = 105,
    Four = 65
};

enum LevelPosition{
    Upper = 0,
    Mid = 1,
    Lower = 2
};

struct Level{
    int pwmPin = -1;
    int fwdPin;
    int revPin;
    int speed;
    int demand = 0;
    Adafruit_VL53L0X* tof;
    uint16_t distance = 0;
    CubePosition cube_state = CubePosition::Four;
    unsigned long state_time;
    bool stopped = true;
};

class Warehouse{
    private:
        constexpr static float loop_time = 50;
        Level level[3];
        unsigned long last_time = 0;
    public:
        void init(unsigned long current_time, Adafruit_VL53L0X* tof1, Adafruit_VL53L0X* tof2, Adafruit_VL53L0X* tof3);
        void cubeOut(LevelPosition pos, unsigned long current_time);
        CubePosition getCubeState(LevelPosition pos);
        void periodicIO(unsigned long current_time);
        void stop(LevelPosition pos);
};

#endif
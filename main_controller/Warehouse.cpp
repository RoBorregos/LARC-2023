#include "Warehouse.h"

void Warehouse::init(unsigned long current_time, Adafruit_VL53L0X* tof1, Adafruit_VL53L0X* tof2, Adafruit_VL53L0X* tof3){
    upper.fwdPin = Constants::kWarehouseUpperMotorA;
    upper.revPin = Constants::kWarehouseUpperMotorB;
    upper.speed = Constants::kWarehouseUpperSpeed;
    upper.demand = 0;
    upper.tof = tof1;
    upper.distance = 0;
    upper.cube_state = CubePosition::Four;
    upper.state_time = current_time;

    mid.fwdPin = Constants::kWarehouseMidMotorA;
    mid.revPin = Constants::kWarehouseMidMotorB;
    mid.speed = Constants::kWarehouseMidSpeed;
    mid.demand = 0;
    mid.tof = tof2;
    mid.distance = 0;
    mid.cube_state = CubePosition::Four;
    mid.state_time = current_time;

    lower.fwdPin = Constants::kWarehouseLowerMotorA;
    lower.revPin = Constants::kWarehouseLowerMotorB;
    lower.speed = Constants::kWarehouseLowerSpeed;
    lower.demand = 0;
    lower.tof = tof3;
    lower.distance = 0;
    lower.cube_state = CubePosition::Four;
    lower.state_time = current_time;

    levels[0] = &upper;
    levels[1] = &mid;
    levels[2] = &lower;

    pinMode(upper.fwdPin, OUTPUT);
    pinMode(upper.revPin, OUTPUT);
    pinMode(37, OUTPUT);
    pinMode(38, OUTPUT);
    pinMode(lower.fwdPin, OUTPUT);
    pinMode(lower.revPin, OUTPUT);

    last_time = current_time;
}

void Warehouse::cubeOut(LevelPosition pos, unsigned long current_time){
    Level* lvl = levels[pos];
    lvl->state_time = current_time;

    switch( lvl->cube_state ){
        case CubePosition::Empty:
            break;
        case CubePosition::One:
            lvl->cube_state = CubePosition::Empty;
            break;
        case CubePosition::Two:
            lvl->cube_state = CubePosition::One;
            break;
        case CubePosition::Three:
            lvl->cube_state = CubePosition::Two;
            break;
        case CubePosition::Four:
            lvl->cube_state = CubePosition::Three;
            break;
    }
}

CubePosition Warehouse::getCubeState(String level){
    Level* lvl;
    if( level == "upper" )
        lvl = &upper;
    else if( level == "mid" )
        lvl = &mid;
    else if( level == "lower" )
        lvl = &lower;
    else
        return CubePosition::Empty;

    return lvl->cube_state;
}

void Warehouse::periodicIO(unsigned long current_time){
    if( current_time - last_time < loop_time )
        return;

    Level* lvl = &mid;
    //for(auto lvl : levels){
        lvl->distance = lvl->tof->readRange();
        Serial.print(lvl->distance);
        Serial.print(" ");
        int error = - (lvl->distance - (int)lvl->cube_state);
        if( lvl->cube_state == CubePosition::Four ){
            lvl->demand = 0;
        } else if( error > 5){
            lvl->demand = lvl->speed;
        } else {
            lvl->demand = 0;
        }

        if( current_time - lvl->state_time > 4000 )
            lvl->demand = 0;

        analogWrite(37, lvl->demand>0? abs(lvl->demand) : 0);
        analogWrite(38, lvl->demand<0? abs(lvl->demand) : 0);

        Serial.print(lvl->fwdPin);
        Serial.print(" ");
        Serial.print(lvl->revPin);
        Serial.print(" ");
        Serial.println(lvl->demand);

    //}

    last_time = current_time;
}
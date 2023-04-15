#include "Warehouse.h"

void Warehouse::init(Adafruit_VLX53L0X* tof1, Adafruit_VLX53L0X* tof2, Adafruit_VLX53L0X* tof3){
    upper.fwdPin = Constants::kWarehouseUpperMotorA;
    upper.revPin = Constants::kWarehouseUpperMotorB;
    upper.speed = 0;
    upper.tof = tof1;
    upper.distance = 0;
    upper.cubeState = CubePosition::Empty;

    mid.fwdPin = Constants::kWarehouseMidMotorA;
    mid.revPin = Constants::kWarehouseMidMotorB;
    mid.speed = 0;
    mid.tof = tof2;
    mid.distance = 0;
    mid.cubeState = CubePosition::Empty;

    lower.fwdPin = Constants::kWarehouseLowerMotorA;
    lower.revPin = Constants::kWarehouseLowerMotorB;
    lower.speed = 0;
    lower.tof = tof3;
    lower.distance = 0;
    lower.cubeState = CubePosition::Empty;

    pinMode(upper.fwdPin, OUTPUT);
    pinMode(upper.revPin, OUTPUT);
    pinMode(mid.fwdPin, OUTPUT);
    pinMode(mid.revPin, OUTPUT);
    pinMode(lower.fwdPin, OUTPUT);
    pinMode(lower.revPin, OUTPUT);
}

void Warehouse::cubeIn(string level){
    Level* lvl;
    if( level == "upper" )
        lvl = &upper;
    else if( level == "mid" )
        lvl = &mid;
    else if( level == "lower" )
        lvl = &lower;
    else
        return;

    switch( lvl->cubeState ){
        case CubePosition::Empty:
            lvl->cubeState = CubePosition::One;
            break;
        case CubePosition::One:
            lvl->cubeState = CubePosition::Two;
            break;
        case CubePosition::Two:
            lvl->cubeState = CubePosition::Three;
            break;
        case CubePosition::Three:
            lvl->cubeState = CubePosition::Four;
            break;
        case CubePosition::Four:
            break;
    }
}

void Warehouse::cubeOut(string level){
    Level* lvl;
    if( level == "upper" )
        lvl = &upper;
    else if( level == "mid" )
        lvl = &mid;
    else if( level == "lower" )
        lvl = &lower;
    else
        return;

    switch( lvl->cubeState ){
        case CubePosition::Empty:
            break;
        case CubePosition::One:
            lvl->cubeState = CubePosition::Empty;
            break;
        case CubePosition::Two:
            lvl->cubeState = CubePosition::One;
            break;
        case CubePosition::Three:
            lvl->cubeState = CubePosition::Two;
            break;
        case CubePosition::Four:
            lvl->cubeState = CubePosition::Three;
            break;
    }
}

CubePosition Warehouse::getCubeState(string level){
    Level* lvl;
    if( level == "upper" )
        lvl = &upper;
    else if( level == "mid" )
        lvl = &mid;
    else if( level == "lower" )
        lvl = &lower;
    else
        return CubePosition::Empty;

    return lvl->cubeState;
}

void Warehouse::periodicIO(unsigned long current_time){
    upper.distance = upper.tof->readRange();
    mid.distance = mid.tof->readRange();
    lower.distance = lower.tof->readRange();
}

void Warehouse::setSpeed(){

}
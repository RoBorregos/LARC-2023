#include "Warehouse.h"

void Warehouse::init(Adafruit_VL53L0X* tof1){//, Adafruit_VL53L0X* tof2, Adafruit_VL53L0X* tof3){
    upper.fwdPin = Constants::kWarehouseUpperMotorA;
    upper.revPin = Constants::kWarehouseUpperMotorB;
    upper.speed = 0;
    upper.kP = Constants::kWarehouseUpperKP;
    //upper.tof = tof1;
    upper.distance = (int)CubePosition::Empty;
    upper.cubeState = CubePosition::Empty;

    mid.fwdPin = Constants::kWarehouseMidMotorA;
    mid.revPin = Constants::kWarehouseMidMotorB;
    mid.speed = 0;
    mid.kP = Constants::kWarehouseMidKP;
//    mid.tof = tof2;
    mid.tof = tof1;
    mid.distance = (int)CubePosition::Empty;
    mid.cubeState = CubePosition::Empty;

    lower.fwdPin = Constants::kWarehouseLowerMotorA;
    lower.revPin = Constants::kWarehouseLowerMotorB;
    lower.speed = 0;
    lower.kP = Constants::kWarehouseLowerKP;
    //lower.tof = tof3;
    lower.distance = (int)CubePosition::Empty;
    lower.cubeState = CubePosition::Empty;

    levels[0] = &upper;
    levels[1] = &mid;
    levels[2] = &lower;

    pinMode(upper.fwdPin, OUTPUT);
    pinMode(upper.revPin, OUTPUT);
    pinMode(mid.fwdPin, OUTPUT);
    pinMode(mid.revPin, OUTPUT);
    pinMode(lower.fwdPin, OUTPUT);
    pinMode(lower.revPin, OUTPUT);
}

void Warehouse::cubeIn(String level){
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

void Warehouse::cubeOut(String level){
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

    return lvl->cubeState;
}

void Warehouse::periodicIO(unsigned long current_time){
    //upper.distance = upper.tof->readRange();
    mid.distance = mid.tof->readRange();
    Serial.print(mid.distance);
    Serial.print(" ");
    //lower.distance = lower.tof->readRange();

    Level* lvl = &mid;
    //for(auto lvl : levels){
        float speed = -lvl->kP*(lvl->distance - (int)lvl->cubeState);
        Serial.println(speed);
        if( (lvl->distance > (int)CubePosition::Empty && speed>0) || (lvl->distance < (int)CubePosition::Four && speed<0 ) ){
            lvl->speed = setSpeed(lvl->fwdPin, lvl->revPin, 0);
        }
        else{
            lvl->speed = setSpeed(lvl->fwdPin, lvl->revPin, -lvl->kP*(lvl->distance - lvl->cubeState));
        }
    //}
}

int Warehouse::setSpeed(int a, int b, float speed){
    bool dir = speed > 0;
    speed = int(abs(speed));
    //deadband
    if( speed < 70 )
        speed = 0;
    else
        speed = min( max(speed, 150) , 200 );

    analogWrite(a, dir? speed : 0);
    analogWrite(b, dir? 0 : speed);

    return speed * (dir? 1 : -1);
}
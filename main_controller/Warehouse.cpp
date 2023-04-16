#include "Warehouse.h"

void Warehouse::init(unsigned long current_time, Adafruit_VL53L0X* tof1){//, Adafruit_VL53L0X* tof2, Adafruit_VL53L0X* tof3){
    upper.fwdPin = Constants::kWarehouseUpperMotorA;
    upper.revPin = Constants::kWarehouseUpperMotorB;
    upper.speed = Constants::kWarehouseUpperSpeed;
    upper.demand = 0;
    //upper.tof = tof1;
    upper.distance = 0;
    upper.cube_state = CubePosition::Four;
    upper.state_time = current_time;

    mid.fwdPin = Constants::kWarehouseMidMotorA;
    mid.revPin = Constants::kWarehouseMidMotorB;
    mid.speed = Constants::kWarehouseMidSpeed;
    mid.demand = 0;
//    mid.tof = tof2;
    mid.tof = tof1;
    mid.distance = 0;
    mid.cube_state = CubePosition::Four;
    mid.state_time = current_time;

    lower.fwdPin = Constants::kWarehouseLowerMotorA;
    lower.revPin = Constants::kWarehouseLowerMotorB;
    lower.speed = Constants::kWarehouseLowerSpeed;
    lower.demand = 0;
    //lower.tof = tof3;
    lower.distance = 0;
    lower.cube_state = CubePosition::Four;
    lower.state_time = current_time;

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
    //upper.distance = upper.tof->readRange();
    mid.distance = mid.tof->readRange();
    Serial.println(mid.distance);
    //lower.distance = lower.tof->readRange();

    Level* lvl = &mid;
    //for(auto lvl : levels){
        int error = - (lvl->distance - (int)lvl->cube_state);
        if( lvl->cube_state == CubePosition::Four && error < -5 ){
            lvl->demand = -lvl->speed;
        } else if( error > 5){
            lvl->demand = lvl->speed;
        } else {
            lvl->demand = 0;
        }

        if( current_time - lvl->state_time > 5000 )
            lvl->demand = 0;

        analogWrite(lvl->fwdPin, lvl->demand>0? abs(lvl->demand) : 0);
        analogWrite(lvl->revPin, lvl->demand<0? abs(lvl->demand) : 0);

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
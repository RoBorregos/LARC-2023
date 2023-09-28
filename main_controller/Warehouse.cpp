#include "Warehouse.h"

void Warehouse::init(unsigned long current_time, Adafruit_VL53L0X* tof1, Adafruit_VL53L0X* tof2, Adafruit_VL53L0X* tof3){
    level[0].fwdPin = Constants::kWarehouseUpperMotorA;
    level[0].revPin = Constants::kWarehouseUpperMotorB;
    level[0].speed = Constants::kWarehouseUpperSpeed;
    level[0].tof = tof1;
    level[0].state_time = current_time;

    level[1].fwdPin = Constants::kWarehouseMidMotorA;
    level[1].revPin = Constants::kWarehouseMidMotorB;
    level[1].speed = Constants::kWarehouseMidSpeed;
    level[1].tof = tof2;
    level[1].state_time = current_time;

    level[2].fwdPin = Constants::kWarehouseLowerMotorA;
    level[2].revPin = Constants::kWarehouseLowerMotorB;
    level[2].speed = Constants::kWarehouseLowerSpeed;
    level[2].tof = tof3;
    level[2].state_time = current_time;

    for(int i=0; i<3; i++){
        pinMode(level[i].fwdPin, OUTPUT);
        pinMode(level[i].fwdPin, OUTPUT);
        stop( (LevelPosition)i );
    }

    last_time = current_time;
}

void Warehouse::cubeOut(LevelPosition pos, unsigned long current_time){
    Level* lvl = &level[pos];
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

    lvl->stopped = false;
}

void Warehouse::reset(){
    for(int i=0; i<3; i++){
        level[i].demand = -level[i].speed;
        
        analogWrite(level[i].fwdPin, level[i].demand>0? abs(level[i].demand) : 0);
        analogWrite(level[i].revPin, level[i].demand<0? abs(level[i].demand) : 0);
    }
    delay(500);
    for(int i=0; i<3; i++){
        stop( (LevelPosition)i );
        level[i].cube_state = CubePosition::Four;
        level[i].stopped = true;
    }
}

CubePosition Warehouse::getCubeState(LevelPosition pos){
    return level[pos].cube_state;
}

void Warehouse::periodicIO(unsigned long current_time){
    if( current_time - last_time < loop_time )
        return;

    //int i=0;
    for(int i=0; i<3; i++){
        if(level[i].stopped){
            stop( (LevelPosition)i );
            continue;
        }

        level[i].distance = level[i].tof->readRange();
        Serial.print(level[i].distance);
        Serial.print(" ");
        int error = - (level[i].distance - (int)level[i].cube_state);

        if( level[i].cube_state == CubePosition::Four ){
            level[i].demand = 0;
        } else {
            level[i].demand = level[i].speed;
        }
        /*} else if( error > 15){
            level[i].demand = level[i].speed;
        } else {
            level[i].demand = 0;
        }*/

        if( current_time - level[i].state_time > 3000 )
            level[i].demand = 0;
        
        analogWrite(level[i].fwdPin, level[i].demand>0? abs(level[i].demand) : 0);
        analogWrite(level[i].revPin, level[i].demand<0? abs(level[i].demand) : 0);
        
        if( level[i].demand == 0 ){
            level[i].stopped = true;
            stop( (LevelPosition)i );
        }
    }

    last_time = current_time;
}

void Warehouse::stop(LevelPosition pos){
    digitalWrite(level[pos].fwdPin, 0);
    digitalWrite(level[pos].revPin, 0);
    level[pos].demand = 0;
}
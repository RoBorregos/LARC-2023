#include "BNO.h"

BNO::BNO(Adafruit_BNO055 *bno){
    this->bno = bno;
}

bool BNO::init(){
    return bno->begin();
}

Orientation BNO::getOrientation(){
    sensors_event_t event; 
    bno->getEvent(&event);
    Orientation orientation;
    orientation.x = event.orientation.x;
    orientation.y = event.orientation.y;
    orientation.z = event.orientation.z;
    // transform all from 0-360 to -180-180
    if(orientation.x > 180){
        orientation.x -= 360;
    }
    if(orientation.y > 180){
        orientation.y -= 360;
    }
    if(orientation.z > 180){
        orientation.z -= 360;
    }
    return orientation;
}
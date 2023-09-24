#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "BNO.h"
  

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28,&Wire2);

BNO mbno = BNO(&bno);
Orientation orientation;

void setup(void) 
{
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  while(!mbno.init())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    delay(1000);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
} 

void loop(){
    Orientation orientation = mbno.getOrientation();
    Serial.print("X: ");
    Serial.print(orientation.x, 4);
    Serial.print("\tY: ");
    Serial.print(orientation.y, 4);
    Serial.print("\tZ: ");
    Serial.print(orientation.z, 4);
    Serial.println("");
    
    delay(100);
}
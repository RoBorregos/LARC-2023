#include "Constants.h"

typedef struct {
  Adafruit_VL53L0X *psensor; // pointer to object
  TwoWire *pwire;
  int id;            // IIC id number for the sensor
  int shutdown_pin;  // which pin for shutdown;
  int interrupt_pin; // which pin to use for interrupts.
  Adafruit_VL53L0X::VL53L0X_Sense_config_t sensor_config;     // options for how to use the sensor
  uint16_t range;        // range value used in continuous mode stuff.
  uint8_t sensor_status; // status from last ranging in continuous.
} sensorList_t;

void interruptFL(){
    mDrive.encoderInterrupt(MotorID::FrontLeft);
}
void interruptFR(){
    mDrive.encoderInterrupt(MotorID::FrontRight);
}
void interruptBL(){
    mDrive.encoderInterrupt(MotorID::BackLeft);
}
void interruptBR(){
    mDrive.encoderInterrupt(MotorID::BackRight);
}


void vlxSetup(){

    sensorList_t sensors[] = {
        {&vlx[0], &Wire, 0x30, Constants::kWarehouseVLXxshutUpper, 0, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0},
        {&vlx[1], &Wire, 0x31, Constants::kWarehouseVLXxshutMid, 0, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0},
        {&vlx[2], &Wire, 0x32, Constants::kWarehouseVLXxshutLower, 0, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0},
        {&vlx[3], &Wire, 0x33, Constants::kElevatorVLXxshut, 0, Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY, 0, 0}
    };

    const int COUNT_SENSORS = sizeof(sensors) / sizeof(sensors[0]); 
    
    for (int i = 0; i < COUNT_SENSORS; i++) {
        pinMode(sensors[i].shutdown_pin, OUTPUT);
        digitalWrite(sensors[i].shutdown_pin, LOW);
    }
    
    for (int i = 0; i < COUNT_SENSORS; i++)
        digitalWrite(sensors[i].shutdown_pin, LOW);
    
    delay(10);
    for (int i = 0; i < COUNT_SENSORS; i++) {
        // one by one enable sensors and set their ID
        digitalWrite(sensors[i].shutdown_pin, HIGH);
        delay(10); // give time to wake up.
        if (!sensors[i].psensor->begin(sensors[i].id, false, sensors[i].pwire, sensors[i].sensor_config)) {
            Serial.print(i, DEC);
            Serial.print(F(": c to start\n"));
        }
    }
}
#ifndef VLX_h
#define VLX_h

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L0X.h>
#include <utility/imumaths.h>

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

class VLX{
    private:
        Adafruit_VL53L0X *_vlx1;
        Adafruit_VL53L0X *_vlx2;
        Adafruit_VL53L0X *_vlx3;
        Adafruit_VL53L0X *_vlx4;
    public:
        void initSensors(Adafruit_VL53L0X *vlx1, Adafruit_VL53L0X *vlx2, Adafruit_VL53L0X *vlx3, Adafruit_VL53L0X *vlx4);
        void restart();
};

#endif
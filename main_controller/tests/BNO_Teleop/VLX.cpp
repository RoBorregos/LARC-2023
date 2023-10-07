#include "VLX.h"
#include "Constants.h"

void VLX::initSensors(Adafruit_VL53L0X *vlx1, Adafruit_VL53L0X *vlx2, Adafruit_VL53L0X *vlx3, Adafruit_VL53L0X *vlx4){
    sensorList_t sensors[] = {
        {vlx1, &Wire, 0x30, Constants::kWarehouseVLXxshutUpper, 0, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0},
        {vlx2, &Wire, 0x31, Constants::kWarehouseVLXxshutMid, 0, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0},
        {vlx3, &Wire, 0x32, Constants::kWarehouseVLXxshutLower, 0, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0},
        {vlx4, &Wire, 0x33, Constants::kElevatorVLXxshut, 0, Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY, 0, 0}
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

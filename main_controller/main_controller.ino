#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#include "Constants.h"
#include "RosBridge.h"
#include "Drive.h"
#include "Motor.h"
#include "Intake.h"
#include "Elevator.h"
#include "LineSensor.h"
//#include "Warehouse.h"

Drive mDrive;
Elevator mElevator;
Intake mIntake;
//Warehouse mWarehouse;
RosBridge ros;

bool ENABLE_ROS = true;

unsigned long debug_time = 0;
int state = -1;
unsigned long state_time = 0;
unsigned long loop_time = 0;

Adafruit_VL53L0X sensor1;
Adafruit_VL53L0X sensor2;
Adafruit_VL53L0X sensor3;
Adafruit_VL53L0X sensor4;

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

sensorList_t sensors[] = {
  {&sensor1, &Wire2, 0x30, 33, 0, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0},
  {&sensor2, &Wire2, 0x31, 32, 0, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0},
  {&sensor3, &Wire1, 0x32, 34, 0, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0},
  {&sensor4, &Wire1, 0x33, 39, 0, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0}
};

const int COUNT_SENSORS = sizeof(sensors) / sizeof(sensors[0]);

unsigned long current_time = 0;

void setup(){
    current_time = millis();

    Wire1.begin();
    Wire2.begin();
    /*for (int i = 0; i < COUNT_SENSORS; i++) {
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
    }*/

    mDrive.init();
    //mWarehouse.init(current_time, &sensor1, &sensor3, &sensor2);
    Serial.begin(115200);

    //Serial.write("<target>");
    attachInterrupt(digitalPinToInterrupt(Constants::kFrontLeftEncoder), interruptFL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Constants::kFrontRightEncoder), interruptFR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Constants::kBackLeftEncoder), interruptBL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Constants::kBackRightEncoder), interruptBR, CHANGE);

    state_time = current_time;
    loop_time = current_time;
    debug_time = current_time;

    if( ENABLE_ROS )
        ros.init(&mDrive, &mIntake, &mElevator);

    //mIntake.setAction(IntakeActions::Pick);
    //mElevator.setPosition(ElevatorPosition::ThirdShelf);
}

void loop(){

    current_time = millis();
    if( ENABLE_ROS )
        ros.spin(current_time);
    mDrive.periodicIO(current_time);
    //mElevator.periodicIO();
    mIntake.periodicIO(current_time);
    //mWarehouse.periodicIO(current_time);

    // Plot (TODO: make a library for this)
    if( current_time - debug_time > 50 ){
        //Serial.println(mDrive.getSpeed(MotorID::FrontLeft));
        //plotData(mDrive.getSpeed(MotorID::FrontLeft), mDrive.getSpeed(MotorID::FrontRight), mDrive.getSpeed(MotorID::BackLeft), mDrive.getSpeed(MotorID::BackRight), targetSpeed);
        debug_time = current_time;
    }
    //delay(10);
}

void plotData(float data1, float data2, float data3, float data4, float data5){
    const byte *byteData1 = (byte *)(&data1);
    const byte *byteData2 = (byte *)(&data2);
    const byte *byteData3 = (byte *)(&data3);
    const byte *byteData4 = (byte *)(&data4);
    const byte *byteData5 = (byte *)(&data5);

    const byte buf[20] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                          byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                          byteData3[0], byteData3[1], byteData3[2], byteData3[3],
                          byteData4[0], byteData4[1], byteData4[2], byteData4[3],
                          byteData5[0], byteData5[1], byteData5[2], byteData5[3]};

    //Serial.write(buf, 20);
    for(int i=0; i<20; i++){
        Serial.print(buf[i]);
        Serial.print(" ");
    }
    Serial.println();
}

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
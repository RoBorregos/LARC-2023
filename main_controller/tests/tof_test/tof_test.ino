/*
 * This code was developed from the example code obtained via the Arduino IDE for the
 * Grove Time-of-Flight Distance Sensor VL53L0X.
 * 
 ***************************************************************************************
 ** WARNING: Some versions of the Arduino IDE software will issue error messages the  **
 **          first time that you compile this code. However, these errors messages    **
 **          disappear on the second and subsequent attempts to compile.              **
 ***************************************************************************************
 * 
 * Wiring:
 *   If you are connecting the distance-sensor directly to an Arduino Uno board, then:
 *     Connect all sensors' GND wires to a GND pin on the Arduino board
 *     Connect all sensors' VCC wires to a 5V pin.
 *     Connect all sensors' SDA wires (Serial DAta line) to analog pin #4 ('A4').
 *     Connect all sensors' SCL wire (Serial CLock line) to analog pin #5 ('A5').
 *     Connect each sensor's XSHUT wire (shut-down line) to a different digital pin
 *             on the Arduino board.
 *   
 *   Alternatively, the sensors' SDA and SCL wires may be connected to the unlabeled
 *   pins beside the AREF pin on the Arduino Uno board. The sensors' SDA wires connect
 *   to the pin immediately beside the AREF pin. The sensors' SCL wires connect to the
 *   next pin, at the edge of the connector.
 *   
 *   If you are wiring the distance-sensors to a Motor Shield board, then follow the
 *   same instructions as the above. (Note that the SDA and SCL pins have better labels
 *   on the Motor Shield board than on the Arduino Uno board.)
*/

// Include the 'Wire.h' libary to allow the Arduino to communicate with the time-of-flight sensors of the 'IIC' bus
#include <Wire.h>  

// Include the 'Adafruit_VL53L0X' libary for the VL53L0X time-of-flight distance sensors
#include <Adafruit_VL53L0X.h>  

// Create a 'Adafruit_VL53L0X' object for each sensor:
Adafruit_VL53L0X sensor1;
Adafruit_VL53L0X sensor2;
Adafruit_VL53L0X sensor3;
Adafruit_VL53L0X sensor4;

//====================================================================
// Define global variables for the sensors:
//====================================================================
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

// Setup for 2 sensors by defining information for each sensor in a 'sensors' array. Include
// a separate line of information for each sensor
sensorList_t sensors[] = {
  // For 'sensor1', define the IIC accress as hexadecimal value 0x30. Assign digital pin #4 to this 
  // sensor's XSHUT pin (shut-down pin). Assign digital pin #5 to the sensor's INTERRUPT pin.
  {&sensor1, &Wire, 0x30, 46, 0, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0},
  // For 'sensor2', define the IIC accress as hexadecimal value 0x31. Assign digital pin #6 to this 
  // sensor's XSHUT pin (shut-down pin). Assign digital pin #7 to the sensor's INTERRUPT pin.
  {&sensor2, &Wire, 0x31, 48, 0, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0},
  {&sensor3, &Wire, 0x32, 50, 0, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0},
  {&sensor4, &Wire, 0x33, 53, 0, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0}
};

// Calculate the number of sensors by checking the size of the above 'sensors' array:
const int COUNT_SENSORS = sizeof(sensors) / sizeof(sensors[0]);

// Create array-variable for the sensors' range (in mm):
uint16_t ranges_mm[COUNT_SENSORS];

//====================================================================
// The 'Initialize_sensors' function:
//====================================================================
/*
    Reset all sensors by setting all of their XSHUT pins low for delay(10), then
    set all XSHUT high to bring out of reset. Keep sensor #1 awake by keeping XSHUT
    pin high. Put all other sensors into shutdown by pulling XSHUT pins low.
    Initialize sensor #1 with lox.begin(new_i2c_address). Pick any number except
    0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its
    XSHUT pin high. Initialize sensor #2 with lox.begin(new_i2c_address) Pick any
    number but 0x29 and whatever you set the first sensor to.
*/
void Initialize_sensors() {
  bool found_any_sensors = false;
  // Set all shutdown pins low to shutdown sensors
  for (int i = 0; i < COUNT_SENSORS; i++)
    digitalWrite(sensors[i].shutdown_pin, LOW);
  delay(10);

  for (int i = 0; i < COUNT_SENSORS; i++) {
    // one by one enable sensors and set their ID
    digitalWrite(sensors[i].shutdown_pin, HIGH);
    delay(10); // give time to wake up.
    if (sensors[i].psensor->begin(sensors[i].id, false, sensors[i].pwire,
                                  sensors[i].sensor_config)) {
      found_any_sensors = true;
    } else {
      Serial.print(i, DEC);
      Serial.print(F(": c to start\n"));
    }
  }
  if (!found_any_sensors) {
    Serial.println("No valid sensors found");
    while (1)
      ;
  }
}  // End of function 'Initialize_sensors'

//====================================================================
// The 'setup' function:
//====================================================================
void setup() {
  // Start the serial monitor at 9600 baud rate:
  Serial.begin(9600);

  // Start the IIC bus:
  Wire.begin();

  // Wait until serial port opens ... For 5 seconds max
  while (!Serial && (millis() < 5000))
    ;

  // Initialize all of the pins.
  Serial.println(F("VL53LOX_multi start, initialize IO pins"));
  for (int i = 0; i < COUNT_SENSORS; i++) {
    pinMode(sensors[i].shutdown_pin, OUTPUT);
    digitalWrite(sensors[i].shutdown_pin, LOW);

    if (sensors[i].interrupt_pin > 0)
      pinMode(sensors[i].interrupt_pin, INPUT_PULLUP);
  }
  Serial.println(F("Starting..."));
  Initialize_sensors();
}  // End of function 'setup'

//====================================================================
// The 'loop' function:
//====================================================================
void loop() {

  // Read the data from the sensors using a 'for' loop:
  for (int i = 0; i < COUNT_SENSORS; i++) {
    ranges_mm[i] = sensors[i].psensor->readRange();  // This is where the sensor's data is captured
  }
  
  // Print out the distances to the serial monitor, again using a 'for' loop:
  for (int i = 0; i < COUNT_SENSORS; i++) {
    Serial.print("S");
    Serial.print(i, DEC);
    Serial.print(" ");
    Serial.print(sensors[i].id, HEX);
    Serial.print(": ");
    Serial.print(ranges_mm[i], DEC);
    Serial.print("  ");
  }
  Serial.println();

  // Delay until the next reading:
  delay(200);  // Argument is in milliseconds
  
}  // End of function 'loop'
//====================================================================



// --------------------------------------
// i2c_scanner
// http://playground.arduino.cc/Main/I2cScanner
//
// Version 1
//    This program (or code that looks like it)
//    can be found in many places.
//    For example on the Arduino.cc forum.
//    The original author is not know.
// Version 2, Juni 2012, Using Arduino 1.0.1
//     Adapted to be as simple as possible by Arduino.cc user Krodal
// Version 3, Feb 26  2013
//    V3 by louarnold
// Version 4, March 3, 2013, Using Arduino 1.0.3
//    by Arduino.cc user Krodal.
//    Changes by louarnold removed.
//    Scanning addresses changed from 0...127 to 1...119,
//    according to the i2c scanner by Nick Gammon
//    http://www.gammon.com.au/forum/?id=10896
// Version 5, March 28, 2013
//    As version 4, but address scans now to 127.
//    A sensor seems to use address 120.
// Version 6, November 27, 2015.
//    Added waiting for the Leonardo serial communication.
//
//
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
//
/*
#include <Wire.h>


void setup() {
  // uncomment these to use alternate pins
  //Wire.setSCL(16);
  //Wire.setSDA(17);
  Wire.begin();
  Serial.begin(9600);
  while (!Serial);        // Leonardo: wait for serial monitor
  Serial.println(F("\nI2C Scanner"));
}


void loop() {
  byte error, address;
  int nDevices;

  Serial.println(F("Scanning..."));

  nDevices = 0;
  for (address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print(F("Device found at address 0x"));
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address,HEX);
      Serial.print("  (");
      printKnownChips(address);
      Serial.println(")");

      nDevices++;
    } else if (error==4) {
      Serial.print(F("Unknown error at address 0x"));
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0) {
    Serial.println(F("No I2C devices found\n"));
  } else {
    Serial.println(F("done\n"));
  }
  delay(5000);           // wait 5 seconds for next scan
}


void printKnownChips(byte address)
{
  // Is this list missing part numbers for chips you use?
  // Please suggest additions here:
  // https://github.com/PaulStoffregen/Wire/issues/new
  switch (address) {
    case 0x00: Serial.print(F("AS3935")); break;
    case 0x01: Serial.print(F("AS3935")); break;
    case 0x02: Serial.print(F("AS3935")); break;
    case 0x03: Serial.print(F("AS3935")); break;
    case 0x04: Serial.print(F("ADAU1966")); break;
    case 0x0A: Serial.print(F("SGTL5000")); break; // MCLK required
    case 0x0B: Serial.print(F("SMBusBattery?")); break;
    case 0x0C: Serial.print(F("AK8963")); break;
    case 0x10: Serial.print(F("CS4272")); break;
    case 0x11: Serial.print(F("Si4713")); break;
    case 0x13: Serial.print(F("VCNL4000,AK4558")); break;
    case 0x18: Serial.print(F("LIS331DLH")); break;
    case 0x19: Serial.print(F("LSM303,LIS331DLH")); break;
    case 0x1A: Serial.print(F("WM8731")); break;
    case 0x1C: Serial.print(F("LIS3MDL")); break;
    case 0x1D: Serial.print(F("LSM303D,LSM9DS0,ADXL345,MMA7455L,LSM9DS1,LIS3DSH")); break;
    case 0x1E: Serial.print(F("LSM303D,HMC5883L,FXOS8700,LIS3DSH")); break;
    case 0x20: Serial.print(F("MCP23017,MCP23008,PCF8574,FXAS21002,SoilMoisture")); break;
    case 0x21: Serial.print(F("MCP23017,MCP23008,PCF8574")); break;
    case 0x22: Serial.print(F("MCP23017,MCP23008,PCF8574")); break;
    case 0x23: Serial.print(F("MCP23017,MCP23008,PCF8574")); break;
    case 0x24: Serial.print(F("MCP23017,MCP23008,PCF8574,ADAU1966,HM01B0")); break;
    case 0x25: Serial.print(F("MCP23017,MCP23008,PCF8574")); break;
    case 0x26: Serial.print(F("MCP23017,MCP23008,PCF8574")); break;
    case 0x27: Serial.print(F("MCP23017,MCP23008,PCF8574,LCD16x2,DigoleDisplay")); break;
    case 0x28: Serial.print(F("BNO055,EM7180,CAP1188")); break;
    case 0x29: Serial.print(F("TSL2561,VL6180,TSL2561,TSL2591,BNO055,CAP1188")); break;
    case 0x2A: Serial.print(F("SGTL5000,CAP1188")); break;
    case 0x2B: Serial.print(F("CAP1188")); break;
    case 0x2C: Serial.print(F("MCP44XX ePot")); break;
    case 0x2D: Serial.print(F("MCP44XX ePot")); break;
    case 0x2E: Serial.print(F("MCP44XX ePot")); break;
    case 0x2F: Serial.print(F("MCP44XX ePot")); break;
    case 0x30: Serial.print(F("Si7210")); break;
    case 0x31: Serial.print(F("Si7210")); break;
    case 0x32: Serial.print(F("Si7210")); break;
    case 0x33: Serial.print(F("MAX11614,MAX11615,Si7210")); break;
    case 0x34: Serial.print(F("MAX11612,MAX11613")); break;
    case 0x35: Serial.print(F("MAX11616,MAX11617")); break;
    case 0x38: Serial.print(F("RA8875,FT6206,MAX98390")); break;
    case 0x39: Serial.print(F("TSL2561, APDS9960")); break;
    case 0x3C: Serial.print(F("SSD1306,DigisparkOLED")); break;
    case 0x3D: Serial.print(F("SSD1306")); break;
    case 0x40: Serial.print(F("PCA9685,Si7021,MS8607")); break;
    case 0x41: Serial.print(F("STMPE610,PCA9685")); break;
    case 0x42: Serial.print(F("PCA9685")); break;
    case 0x43: Serial.print(F("PCA9685")); break;
    case 0x44: Serial.print(F("PCA9685, SHT3X, ADAU1966")); break;
    case 0x45: Serial.print(F("PCA9685, SHT3X")); break;
    case 0x46: Serial.print(F("PCA9685")); break;
    case 0x47: Serial.print(F("PCA9685")); break;
    case 0x48: Serial.print(F("ADS1115,PN532,TMP102,LM75,PCF8591,CS42448")); break;
    case 0x49: Serial.print(F("ADS1115,TSL2561,PCF8591,CS42448")); break;
    case 0x4A: Serial.print(F("ADS1115,Qwiic Keypad,CS42448")); break;
    case 0x4B: Serial.print(F("ADS1115,TMP102,BNO080,Qwiic Keypad,CS42448")); break;
    case 0x50: Serial.print(F("EEPROM,FRAM")); break;
    case 0x51: Serial.print(F("EEPROM")); break;
    case 0x52: Serial.print(F("Nunchuk,EEPROM")); break;
    case 0x53: Serial.print(F("ADXL345,EEPROM")); break;
    case 0x54: Serial.print(F("EEPROM")); break;
    case 0x55: Serial.print(F("EEPROM")); break;
    case 0x56: Serial.print(F("EEPROM")); break;
    case 0x57: Serial.print(F("EEPROM")); break;
    case 0x58: Serial.print(F("TPA2016,MAX21100")); break;
    case 0x5A: Serial.print(F("MPR121")); break;
    case 0x60: Serial.print(F("MPL3115,MCP4725,MCP4728,TEA5767,Si5351")); break;
    case 0x61: Serial.print(F("MCP4725,AtlasEzoDO")); break;
    case 0x62: Serial.print(F("LidarLite,MCP4725,AtlasEzoORP")); break;
    case 0x63: Serial.print(F("MCP4725,AtlasEzoPH")); break;
    case 0x64: Serial.print(F("AtlasEzoEC, ADAU1966")); break;
    case 0x66: Serial.print(F("AtlasEzoRTD")); break;
    case 0x68: Serial.print(F("DS1307,DS3231,MPU6050,MPU9050,MPU9250,ITG3200,ITG3701,LSM9DS0,L3G4200D")); break;
    case 0x69: Serial.print(F("MPU6050,MPU9050,MPU9250,ITG3701,L3G4200D")); break;
    case 0x6A: Serial.print(F("LSM9DS1")); break;
    case 0x6B: Serial.print(F("LSM9DS0")); break;
    case 0x6F: Serial.print(F("Qwiic Button")); break;
    case 0x70: Serial.print(F("HT16K33,TCA9548A")); break;
    case 0x71: Serial.print(F("SFE7SEG,HT16K33")); break;
    case 0x72: Serial.print(F("HT16K33")); break;
    case 0x73: Serial.print(F("HT16K33")); break;
    case 0x76: Serial.print(F("MS5607,MS5611,MS5637,BMP280")); break;
    case 0x77: Serial.print(F("BMP085,BMA180,BMP280,MS5611")); break;
    case 0x7C: Serial.print(F("FRAM_ID")); break;
    default: Serial.print(F("unknown chip"));
  }
}
*/
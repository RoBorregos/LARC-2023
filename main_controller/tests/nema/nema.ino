#include <Stepper.h> 
#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X vlx[4];
#define STEPS 3200


// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver

Stepper stepper(STEPS, 30, 31); // Pin 2 connected to DIRECTION & Pin 3 connected to STEP Pin of Driver

#define motorInterfaceType 1

int Pval = 0;

int potVal = 0;

int STEPS_PER_MM = 310;
int STEPPER_SPEED = 500;

// Elevetor heights: Pick: 70mm; Level1: 155; Level2: 225; Level3: 295

//moves the elevator to a height
void moveElevatorToHeight(uint height, uint tolerance){
  int steps = 50;
  // counter to print how many steps the motor has taken
  long int step_count = 0;
  uint current_height = readVLX(3);
  Serial.print("Initializing, current height: ");
  Serial.println(current_height);
  while (abs(height - current_height) > tolerance){
    if (height > current_height){
      stepper.step(-steps);
    }
    else{
      stepper.step(steps);
    }
    step_count += steps;
    current_height = readVLX(3);
    Serial.print("Initializing, current height: ");
    Serial.println(current_height);
  }
  Serial.print("Done, steps taken: "); Serial.println(step_count); 
  stepper.step(0);
}

//moves the elevator to a height
void moveElevatorToHeight_MM(uint height, uint tolerance){
  
  uint current_height = readVLX(3);
  Serial.print("Current height: ");
  Serial.println(current_height);

  int steps = -(height - current_height) * STEPS_PER_MM;
  Serial.println("Starting movement");
  stepper.step(steps);

  current_height = readVLX(3);

  steps = STEPS_PER_MM/2;
  while (abs(height - current_height) > tolerance){
    if (height > current_height){
      stepper.step(-steps);
    }
    else{
      stepper.step(steps);
    }
    current_height = readVLX(3);
    Serial.print("Initializing, current height: ");
    Serial.println(current_height);
  }
}

// function to read and filter the value from the vlx
uint readVLX(uint sensorID){
    uint range = vlx[sensorID].readRange();
    while (range > 1000){
        range = vlx[sensorID].readRange();
    }
    return range;
}

void setup() {

  // Set the maximum speed in steps per second:

stepper.setSpeed(STEPPER_SPEED); //max 4688 / > 1000 torque
Serial.begin(115200);
vlxSetup();
//moveElevatorToHeight_MM(100, 1);

}

int current_steps = 0;

void loop() {

  if (Serial.available()) {

    //read line and convert to int
    String lineRead = Serial.readStringUntil('\n');
    lineRead.trim();
    Pval = lineRead.toInt();
    Serial.println(Pval);
    //move to a number of steps from 0 position
    int required_steps = Pval - current_steps;
    Serial.print("Required steps: "); Serial.println(required_steps);
    stepper.step(-required_steps);
    current_steps += required_steps;
    // move to a height
    /*if (Pval > 59 && Pval < 300){
      moveElevatorToHeight_MM(Pval, 0);
    }*/
  }
  /*Serial.print("VLX: ");
  Serial.println(readVLX(3));
  delay(1);*/

}
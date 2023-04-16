#ifndef Constants_h
#define Constants_h

#include <Arduino.h>

class Constants{
    private:
    /* data */
    public:
        // Drive constants //
        static constexpr float kWheelDiameter = 0.054; //meters
        static constexpr float kWheelTrack = 0.23; //meters
        static constexpr float kWheelBase = 0.155; //meters
        static constexpr float kDriveKP = 0.1;
        
        //Motors (A: fwd, B: rev)
        static constexpr float kMotorsRPM = 380; //RPM
        static constexpr float kMotorMinPWM = 140; //PWM (0-255)
        static constexpr int kFrontLeftA = 6;
        static constexpr int kFrontLeftB = 7;
        static constexpr int kFrontRightA = 0;
        static constexpr int kFrontRightB = 1;
        static constexpr int kBackLeftA = 5;
        static constexpr int kBackLeftB = 4;
        static constexpr int kBackRightA = 3;
        static constexpr int kBackRightB = 2;

        //Encoders
        static constexpr float kEncoderTicksPerRevolution = 979.62;
        static constexpr int kFrontLeftEncoder = 26;
        static constexpr int kFrontRightEncoder = 23;
        static constexpr int kBackLeftEncoder = 27;
        static constexpr int kBackRightEncoder = 12;

        //PID
        static constexpr float kMotorKP = 5;
        static constexpr float kMotorKI = 0.0;
        static constexpr float kMotorKD = 0.0;

        // Line sensor //
        static constexpr int kLineSensorS0 = 18;
        static constexpr int kLineSensorS1 = 19;
        static constexpr int kLineSensorS2 = 20;
        static constexpr int kLineSensorS3 = 21;
        static constexpr int kLineSensorSignal = 22;
        static constexpr int kLineSensorValue = 500;

        // Intake //
        static constexpr int kIntakeMotor1A = 8;
        static constexpr int kIntakeMotor1B = 9;
        static constexpr int kIntakeMotor2A = 10;
        static constexpr int kIntakeMotor2B = 11;
        static constexpr int kIntakePickSpeed = 255;
        static constexpr int kIntakeDropSpeed = -255;

        // Elevator //
        static constexpr int kStepperSteps = 200;
        static constexpr int kStepperDirectionPin = 28;
        static constexpr int kStepperStepPin = 29;
        static constexpr int kStepperSpeed = 2000; //max 4688 / > 1000 torque
        static constexpr int kElevatorVLXxshut = 34;

        // Warehouse //
        static constexpr int kWarehouseUpperMotorA = 37;
        static constexpr int kWarehouseUpperMotorB = 38;
        static constexpr int kWarehouseMidMotorA = 35;
        static constexpr int kWarehouseMidMotorB = 36;
        static constexpr int kWarehouseLowerMotorA = 14;
        static constexpr int kWarehouseLowerMotorB = 15;
        static constexpr int kWarehouseVLXxshutUpper = 31;
        static constexpr int kWarehouseVLXxshutMid = 32;
        static constexpr int kWarehouseVLXxshutLower = 33;
        static constexpr int kWarehouseUpperSpeed = 180;
        static constexpr int kWarehouseMidSpeed = 150;
        static constexpr int kWarehouseLowerSpeed = 0;
};

#endif
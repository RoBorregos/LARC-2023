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
        static constexpr float kDriveKP = 0.02;
        
        //Motors (A: fwd, B: rev)
        static constexpr float kMotorsRPM = 380; //RPM
        static constexpr float kMotorMinPWM = 140; //PWM (0-255)
        static constexpr int kFrontLeftA = 1;
        static constexpr int kFrontLeftB = 0;
        static constexpr int kFrontRightA = 4;
        static constexpr int kFrontRightB = 5;
        static constexpr int kBackLeftA = 2;
        static constexpr int kBackLeftB = 3;
        static constexpr int kBackRightA = 7;
        static constexpr int kBackRightB = 6;

        //Encoders
        static constexpr float kEncoderTicksPerRevolution = 979.62;
        static constexpr int kFrontLeftEncoder = 23;
        static constexpr int kFrontRightEncoder = 27;
        static constexpr int kBackLeftEncoder = 12;
        static constexpr int kBackRightEncoder = 26;

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
        static constexpr int kIntakeMotor2A = 11;
        static constexpr int kIntakeMotor2B = 10;
        static constexpr int kIntakePresence = 30;
        static constexpr int kIntakePickSpeed = 255;
        static constexpr int kIntakeInSpeed = 255;
        static constexpr int kIntakeOutSpeed = 160;
        static constexpr int kIntakeDropSpeed = 255;

        // Elevator //
        static constexpr int kStepperSteps = 200;
        static constexpr int kStepperDirectionPin = 28;
        static constexpr int kStepperStepPin = 29;
        static constexpr int kStepperSpeed = 2000; //max 4688 / > 1000 torque
        static constexpr int kElevatorVLXxshut = 39;

        // Warehouse //
        static constexpr int kWarehouseUpperMotorA = 35;
        static constexpr int kWarehouseUpperMotorB = 36;
        static constexpr int kWarehouseMidMotorA = 37;
        static constexpr int kWarehouseMidMotorB = 38;
        static constexpr int kWarehouseLowerMotorA = 14;
        static constexpr int kWarehouseLowerMotorB = 15;
        static constexpr int kWarehouseVLXxshutUpper = 32;
        static constexpr int kWarehouseVLXxshutMid = 34;
        static constexpr int kWarehouseVLXxshutLower = 33;
        static constexpr int kWarehouseUpperSpeed = 180;
        static constexpr int kWarehouseMidSpeed = 150;
        static constexpr int kWarehouseLowerSpeed = 180;
};

#endif
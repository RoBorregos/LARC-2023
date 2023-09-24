#ifndef Constants_h
#define Constants_h

#include <Arduino.h>

// Updated 31-08-2023 for teensy 4.1

class Constants{
    private:
    /* data */
    public:
        // Drive constants //
        static constexpr float kWheelDiameter = 0.075; //meters
        static constexpr float kWheelTrack = 0.23; //meters
        static constexpr float kWheelBase = 0.155; //meters
        static constexpr float kDriveKP = 0.017;
        static constexpr float kDriveKD = 0.00;

        static constexpr float kBNOKP = 0.1;
        static constexpr float kBNOKI = 0.0;
        static constexpr float kBNOKD = 1.0;
        static constexpr float kBNOKImax = 0.1;
        static constexpr float KBNOMaxAngular = 2.0;
        static constexpr float kBNOMinAngular = -2.0;
        static constexpr float kAngleTolerance = 3.5;
        
        //Motors (A: fwd, B: rev)
        static constexpr float kMotorsRPM = 380; //RPM
        static constexpr float kMotorMinPWM = 50; //PWM (0-255)
        static constexpr uint8_t kFrontLeftA = 1;
        static constexpr uint8_t kFrontLeftB = 0;
        static constexpr uint8_t kFrontRightA = 7;
        static constexpr uint8_t kFrontRightB = 6;
        static constexpr uint8_t kBackLeftA = 2;
        static constexpr uint8_t kBackLeftB = 3;
        static constexpr uint8_t kBackRightA = 4;
        static constexpr uint8_t kBackRightB = 5;

        //Encoders
        static constexpr float kEncoderTicksPerRevolution = 979.62;
        static constexpr uint8_t kFrontLeftEncoder = 33;
        static constexpr uint8_t kFrontRightEncoder = 34;
        static constexpr uint8_t kBackLeftEncoder = 32;
        static constexpr uint8_t kBackRightEncoder = 35;

        //PID: stable off ground 16, 2, 8, max I 0.05
        static constexpr float kMotorKP = 18.0;
        static constexpr float kMotorKI = 3.0;
        static constexpr float kMotorKImax = 0.05;
        static constexpr float kMotorKD = 8.0;
        static constexpr float kMotorMaxOut = 10.0;
        static constexpr float kMotorMinOut = 0.0;

        // Line sensor //
        static constexpr uint8_t kLineSensorS0 = 14;
        static constexpr uint8_t kLineSensorS1 = 41;
        static constexpr uint8_t kLineSensorS2 = 40;
        static constexpr uint8_t kLineSensorS3 = 39;
        static constexpr uint8_t kLineSensorSignal = 38;
        static constexpr int kLineSensorValue = 800;

        // Intake //
        static constexpr uint8_t kIntakeMotor1A = 22;
        static constexpr uint8_t kIntakeMotor1B = 23;
        static constexpr uint8_t kIntakeMotor2A = 28;
        static constexpr uint8_t kIntakeMotor2B = 29;
        static constexpr uint8_t kIntakePresence = 44;
        static constexpr int kIntakePickSpeed = 255;
        static constexpr int kIntakeInSpeed = 255;
        static constexpr int kIntakeOutSpeed = 140;
        static constexpr int kIntakeDropSpeed = 255;

        // Elevator //
        static constexpr int kStepperSteps = 3200;
        static constexpr uint8_t kStepperDirectionPin = 30;
        static constexpr uint8_t kStepperStepPin = 31;
        static constexpr int kStepperSpeed = 2000; //max 4688 / > 1000 torque
        static constexpr uint8_t kElevatorVLXxshut = 17;

        // Warehouse //
        static constexpr uint8_t kWarehouseUpperMotorA = 8;
        static constexpr uint8_t kWarehouseUpperMotorB = 9;
        static constexpr uint8_t kWarehouseMidMotorA = 10;
        static constexpr uint8_t kWarehouseMidMotorB = 11;
        static constexpr uint8_t kWarehouseLowerMotorA = 29;
        static constexpr uint8_t kWarehouseLowerMotorB = 28;
        static constexpr uint8_t kWarehouseVLXxshutUpper = 16;
        static constexpr uint8_t kWarehouseVLXxshutMid = 15;
        static constexpr uint8_t kWarehouseVLXxshutLower = 20;
        static constexpr int kWarehouseUpperSpeed = 180;
        static constexpr int kWarehouseMidSpeed = 180;
        static constexpr int kWarehouseLowerSpeed = 140;
};

#endif
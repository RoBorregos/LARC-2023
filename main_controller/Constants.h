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
        static constexpr int kFrontLeftPWM = 5;
        static constexpr int kFrontLeftA = 24;
        static constexpr int kFrontLeftB = 25;
        static constexpr int kFrontRightPWM = 4;
        static constexpr int kFrontRightA = 22;
        static constexpr int kFrontRightB = 23;
        static constexpr int kBackLeftPWM = 6;
        static constexpr int kBackLeftA = 27;
        static constexpr int kBackLeftB = 26;
        static constexpr int kBackRightPWM = 7;
        static constexpr int kBackRightA = 29;
        static constexpr int kBackRightB = 28;

        //Encoders
        static constexpr float kEncoderTicksPerRevolution = 979.62;
        static constexpr int kFrontLeftEncoder = 2;
        static constexpr int kFrontRightEncoder = 3;
        static constexpr int kBackLeftEncoder = 18;
        static constexpr int kBackRightEncoder = 19;

        //PID
        static constexpr float kMotorKP = 5;
        static constexpr float kMotorKI = 0.0;
        static constexpr float kMotorKD = 0.0;

        // Line sensor //
        static constexpr int kLineSensorS0 = A11;
        static constexpr int kLineSensorS1 = A12;
        static constexpr int kLineSensorS2 = A13;
        static constexpr int kLineSensorS3 = A14;
        static constexpr int kLineSensorSignal = A15;
        static constexpr int kLineSensorValue = 500;

        // Intake //
        static constexpr int kIntakeMotor1PWM = 9;
        static constexpr int kIntakeMotor1A = 32;
        static constexpr int kIntakeMotor1B = 33;
        static constexpr int kIntakeMotor2PWM = 10;
        static constexpr int kIntakeMotor2A = 35;
        static constexpr int kIntakeMotor2B = 34;
        static constexpr int kIntakePresence = 44;
        static constexpr int kIntakePickSpeed = 255;
        static constexpr int kIntakeInSpeed = 255;
        static constexpr int kIntakeOutSpeed = 160;
        static constexpr int kIntakeDropSpeed = 255;

        // Elevator //
        static constexpr int kStepperSteps = 200;
        static constexpr int kStepperDirectionPin = 36;
        static constexpr int kStepperStepPin = 37;
        static constexpr int kStepperSpeed = 2000; //max 4688 / > 1000 torque
        static constexpr int kElevatorVLXxshut = 53;

        // Warehouse //
        static constexpr int kWarehouseUpperMotorA = 49;
        static constexpr int kWarehouseUpperMotorB = 47;
        static constexpr int kWarehouseMidMotorA = 51;
        static constexpr int kWarehouseMidMotorB = 52;
        static constexpr int kWarehouseLowerPWM = 8;
        static constexpr int kWarehouseLowerMotorA = 30;
        static constexpr int kWarehouseLowerMotorB = 31;
        static constexpr int kWarehouseVLXxshutUpper = 46;
        static constexpr int kWarehouseVLXxshutMid = 48;
        static constexpr int kWarehouseVLXxshutLower = 50;
        static constexpr int kWarehouseUpperSpeed = 180;
        static constexpr int kWarehouseMidSpeed = 180;
        static constexpr int kWarehouseLowerSpeed = 180;
};

#endif
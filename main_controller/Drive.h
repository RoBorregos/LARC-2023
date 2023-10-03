/*
https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
*/

#ifndef Drive_h
#define Drive_h

#include "Arduino.h"
#include "Constants.h"
#include "Motor.h"
#include "BNO.h"
#include "PID.h"
#include "LineSensor.h"

enum MotorID{
    FrontLeft,
    FrontRight,
    BackLeft,
    BackRight
};

struct Pose2d{
    float x = 0;
    float y = 0;
    float theta = 0;
};

class Drive{
    private:
        constexpr static float loop_time = 10;
        constexpr static float pid_time = 5; 
        Pose2d velocity;
        Pose2d position;
        float angle;
        float setpoint;
        float global_setpoint = 0;
        float error;
        float last_error;
        unsigned long last_time = 0;
        unsigned long speed_last_time = 0;
        bool spin_flag = false;
        bool line_move = false;
        int line_state = 0;
        bool verbose = false;
        float kMinSpeed = Constants::kMinSpeed;
        LineSensor *lineSensor;
        BNO *bno;
        float BNOKP = Constants::kBNOKP;
        float BNOKI = Constants::kBNOKI;
        float BNOKD = Constants::kBNOKD;
        float BNOKImax = Constants::kBNOKImax;
        float BNOKout_min = Constants::kBNOMinAngular;
        float BNOKout_max = Constants::KBNOMaxAngular;
        PID pidControllerBNO;
        float curr_angle_x;
        uint16_t hard_stop_current = 0;
        uint16_t hard_stop_time = Constants::kHardStopTime;
        uint8_t intake_presence = Constants::kIntakePresence;
    public:
        void init(BNO *bno, LineSensor *line_sensor);
        void restart();
        Motor frontLeft;
        Motor frontRight;
        Motor backLeft;
        Motor backRight;
        float getAngleX();
        void setSpeed(float linearX, float linearY, float angularZ);
        void setSpeed(float linearX, float linearY, float angularZ, unsigned long current_time);
        void setSpeedOriented(float linearX, float linearY, float angularZ, unsigned long current_time);
        void setAngle(float angle);
        void setGlobalSetpoint();
        void stop();
        void hardStop();
        void periodicIO(unsigned long current_time);
        void encoderInterrupt(MotorID motorID);
        float getSpeed(MotorID motorID);
        long getTicks(MotorID motorID);
        void setVerbose(bool verbose);
        Pose2d getChassisSpeeds();
        Pose2d getPosition();
        void resetOdometry();
        unsigned long presence_detection_time = 0;
        bool flag = false;
        // current angle the robot should be facing
        float robot_angle = 0;
};

#endif
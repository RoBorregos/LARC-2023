#ifndef RosBridge_h
#define RosBridge_h

#include "Arduino.h"
#include "Drive.h"

class RosBridge{
    private:
        Drive *_drive;

        //////////////////////////////////Velocity Suscriber//////////////////////////////////////
        // Receives velocity commands.
        void velocityCallback(float linearx, float lineary, float angularz);

        // Receives imu angle
        void imuCallback(float angle);

        ////////////////////////////////Odometry Publisher///////////////////
        void getOdometry();

        //Serial
        void readSerial();
        static constexpr uint16_t kWatchdogPeriod = 500;
        
        //Odometry
        float velX = 0.0;
        float velY = 0.0;
        float velTheta = 0.0;
        float posX = 0.0;
        float posY = 0.0;

        // Timers.
        unsigned long odom_timer_ = 0;
        unsigned long watchdog_timer_ = 0;

        // CMD Velocity.
        float linearX_ = 0;
        float linearY_ = 0;
        float angularZ_ = 0;
        float angle_ = 0;

        void executeCommand(uint8_t packet_size, uint8_t command, uint8_t* buffer);
        void writeSerial(bool success, uint8_t* payload, int elements);
    public:
        void init(Drive *drive);
        void spin(unsigned long current_time);
};

#endif
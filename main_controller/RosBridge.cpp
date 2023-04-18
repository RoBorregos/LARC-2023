#include "RosBridge.h"

void RosBridge::init(Drive *drive){
    _drive = drive;
}

//Read serial/////////////////////////////////////////////////////////
void RosBridge::readSerial() {
    static uint8_t buffer[18];
    static uint8_t index = 0;
    static uint8_t packet_size = 0;
    static uint8_t command = 0;
    static uint8_t check_sum = 0;
    
    while (Serial.available()) {
        buffer[index++] = Serial.read();

        // Check packet header
        if (index == 1 && buffer[0] != 0xFF) {
            index = 0;
            packet_size = 0;
            command = 0;
        }
        if (index == 2 && buffer[1] != 0xAA) {
            packet_size = 0;
            command = 0;
            index = 0;
        }
        
        // Read packet size and command
        if (index == 4) {
            packet_size = buffer[2];
            command = buffer[3];
        }
        
        // Check if the entire packet has been received
        if (index == 4 + (packet_size) + 1) {
            check_sum = buffer[index - 1];
            if (check_sum != command + 1) {
                // Checksum error
                index = 0;
                packet_size = 0;
                command = 0;
                continue;
            }
            // Execute the command
            executeCommand(packet_size, command, &buffer[4]);
            
            // Reset index and packet_size
            index = 0;
            packet_size = 0;
        }
    }
}

//////////////////////////////////Velocity Suscriber//////////////////////////////////////
void RosBridge::velocityCallback(double linearx, double lineary, double angularz) {
    linearX_ = linearx;
    linearY_ = lineary;
    angularZ_ = angularz;
    watchdog_timer_ = millis();
}

////////////////////////////////Odometry Publisher//////////////////////////////////////
void RosBridge::getOdometry() {
    Pose2d vel = _drive->getChassisSpeeds();
    Pose2d pos = _drive->getPosition();
    velX = vel.x;
    velY = vel.y;
    velTheta = vel.theta;
    posX = pos.x;
    posY = pos.y;
}

//Execute command/////////////////////////////////////////////////
void RosBridge::executeCommand(uint8_t packet_size, uint8_t command, uint8_t* buffer) {
    switch (command) {
        case 0x04: // Velocity command
        if (packet_size == 12) { // Check packet size
            float x, y, angular;
            memcpy(&x, buffer, sizeof(x));
            memcpy(&y, buffer + sizeof(x), sizeof(y));
            memcpy(&angular, buffer + sizeof(x) + sizeof(y), sizeof(angular));
            velocityCallback(x, y, angular);
            writeSerial(true, nullptr, 0);
        }
        break;
        case 0x13: // Hardware Version 
        if (packet_size == 1) { // Check packet size
            uint32_t version[] = {1};
            writeSerial(true, (uint8_t*)version, sizeof(version));
        }
        break;
        case 0x00: // Baud
        if (packet_size == 1) { // Check packet size
            uint32_t baud[] = {57600};
            writeSerial(true, (uint8_t*)baud, sizeof(baud));
        }
        break;
        case 0x02: // Get Odometry
        if (packet_size == 1) { // Check packet size
            getOdometry();
            int data[] = {velX, velY, velTheta, posX, posY};
            writeSerial(true, (uint8_t*)data, sizeof(data));
        }
        break;
        case 0x41: // Reset IMU
        if (packet_size == 1) { // Check packet size
            writeSerial(true, nullptr, 0);
            bno_->reset();
        }
        break;
        case 0x05: // Send IMU
        if (packet_size == 4) { // Check packet size
            float data[] = {bno_->getYaw(), bno_->getYawVel(), bno_-> getXAccel(), bno_->getYAccel(), bno_->getZAccel()};
            writeSerial(true, (uint8_t*)data, sizeof(data));
        }
        break;
        case 0x15: // Get Emergency Button
        if (packet_size == 1) { // Check packet size
            uint8_t status[] = {digitalRead(emergency_btn_pin)}; // Read the button status
            writeSerial(true, (uint8_t*)status, sizeof(status));
        }
        break;
        case 0x03: // Reset Encoders
        if (packet_size == 1) { // Check packet size
            move_all_->left_motor_.setOdomTicks(0);
            move_all_->right_motor_.setOdomTicks(0);
            writeSerial(true, nullptr, 0);
        }
        break;
        default:
        break;
    }
}

void RosBridge::writeSerial(bool success, uint8_t* payload, int elements) {
  uint8_t ack = success ? 0x00 : 0x01;
  Serial.write(0xFF);
  Serial.write(0xAA);
  Serial.write(sizeof(uint8_t) * elements + 1); // Packet size
  Serial.write(ack); // ACK
  
  // Send payload bytes
  for (size_t i = 0; i < elements; i++) {
    Serial.write(payload[i]);
  }

  Serial.write(0x00); // Footer
  Serial.flush();

}

void RosBridge::spin(unsigned long current_time){
    readSerial();
}
#include "RosBridge.h"

void RosBridge::init(Drive *drive, Intake *intake, Elevator *elevator, Warehouse *warehouse, LineSensor *lineSensor){
    _drive = drive;
    _intake = intake;
    _elevator = elevator;
    _warehouse = warehouse;
    _lineSensor = lineSensor;
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
        if (index == 3 + (packet_size) + 1) {
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
void RosBridge::velocityCallback(float linearx, float lineary, float angularz) {
    linearX_ = linearx;
    linearY_ = lineary;
    angularZ_ = angularz;
    watchdog_timer_ = millis();
}

void RosBridge::imuCallback(float angle) {
    angle_ = angle;
}

void RosBridge::intakeCallback(int command) {
    switch(command){
        case 1:
            _intake->setAction(IntakeActions::Pick);
            break;
        case 2:
            _intake->setAction(IntakeActions::In);
            break;
        case 3:
            _intake->setAction(IntakeActions::Out);
            break;
        case 4:
            _intake->setAction(IntakeActions::Drop);
            break;
        case 5:
            _intake->setAction(IntakeActions::Stop);
            break;
    }
}

void RosBridge::elevatorCallback(int command) {
    ElevatorPosition positions[] = {
        ElevatorPosition::PickPos,
        ElevatorPosition::PickPos,
        ElevatorPosition::FirstIn,
        ElevatorPosition::SecondIn,
        ElevatorPosition::ThirdIn,
        ElevatorPosition::FirstOut,
        ElevatorPosition::SecondOut,
        ElevatorPosition::ThirdOut,
        ElevatorPosition::FirstShelf,
        ElevatorPosition::SecondShelf,
        ElevatorPosition::ThirdShelf
    };
    _elevator->setPosition(positions[command]);
}

void RosBridge::warehouseCallback(int level){
    if( level == 4 )
        _warehouse->reset();
    else
        _warehouse->cubeOut( LevelPosition(level-1), current_time_);
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
        if (packet_size == 13) { // Check packet size
            float x, y, angular;
            memcpy(&x, buffer, sizeof(x));
            memcpy(&y, buffer + sizeof(x), sizeof(y));
            memcpy(&angular, buffer + sizeof(x) + sizeof(y), sizeof(angular));
            velocityCallback(x, y, angular);
            writeSerial(true, nullptr, 0);
            //digitalWrite(13, !digitalRead(13));
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
            uint32_t baud[] = {115200};
            writeSerial(true, (uint8_t*)baud, sizeof(baud));
        }
        break;
        case 0x02: // Get Odometry
        if (packet_size == 1) { // Check packet size
            getOdometry();
            float data[] = {velX, velY, velTheta, posX, posY};
            writeSerial(true, (uint8_t*)data, sizeof(data));
        }
        break;
        case 0x03: // Reset odometry
        if (packet_size == 1) { // Check packet size
            _drive->resetOdometry();
            writeSerial(true, nullptr, 0);
        }
        break;
        case 0x05: // Send IMU
        if (packet_size == 5) { // Check packet size
            float angle;
            memcpy(&angle, buffer, sizeof(angle));
            imuCallback(angle);
            writeSerial(true, nullptr, 0);
            //digitalWrite(13, !digitalRead(13));
        }
        break;
        case 0x06: // Send Intake
        if (packet_size == 5) { // Check packet size
            int command;
            memcpy(&command, buffer, sizeof(command));
            intakeCallback(command);
            writeSerial(true, nullptr, 0);
        }
        break;
        case 0x08: // Send Elevator}
        if (packet_size == 5) { // Check packet size
            int command;
            memcpy(&command, buffer, sizeof(command));
            elevatorCallback(command);
            writeSerial(true, nullptr, 0);
        }
        break;
        case 0x0A: // Send Warehouse 
        if (packet_size == 5) { // Check packet size
            int level;
            memcpy(&level, buffer, sizeof(level));
            warehouseCallback(level);
            writeSerial(true, nullptr, 0);
        }
        break;
        case 0x0B: // Send Line Sensor
        if (packet_size == 1) { // Check packet size
            char data[] = {
                _lineSensor->lineDetected(SensorID::FrontLeft1)? '1' : '0',
                _lineSensor->lineDetected(SensorID::FrontLeft2)? '1' : '0',
                _lineSensor->lineDetected(SensorID::FrontRight1)? '1' : '0',
                _lineSensor->lineDetected(SensorID::FrontRight2)? '1' : '0',
                _lineSensor->lineDetected(SensorID::BackLeft1)? '1' : '0',
                _lineSensor->lineDetected(SensorID::BackLeft2)? '1' : '0',
                _lineSensor->lineDetected(SensorID::BackRight1)? '1' : '0',
                _lineSensor->lineDetected(SensorID::BackRight2)? '1' : '0'
            };
            writeSerial(true, (uint8_t*)data, sizeof(data));
        }
        break;
        case 0x0D: // Receive set global setpoint
        if(packet_size == 1){
            _drive->setGlobalSetpoint();
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
    current_time_ = current_time;

    readSerial();
    /*if((millis() - watchdog_timer_) > kWatchdogPeriod) {
        linearX_ = 0.0;
        linearY_ = 0.0;
        angularZ_ = 0.0;        
        watchdog_timer_ = millis();
    }*/
    _drive->setSpeedOriented(linearX_, linearY_, angularZ_, current_time_);
    _drive->setAngle(angle_);
}
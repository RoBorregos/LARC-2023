#include "Kinematics.h"

Kinematics::Kinematics(float wheelTrack, float wheelBase){
    float xPos = wheelTrack / 2;
    float yPos = wheelBase / 2;

    this->inverseKinematics[0][0] = 1;
    this->inverseKinematics[0][1] = -1;
    this->inverseKinematics[0][2] = -xPos - yPos;
    this->inverseKinematics[1][0] = 1;
    this->inverseKinematics[1][1] = 1;
    this->inverseKinematics[1][2] = xPos + yPos;
    this->inverseKinematics[2][0] = 1;
    this->inverseKinematics[2][1] = 1;
    this->inverseKinematics[2][2] = -xPos - yPos;
    this->inverseKinematics[3][0] = 1;
    this->inverseKinematics[3][1] = -1;
    this->inverseKinematics[3][2] = xPos + yPos;
}

Kinematics::WheelSpeeds Kinematics::getWheelSpeeds(float linearX, float linearY, float angularZ){
    input[0][0] = linearX;
    input[1][0] = linearY;
    input[2][0] = angularZ;
    
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 3; j++){
            output[i][0] += inverseKinematics[i][j] * input[j][0];
        }
    }
    
    WheelSpeeds wheelSpeeds;
    wheelSpeeds.frontLeft = output[0][0];
    wheelSpeeds.frontRight = output[1][0];
    wheelSpeeds.backLeft = output[2][0];
    wheelSpeeds.backRight = output[3][0];
    return wheelSpeeds;
}

/*float[][] Kinematics::matrixMult(float[][] matrixA, float[][] matrixB){
    float[][] output = new float[matrixA.length][matrixB[0].length];
    for(int i = 0; i < matrixA.length; i++){
        for(int j = 0; j < matrixB[0].length; j++){
            for(int k = 0; k < matrixA[0].length; k++){
                output[i][j] += matrixA[i][k] * matrixB[k][j];
            }
        }
    }
    return output;
}*/
/*
 * mpu6050.h
 *
 *  Created on: Nov 10, 2022
 *      Author: Lenovo
 */

#ifndef INC_GY521_H_
#define INC_GY521_H_

#endif /* INC_GY521_H_ */

#include <stdint.h>


// MPU6050 structure
typedef struct
{

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    float Ax;
    float Ay;
    float Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    float Gx;
    float Gy;
    float Gz;

    float Temperature;

    float KalmanAngleX;
    float KalmanAngleY;
} MPU6050_t;

// Kalman structure
typedef struct
{
    float Q_angle;
    float Q_bias;
    float R_measure;
    float angle;
    float bias;
    float P[2][2];
} Kalman_t;

uint8_t MPU6050_Init();

void MPU6050_Read_Accel(MPU6050_t *DataStruct);

void MPU6050_Read_Gyro( MPU6050_t *DataStruct);

void MPU6050_Read_Temp( MPU6050_t *DataStruct);

void MPU6050_Read_All(MPU6050_t *DataStruct);

float Kalman_getAngle(Kalman_t *Kalman, float newAngle, float newRate, float dt);

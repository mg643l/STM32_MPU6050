#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define MPU6050_ADDR (0x68 << 1)

typedef struct {
    float roll;
    float pitch;
    float yaw;
} EulerAngles;

extern I2C_HandleTypeDef hi2c1;

void MPU6050_Init(void);
void MPU6050_Read_Accel(float *ax, float *ay, float *az);
void MPU6050_Read_Gyro(float *gx, float *gy, float *gz);
void Madgwick_Filter(float gx, float gy, float gz, float ax, float ay, float az);
void Euler_Conversion(EulerAngles *angles);

#endif

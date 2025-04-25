#include "MPU6050.h"
#include <math.h>

// MPU6050 Register Addresses
#define PWR_MNGT_1_ADDR 0x6B
#define SMPLRT_ADDR 0x19
#define GYRO_CONFIG_ADDR 0x1B
#define ACCEL_CONFIG_ADDR 0x1C

// Madgwick Filter Constants
#define deltat 0.02f
#define gyroMeasError (3.14159265358979f * (5.0f / 180.0f))
#define beta sqrtf(3.0f / 4.0f) * gyroMeasError

// Quaternion state
static float SEq_1 = 1.0f, SEq_2 = 0.0f, SEq_3 = 0.0f, SEq_4 = 0.0f;

void MPU6050_Init(void) {
    uint8_t value = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MNGT_1_ADDR, 1, &value, 1, 1000);
    value = 0x07;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_ADDR, 1, &value, 1, 1000);
    value = 0x10; // Gyro ±1000 dps
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_ADDR, 1, &value, 1, 1000);
    value = 0x18; // Accel ±16g
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_ADDR, 1, &value, 1, 1000);
}

void MPU6050_Read_Accel(float *ax, float *ay, float *az) {
    uint8_t Rec_Data[6];
    int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, Rec_Data, 6, 1000);
    Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    *ax = (float)Accel_X_RAW / 208.837880418f;
    *ay = (float)Accel_Y_RAW / 208.837880418f;
    *az = (float)Accel_Z_RAW / 208.837880418f;
}

void MPU6050_Read_Gyro(float *gx, float *gy, float *gz) {
    uint8_t Rec_Data[6];
    int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x43, 1, Rec_Data, 6, 1000);
    Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    *gx = (float)Gyro_X_RAW / 32.8f * (M_PI / 180.0f);
    *gy = (float)Gyro_Y_RAW / 32.8f * (M_PI / 180.0f);
    *gz = (float)Gyro_Z_RAW / 32.8f * (M_PI / 180.0f);
}

void Madgwick_Filter(float gx, float gy, float gz, float ax, float ay, float az) {
    float halfSEq_1 = 0.5f * SEq_1;
    float halfSEq_2 = 0.5f * SEq_2;
    float halfSEq_3 = 0.5f * SEq_3;
    float halfSEq_4 = 0.5f * SEq_4;
    float twoSEq_1 = 2.0f * SEq_1;
    float twoSEq_2 = 2.0f * SEq_2;
    float twoSEq_3 = 2.0f * SEq_3;

    float norm = sqrtf(ax * ax + ay * ay + az * az);
    ax /= norm;
    ay /= norm;
    az /= norm;

    float f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - ax;
    float f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - ay;
    float f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - az;
    float J_11or24 = twoSEq_3;
    float J_12or23 = 2.0f * SEq_4;
    float J_13or22 = twoSEq_1;
    float J_14or21 = twoSEq_2;
    float J_32 = 2.0f * J_14or21;
    float J_33 = 2.0f * J_11or24;

    float SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
    float SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
    float SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
    float SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;

    norm = sqrtf(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 +
                SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
    SEqHatDot_1 /= norm;
    SEqHatDot_2 /= norm;
    SEqHatDot_3 /= norm;
    SEqHatDot_4 /= norm;

    float SEqDot_omega_1 = -halfSEq_2 * gx - halfSEq_3 * gy - halfSEq_4 * gz;
    float SEqDot_omega_2 = halfSEq_1 * gx + halfSEq_3 * gz - halfSEq_4 * gy;
    float SEqDot_omega_3 = halfSEq_1 * gy - halfSEq_2 * gz + halfSEq_4 * gx;
    float SEqDot_omega_4 = halfSEq_1 * gz + halfSEq_2 * gy - halfSEq_3 * gx;

    SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
    SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
    SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
    SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;

    norm = sqrtf(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
    SEq_1 /= norm;
    SEq_2 /= norm;
    SEq_3 /= norm;
    SEq_4 /= norm;
}

void Euler_Conversion(EulerAngles *angles) {
    angles->roll = atan2f(2.0f * (SEq_1 * SEq_2 + SEq_3 * SEq_4), 1.0f - 2.0f * (SEq_2 * SEq_2 + SEq_3 * SEq_3));

    float sinp = 2.0f * (SEq_1 * SEq_3 - SEq_4 * SEq_2);
    if (fabsf(sinp) >= 1)
        angles->pitch = copysignf(M_PI / 2, sinp);
    else
        angles->pitch = asinf(sinp);

    angles->yaw = atan2f(2.0f * (SEq_1 * SEq_4 + SEq_2 * SEq_3),
                      1.0f - 2.0f * (SEq_3 * SEq_3 + SEq_4 * SEq_4));

    angles->roll *= 180.0f / M_PI;
    angles->pitch *= 180.0f / M_PI;
    angles->yaw *= 180.0f / M_PI;
}

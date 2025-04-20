#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <math.h>

// MPU6050 Definitions
#define MPU6050_ADDR (0x68 << 1)  // 0xD0
#define PWR_MNGT_1_ADDR 0x6B
#define SMPLRT_ADDR 0x19
#define GYRO_CONFIG_ADDR 0x1B
#define ACCEL_CONFIG_ADDR 0x1C

// Madgwick Filter Constants
#define deltat 0.02f // sampling period
#define gyroMeasError (3.14159265358979f * (5.0f / 180.0f)) // gyroscope measurement error in rad/s
#define beta sqrtf(3.0f / 4.0f) * gyroMeasError // compute beta

// Global variables for Madgwick filter
float SEq_1 = 1.0f, SEq_2 = 0.0f, SEq_3 = 0.0f, SEq_4 = 0.0f;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} EulerAngles;

I2C_HandleTypeDef hi2c1;
extern USBD_HandleTypeDef hUsbDeviceFS;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

// Initialisation setting to be written to IMU registers
void MPU6050_Init() {

	// Power management register, wakes IMU from sleep mode
    uint8_t value = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MNGT_1_ADDR, 1, &value, 1, 1000);

    // Sample rate register, sets rate for IMU to sample at
    value = 0x07;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_ADDR, 1, &value, 1, 1000);

    // Sets degrees per second range for gyro
    value = 0x10; // 1000 dps
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_ADDR, 1, &value, 1, 1000);

    // Sets g range for gyro
    value = 0x18; // 16g
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_ADDR, 1, &value, 1, 1000);
}

// Read acceleration data from MPU6050
void MPU6050_Read_Accel(float *ax, float *ay, float *az) {
    uint8_t Rec_Data[6];
    int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, Rec_Data, 6, 1000);

    // Combine H and L bytes
    Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    // Calculate compensation for resolution
    *ax = (float)Accel_X_RAW / 208.837880418f;
    *ay = (float)Accel_Y_RAW / 208.837880418f;
    *az = (float)Accel_Z_RAW / 208.837880418f;
}

// Read gyroscope data from MPU6050
void MPU6050_Read_Gyro(float *gx, float *gy, float *gz) {
    uint8_t Rec_Data[6];
    int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x43, 1, Rec_Data, 6, 1000);

    // Combine H and L bytes
    Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    // Convert to rad/s and calculate compensation for resolution
    *gx = (float)Gyro_X_RAW / 32.8f * (M_PI / 180.0f);
    *gy = (float)Gyro_Y_RAW / 32.8f * (M_PI / 180.0f);
    *gz = (float)Gyro_Z_RAW / 32.8f * (M_PI / 180.0f);
}

// Sensor-fusion algorithm to calculate orientation
void Madgwick_Filter(float gx, float gy, float gz, float ax, float ay, float az) {
    float halfSEq_1 = 0.5f * SEq_1;
    float halfSEq_2 = 0.5f * SEq_2;
    float halfSEq_3 = 0.5f * SEq_3;
    float halfSEq_4 = 0.5f * SEq_4;
    float twoSEq_1 = 2.0f * SEq_1;
    float twoSEq_2 = 2.0f * SEq_2;
    float twoSEq_3 = 2.0f * SEq_3;

    // Normalise accelerometer measurement
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    ax /= norm;
    ay /= norm;
    az /= norm;

    // Compute objective function and Jacobian
    float f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - ax;
    float f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - ay;
    float f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - az;
    float J_11or24 = twoSEq_3;
    float J_12or23 = 2.0f * SEq_4;
    float J_13or22 = twoSEq_1;
    float J_14or21 = twoSEq_2;
    float J_32 = 2.0f * J_14or21;
    float J_33 = 2.0f * J_11or24;

    // Compute gradient
    float SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
    float SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
    float SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
    float SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;

    // Normalise gradient
    norm = sqrtf(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 +
                SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
    SEqHatDot_1 /= norm;
    SEqHatDot_2 /= norm;
    SEqHatDot_3 /= norm;
    SEqHatDot_4 /= norm;

    // Compute quaternion derivative from gyro
    float SEqDot_omega_1 = -halfSEq_2 * gx - halfSEq_3 * gy - halfSEq_4 * gz;
    float SEqDot_omega_2 = halfSEq_1 * gx + halfSEq_3 * gz - halfSEq_4 * gy;
    float SEqDot_omega_3 = halfSEq_1 * gy - halfSEq_2 * gz + halfSEq_4 * gx;
    float SEqDot_omega_4 = halfSEq_1 * gz + halfSEq_2 * gy - halfSEq_3 * gx;

    // Integrate estimated quaternion derivative
    SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
    SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
    SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
    SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;

    // Normalise quaternion
    norm = sqrtf(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
    SEq_1 /= norm;
    SEq_2 /= norm;
    SEq_3 /= norm;
    SEq_4 /= norm;
}

// Convert quaternion to roll, pitch and yaw
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

// Broadcast the data over USB to be viewed on PuTTY
void Serial_Send_Data(EulerAngles *angles) {
    char buffer[64];
    int length = snprintf(buffer, sizeof(buffer),
        "Orientation: Roll=%.2f° Pitch=%.2f° Yaw=%.2f°\r\n",
        angles->roll, angles->pitch, angles->yaw);

    while (CDC_Transmit_FS((uint8_t*)buffer, length) == USBD_BUSY) {
        HAL_Delay(1);
    }
}

int main(void) {
	// MCU and sensor initialisation
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USB_DEVICE_Init();
    MPU6050_Init();

    float ax, ay, az;
    float gx, gy, gz;
    EulerAngles orientation;

    // Loop forever
    while (1) {
    	// Read IMU data
        MPU6050_Read_Accel(&ax, &ay, &az);
        MPU6050_Read_Gyro(&gx, &gy, &gz);

        // Apply filter and conversion to get current orientation
        Madgwick_Filter(gx, gy, gz, ax, ay, az);
        Euler_Conversion(&orientation);

        // If USB is connected, send serial data
        if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
            Serial_Send_Data(&orientation);
        }

        HAL_Delay(20);
    }
}

// Auto-generated code
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif

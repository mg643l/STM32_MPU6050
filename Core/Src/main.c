#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "MPU6050.h"
#include <math.h>

// Global variables for Madgwick filter
extern float SEq_1;
extern float SEq_2;
extern float SEq_3;
extern float SEq_4;


I2C_HandleTypeDef hi2c1;
extern USBD_HandleTypeDef hUsbDeviceFS;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

// Broadcast the data over USB to be viewed on PuTTY
void Serial_Send_Quaternion(float q0, float q1, float q2, float q3) {
    char buffer[128];
    int length = snprintf(buffer, sizeof(buffer),
        "Quaternion: q0=%.4f q1=%.4f q2=%.4f q3=%.4f\r\n",
        q0, q1, q2, q3);

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
    float q0, q1, q2, q3;
    EulerAngles orientation;

    // Loop forever
    while (1) {
    	// Read IMU data
        MPU6050_Read_Accel(&ax, &ay, &az);
        MPU6050_Read_Gyro(&gx, &gy, &gz);

        // Apply filter and conversion to get current orientation
        Madgwick_Filter(gx, gy, gz, ax, ay, az);
        Euler_Conversion(&orientation);

        // Get the quaternion values
        q0 = SEq_1;
        q1 = SEq_2;
        q2 = SEq_3;
        q3 = SEq_4;

        // If USB is connected, send quaternion data
        if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
        	Serial_Send_Quaternion(q0, q1, q2, q3);
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

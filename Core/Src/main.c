#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#define MPU6050_ADDR (0x68 << 1)  // 0xD0
#define PWR_MNGT_1_ADDR 0x6B
#define SMPLRT_ADDR 0x19
#define GYRO_CONFIG_ADDR 0x1B
#define ACCEL_CONFIG_ADDR 0x1C

typedef struct {
	float Ax;
	float Ay;
	float Az;
} MPU6050_Accel;

typedef struct {
	float Gx;
	float Gy;
	float Gz;
} MPU6050_Gyro;

I2C_HandleTypeDef hi2c1;
extern USBD_HandleTypeDef hUsbDeviceFS;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

void MPU6050_Init(){
	uint8_t value = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MNGT_1_ADDR, 1, &value, 1, 1000);

	value = 0x07;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_ADDR, 1, &value, 1, 1000);

	value = 0x10; // 1000 dps
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_ADDR, 1, &value, 1, 1000);

	value = 0x18; // 16g
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_ADDR, 1, &value, 1, 1000);
}

void MPU6050_Read_Accel(MPU6050_Accel *accel){
	uint8_t Rec_Data[6];
	int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;

	// Read 6 BYTES starting from ACCEL_XOUT_H (0x3B)
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, Rec_Data, 6, 1000);

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

	accel->Ax = (float)Accel_X_RAW / 208.837880418;
	accel->Ay = (float)Accel_Y_RAW / 208.837880418;
	accel->Az = (float)Accel_Z_RAW / 208.837880418;
}

void MPU6050_Read_Gyro(MPU6050_Gyro *gyro){
	uint8_t Rec_Data[6];
	int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;

	// Read 6 BYTES starting from ACCEL_XOUT_H (0x3B)
	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, 0x43, 1, Rec_Data, 6, 1000);

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

	gyro->Gx = (float)Gyro_X_RAW / 32.8;
	gyro->Gy = (float)Gyro_Y_RAW / 32.8;
	gyro->Gz = (float)Gyro_Z_RAW / 32.8;
}

void serialSend(float Ax, float Ay, float Az, float Gx, float Gy, float Gz) {
    char buffer1[64];
    int length1 = snprintf(buffer1, sizeof(buffer1), "Ax: %.2f | Ay: %.2f | Az: %.2f\r\n", Ax, Ay, Az);

    while (CDC_Transmit_FS((uint8_t*)buffer1, length1) == USBD_BUSY) {
    	HAL_Delay(1);
    }

    char buffer2[64];
        int length2 = snprintf(buffer2, sizeof(buffer2), "Gx: %.2f | Gy: %.2f | Gz: %.2f\r\n", Gx, Gy, Gz);

        while (CDC_Transmit_FS((uint8_t*)buffer2, length2) == USBD_BUSY) {
        	HAL_Delay(1);
        }
}


int main(void)
{
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_USB_DEVICE_Init();
	MPU6050_Init();

	MPU6050_Accel accel;
	MPU6050_Gyro gyro;

	while (1)
	{
		MPU6050_Read_Accel(&accel);
		MPU6050_Read_Gyro(&gyro);

		if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
		    serialSend(accel.Ax, accel.Ay, accel.Az, gyro.Gx, gyro.Gy, gyro.Gz);
		}

		HAL_Delay(20);
  }
}

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

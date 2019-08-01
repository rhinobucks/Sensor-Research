/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "vl53l1_api.h"
#include "dwt_stm32_delay.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// I2C addresses of GPIO expanders on the X-NUCLEO-53L1A1
#define EXPANDER_1_ADDR 0x84 // 0x42 << 1
#define EXPANDER_2_ADDR 0x86 // 0x43 << 1

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART2_UART_Init(void);
void Error_Handler(void);
float hcsr04_read (void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

int k=0;
int diff=0;
int local_time=0;
float sensor_time;
float distance;

float hcsr04_read (void)
{

	local_time=0;
	HAL_GPIO_WritePin(GPIOA, TRIG_Pin, GPIO_PIN_RESET);  // pull the TRIG pin HIGH
	DWT_Delay_us(2); //wait for 2 us


	HAL_GPIO_WritePin(GPIOA, TRIG_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	DWT_Delay_us(10); //wait for 10 us

	HAL_GPIO_WritePin(GPIOA, TRIG_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low

	// read the time for which the pin is high
	//DWT_Delay_us(2); //wait for 10 us
	while (!(HAL_GPIO_ReadPin(GPIOA, ECHO_Pin)))// wait for the ECHO pin to go high
	{
		//HAL_GPIO_WritePin(GPIOA, LD2_Pin,GPIO_PIN_RESET);
	}

	while (HAL_GPIO_ReadPin(GPIOA, ECHO_Pin))    // while the pin is high
	 {
		//HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);
		local_time++;   // measure time for which the pin is high
		DWT_Delay_us(1); //wait for 1 us

	 }
	return local_time*2;
}







/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */





int main(void)
{
  /* USER CODE BEGIN 1 */

	uint8_t buff[100];
	VL53L1_RangingMeasurementData_t RangingData;
	VL53L1_Dev_t  vl53l1_c; // center module
	VL53L1_DEV    Dev = &vl53l1_c;
	VL53L1_CalibrationData_t OpCent;
	VL53L1_UserRoi_t roiConfig;


	uint8_t rx[1];
	char str1[100]="1234567890";
	//char str2[100]="1234567890";




  /* USER CODE END 1 */


  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  HAL_IncTick();

  /* USER CODE BEGIN Init */

   DWT_Delay_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */


  // initialize vl53l1x communication parameters
  Dev->I2cHandle = &hi2c1;
  Dev->I2cDevAddr = 0x52;

  /*** Initialize GPIO expanders ***/
  // Unused GPIO should be configured as outputs to minimize the power consumption
  buff[0] = 0x14; // GPDR (GPIO set direction register)
  buff[1] = 0xFF; // GPIO_0 - GPIO_7
  buff[2] = 0xFF; // GPIO_8 - GPIO_15
  HAL_I2C_Master_Transmit( &hi2c1, EXPANDER_1_ADDR, buff, 3, 0xFFFF );
  HAL_I2C_Master_Transmit( &hi2c1, EXPANDER_2_ADDR, buff, 3, 0xFFFF );

  // clear XSHUT (disable center module) -> expander 1, GPIO_15
  buff[0] = 0x13; // GPSR + 1 ( GPIO set pin state register)
  HAL_I2C_Master_Transmit( &hi2c1, EXPANDER_1_ADDR, buff, 1, 0xFFFF );
  HAL_I2C_Master_Receive( &hi2c1, EXPANDER_1_ADDR, buff, 1, 0xFFFF );
  buff[1] = buff[0] & ~( 1 << ( 15 - 8 ) ); // clear GPIO_15
  buff[0] = 0x13; // GPSR + 1 ( GPIO set pin state register)
  HAL_I2C_Master_Transmit( &hi2c1, EXPANDER_1_ADDR, buff, 2, 0xFFFF );

  HAL_Delay( 2 ); // 2ms reset time

  // set XSHUT (enable center module) -> expander 1, GPIO_15
  buff[0] = 0x13; // GPSR + 1 ( GPIO set pin state)
  HAL_I2C_Master_Transmit( &hi2c1, EXPANDER_1_ADDR, buff, 1, 0xFFFF );
  HAL_I2C_Master_Receive( &hi2c1, EXPANDER_1_ADDR, buff, 1, 0xFFFF );
  buff[1] = buff[0] | ( 1 << ( 15 - 8 ) ); // set GPIO_15
  buff[0] = 0x13; // GPSR + 1 ( GPIO set pin state register)
  HAL_I2C_Master_Transmit( &hi2c1, EXPANDER_1_ADDR, buff, 2, 0xFFFF );

  HAL_Delay( 2 );

  /*** VL53L1X Initialization ***/
  VL53L1_WaitDeviceBooted( Dev );
  VL53L1_DataInit( Dev );
  VL53L1_StaticInit( Dev );
  VL53L1_SetDistanceMode( Dev, VL53L1_DISTANCEMODE_LONG ); //default long
  VL53L1_SetMeasurementTimingBudgetMicroSeconds( Dev, 50000 ); //default 50000
  VL53L1_SetInterMeasurementPeriodMilliSeconds( Dev, 500 );  //default 500
  VL53L1_StartMeasurement( Dev );

  //VL53L1_SetUserROI();

  VL53L1_GetCalibrationData(Dev, &OpCent); //Calibration data


  /* USER CODE END 2 */

  /* Infinite loop */
   /* USER CODE BEGIN WHILE */



  HAL_GPIO_WritePin(GPIOA, TRIG_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low
   while (1)
   {

	   __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_RXNE);
	   //HAL_UART_Receive( &huart2, rx, strlen( (char*)rx ),0x1000); //Blocking mode
	   HAL_UART_Receive_IT( &huart2, rx, strlen( (char*)rx )); //Non-blocking mode



	   if(*rx=='A') //Default state 16 x 16
	   {
		   VL53L1_StopMeasurement( Dev );
		   roiConfig.TopLeftX = 0;
		   roiConfig.TopLeftY = 15;
		   roiConfig.BotRightX = 15;
		   roiConfig.BotRightY = 0;
		   VL53L1_SetUserROI(Dev, &roiConfig);
		   VL53L1_StartMeasurement( Dev );

	   }

	   else if(*rx=='B')
	   {




	   }





	   //OPTICAL SENSOR: USER CODE BEGIN 3
	  			 	    VL53L1_WaitMeasurementDataReady( Dev );

	  			 	    VL53L1_GetRangingMeasurementData( Dev, &RangingData );

	  			   	   //OPTICAL SENSOR: USER CODE END 3



	  				   //ULTRASONIC SENSOR: USER CODE BEGIN 3

	  					  sensor_time = hcsr04_read();
	  					  distance  = sensor_time * (.343/2); // meters per second * seconds
	  					  k=distance;






	  					  //sprintf( (char*)buff, "Ultron: %d, ToF: %d, Difference: %d \n\r", k, RangingData.RangeMilliMeter, diff ); //Printed labels

	  					  //sprintf( (char*)buff, "%d, %d \n\r", k, RangingData.RangeMilliMeter);  //No printed labels
	  					  sprintf(str1, "%.*f",8,distance);

	  					  sprintf( (char*)buff, "%.8s, %d, %d, %.4f, %.4f, %d, %d, %f  \n\r",str1,RangingData.RangeMilliMeter,RangingData.RangeStatus,( RangingData.SignalRateRtnMegaCps / 65536.0 ),(RangingData.AmbientRateRtnMegaCps / 65336.0 ),(RangingData.StreamCount),(RangingData.EffectiveSpadRtnCount / 256),( RangingData.SigmaMilliMeter / 65536.0 ));  //No printed labels
	  					 // sprintf(str2, "%d",OpCent.optical_centre);
	  					  //sprintf((char*)buff, "%s\n\r", str2);


	  					  HAL_UART_Transmit( &huart2, buff, strlen( (char*)buff ), 0xFFFF );
	  					  VL53L1_ClearInterruptAndStartMeasurement( Dev );

	  					  //Ultrasonic sensor range, Optical sensor range

	  					  __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_RXNE);




	  					 //ULTRASONIC SENSOR: USER CODE END 3





   }





}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  // Initializes the CPU, AHB and APB busses clocks
  /*
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 288;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  */


  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 288;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
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
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TRIG_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TRIG_Pin LD2_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ECHO_Pin */
  GPIO_InitStruct.Pin = ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
PUTCHAR_PROTOTYPE
{
 /* Place your implementation of fputc here */
 /* e.g. write a character to the USART2 and Loop until the end of transmission */
 HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

return ch;
}
/* USER CODE END 4 */


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

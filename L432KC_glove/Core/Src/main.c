/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LiquidCrystal.h"
#include "VL53L4CD_api.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define accel_addr (0b0011001 << 1)
#define CTRL_REG1_A 0x20
#define lower_x (0x28 | 0x80)
#define ACCELEROMETER_UART_ID 0

// Initial offsets
int16_t accel_xoffset = 0;
int16_t accel_yoffset = 0;
int16_t accel_zoffset = -19000;
uint8_t calibrateFlag = 0;

#define ADC_BUFFER_LENGTH 2
uint16_t adc_buffer[ADC_BUFFER_LENGTH];

/*
 * Glove modes
 * 0: Control car
 * 1: Control arm
 */
uint8_t glove_mode = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int status;
volatile int IntCount;
uint8_t p_data_ready;
uint16_t dev, sensor_id;
VL53L4CD_ResultsData_t results;		/* Results data from VL53L4CD */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void get_data_by_polling(uint16_t dev);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == Mode_Pin)
	{
		glove_mode = (glove_mode == 0) ? 1 : 0;
	}
	else if (GPIO_Pin == Calibrate_Pin)
	{
		calibrateFlag = 1;
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C3_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  dev = 0x52;
  status = VL53L4CD_GetSensorId(dev, &sensor_id);
  	if(status || (sensor_id != 0xebaa))
  	{
  		return 2;
  	}
  	status = VL53L4CD_SensorInit(dev);
    // Initialize variables
    HAL_StatusTypeDef ret;
  	uint8_t buf[50];

  	int16_t x_val, y_val, z_val;
	char x_send[8];
	char y_send[8];
	char z_send[8];
	char adc_str[8];

	// TODO: Enable TOF
	char tofStr[8];

	// Enable accelerometer
	buf[0] = CTRL_REG1_A;
	buf[1] = 0b10010111;

	ret = HAL_I2C_Master_Transmit(&hi2c3, accel_addr, buf, 2, HAL_MAX_DELAY);
	if (ret != HAL_OK) { return 1; } // return with error code 1

	// Enable screen
	LiquidCrystal_init(0);
	status = VL53L4CD_StartRanging(dev);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_LENGTH);
  while (1)
  {
	HAL_Delay(100); // TODO: Do we need this delay?

	char final_send[40];
	final_send[0] = '#';

	char modeStr[2];
	modeStr[0] = (glove_mode == 0) ? '0' : '1';
	modeStr[1] = '\0';

	final_send[1] = modeStr[0];

	//Print to LCD Screen
	LiquidCrystal_clear();
	LiquidCrystal_print("Mode: ");
	LiquidCrystal_print(modeStr);

	// Retrieve accelerometer values
	buf[0] = lower_x;
	ret = HAL_I2C_Master_Transmit(&hi2c3, accel_addr, buf, 1, HAL_MAX_DELAY);
	if (ret != HAL_OK) { continue; }
	ret = HAL_I2C_Master_Receive(&hi2c3, accel_addr, buf, 6, HAL_MAX_DELAY);
	if (ret != HAL_OK) { continue; }

	x_val = (buf[1] << 8) | buf[0];
	y_val = (buf[3] << 8) | buf[2];
	z_val = (buf[5] << 8) | buf[4];

	// If need to calibrate
	if (calibrateFlag == 1)
	{
		// Update offsets
		accel_xoffset = x_val;
		accel_yoffset = y_val;
		accel_zoffset = z_val;

		// TODO: Calibrate TOF Sensor

		// TODO: Should we calibrate the flex sensor value too?

		// Reset calibrate flag
		calibrateFlag = 0;
	}

	// Apply calibrated offset
	x_val -= accel_xoffset;
	y_val -= accel_yoffset;
	z_val -= accel_zoffset;
	z_val += -19000;

	if (glove_mode == 0)
	{
		sprintf(x_send, "%d", x_val);
		sprintf(y_send, "%d", y_val);
		sprintf(z_send, "%d", z_val);

		// Package accelerometer values
		memcpy(final_send+2, x_send, 8);
		memcpy(final_send+10, y_send, 8);
		memcpy(final_send+18, z_send, 8);

		// Start Character (1) + Glove Mode (1) + Accelerometer values (8+8+8=24) = 26 bytes
		HAL_UART_Transmit(&huart2, (const uint8_t *)final_send, 26, 0xFFFF);

		//Print to LCD Screen
		LiquidCrystal_setCursor(0, 1);
		LiquidCrystal_print("x:");
		LiquidCrystal_print(x_send);
		LiquidCrystal_print(" | ");
		LiquidCrystal_print("y:");
		LiquidCrystal_print(y_send);
//		LiquidCrystal_print("z:");
//		LiquidCrystal_print(z_send);
//		LiquidCrystal_print(" | ");
//		LiquidCrystal_print(adc_str);
	}
	else if (glove_mode == 1)
	{
		// TODO: Implement TOF
		status = VL53L4CD_CheckForDataReady(dev, &p_data_ready);

		if(p_data_ready) {
		/* Read measured distance. RangeStatus = 0 means valid data */
		VL53L4CD_GetResult(dev, &results);

		sprintf(tofStr, "%d", (int)results.distance_mm);


		/* (Mandatory) Clear HW interrupt to restart measurements */
			VL53L4CD_ClearInterrupt(dev);
		}else{
						HAL_Delay(5);
		}

		// Need to convert tof distance to tofStr
		memcpy(final_send+2, tofStr, 8);
		// Retrieve ADC values
		// TODO: Right now only channel 6 works. Channel 11 has hardware issue probably.
		uint16_t ADC_ch6 = adc_buffer[0];  // Channel 6 (Rank 1)
		uint16_t ADC_ch11 = adc_buffer[1]; // Channel 11 (Rank 2)

		// Transmit ADC (flex sensor) values
		sprintf(adc_str, "%d", ADC_ch6);
		memcpy(final_send+10, adc_str, 8);

		// TODO: Send TOF values

		// Start Character (1) + Glove Mode (1) + TOF Values (8) + Flex sensor values (8) = 18 bytes
		HAL_UART_Transmit(&huart2, (const uint8_t *)final_send, 18, 0xFFFF);

		LiquidCrystal_setCursor(0, 1);
		LiquidCrystal_print("ADC:");
		LiquidCrystal_print(adc_str);
		LiquidCrystal_print("|");
		LiquidCrystal_print("ToF:");
		LiquidCrystal_print(tofStr);
	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV32;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c3.Init.Timing = 0x00B07CB4;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_D4_Pin|LCD_D1_Pin|LCD_EN_Pin|LCD_D0_Pin
                          |LCD_D2_Pin|LCD_D3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RW_Pin|LCD_D6_Pin|LCD_D7_Pin|LCD_RS_Pin
                          |LCD_D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Mode_Pin Calibrate_Pin */
  GPIO_InitStruct.Pin = Mode_Pin|Calibrate_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_D4_Pin LCD_D1_Pin LCD_EN_Pin LCD_D0_Pin
                           LCD_D2_Pin LCD_D3_Pin */
  GPIO_InitStruct.Pin = LCD_D4_Pin|LCD_D1_Pin|LCD_EN_Pin|LCD_D0_Pin
                          |LCD_D2_Pin|LCD_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RW_Pin LCD_D6_Pin LCD_D7_Pin LCD_RS_Pin
                           LCD_D5_Pin */
  GPIO_InitStruct.Pin = LCD_RW_Pin|LCD_D6_Pin|LCD_D7_Pin|LCD_RS_Pin
                          |LCD_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

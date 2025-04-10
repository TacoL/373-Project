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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 * MODES:
 * 0: Glove
 * 1: Vehicle
 * 2: Arm (Testing)
 */
#define MODE 2

// Add libraries as needed
	#include <stdlib.h>
	#include <math.h>
	#include <string.h>

	#include "ArmDriver.h"


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//#define MESSAGE_ID_POS 30
//#define NUM_PACKETS_POS 26
//#define CURRENT_PACKET_POS 22
//#define DATA_POS 6
//
//#define MESSAGE_ID_MASK (0b11 << MESSAGE_ID_POS)
//#define NUM_PACKETS_MASK (0xF << NUM_PACKETS_POS)
//#define CURRENT_PACKET_MASK (0xF << CURRENT_PACKET_POS)
//#define DATA_MASK (0xFFFF << DATA_POS)


// package data so that UART can receive consistent packages
// Data will be packaged into a 32-bit packets where:
// Bits 31-30: Message ID
// Bits 29-26: Total Number of Packets for this transmission
// Bits 25-22: Current packet number
// Bits 21-06: Data (16 bits = 2 chars)
// Bits 05-00: Reserved for idk
//uint32_t* packageData(int messageID, char* dataToPackage)
//{
//	// Get size of data
//	int size = 0;
//	char* dummy = dataToPackage;
//	while (dummy != NULL)
//	{
//		size++;
//		dummy++;
//	}
//
//	// Calculate number of packets (2 chars per packet)
//	int numPackets = ceil(size / 2);
//	if (numPackets > 16) { return NULL; }
//
//	// Package Data
//	uint32_t* packets = (uint32_t*)malloc(numPackets * sizeof(uint32_t));
//	for (int packetNum = 0; packetNum < numPackets; ++packetNum)
//	{
//		uint32_t toPackage = (messageID << MESSAGE_ID_POS) | (numPackets << NUM_PACKETS_POS) | (packetNum << CURRENT_PACKET_POS) | (dataToPackage[2*packetNum + 1] << (DATA_POS+8)) | (dataToPackage[2*packetNum] << DATA_POS);
//		packets[packetNum] = toPackage;
//	}
//
//	return packets;
//}
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
  MX_TIM3_Init();
  MX_UART5_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */


	// init variables
	HAL_StatusTypeDef ret;
	uint8_t buf[50];

	int16_t x_val, y_val, z_val = 0;
	char x_str[100];
	char y_str[100];
	char z_str[100];
	char adc_str[100];


	// pwm for speaker
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

	// pwm for motor
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	TIM3->PSC = 7;
	TIM3->ARR = 3999;

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	TIM3->CCR1 = 0;
	TIM3->CCR2 = 0;
	TIM3->CCR3 = 0;
	TIM3->CCR4 = 0;

	// Enable arm
	// Note: Baud Rate = 115200
	LX16ABus_init(&huart2);

	int k = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	#if MODE == 1
		ret = HAL_UART_Receive(&huart5, buf, 24, HAL_MAX_DELAY);
		if (ret != HAL_OK) { continue; }

		int i = 0;
		int j = 0;
		while(buf[i] != '\0') {
			x_str[j] = buf[i];
			++j;
			++i;
		}
		x_str[j] = '\0';
		j = 0;
		i = 8;
		while(buf[i] != '\0') {
			y_str[j] = buf[i];
			++j;
			++i;
		}
		y_str[j] = '\0';
		j = 0;
		i = 16;
		while(buf[i] != '\0') {
			z_str[j] = buf[i];
			++j;
			++i;
		}
		z_str[j] = '\0';

		// Receive flex sensor values
		ret = HAL_UART_Receive(&huart5, buf, 16, HAL_MAX_DELAY);
		if (ret != HAL_OK) { continue; }

		memcpy(adc_str, buf, 16);

		// Convert accelerometer values to pwm
		// Note: range: 500-12500
		x_val = atoi(x_str);
		y_val = atoi(y_str);
		z_val = atoi(z_str);

		x_val = (abs(x_val) > 12500) ? (x_val < 0 ? -12000 : 12000) : x_val;
		x_val = (abs(x_val) < 1000) ? 0 : x_val;
		y_val = (abs(y_val) > 12500) ? (y_val < 0 ? -12000 : 12000) : y_val;
		y_val = (abs(y_val) < 1000) ? 0 : y_val;
		z_val = (abs(z_val) > 12500) ? (z_val < 0 ? -12000 : 12000) : z_val;
		z_val = (abs(z_val) < 1000) ? 0 : z_val;

		float x_float = x_val / 12000.0;
		float y_float = y_val / 12000.0;
		float z_float = z_val / 12000.0;

		x_float = x_float * 1023;
		y_float = y_float * 1023;
		z_float = z_float * 1023;

		float nMotPremixL, nMotPremixR, nPivSpeed, fPivScale, nMotMixL, nMotMixR;

		if (y_float >= 0) {
		  // Forward
		  nMotPremixL = (x_float>=0)? 1023.0 : (1023.0 + x_float);
		  nMotPremixR = (x_float>=0)? (1023.0 - x_float) : 1023.0;
		} else {
		  // Reverse
		  nMotPremixL = (x_float>=0)? (1023.0 - x_float) : 1023.0;
		  nMotPremixR = (x_float>=0)? 1023.0 : (1023.0 + x_float);
		}

		// Scale Drive output due to Joystick Y input (throttle)
		nMotPremixL = nMotPremixL * y_float/1023.0;
		nMotPremixR = nMotPremixR * y_float/1023.0;

		// Now calculate pivot amount
		// - Strength of pivot (nPivSpeed) based on Joystick X input
		// - Blending of pivot vs drive (fPivScale) based on Joystick Y input
		nPivSpeed = x_float;
		fPivScale = (fabs(y_float)>1023)? 0.0 : (1.0 - fabs(y_float)/1023);


		// Calculate final mix of Drive and Pivot
		nMotMixL = (1.0-fPivScale)*nMotPremixL + fPivScale*( nPivSpeed);
		nMotMixR = (1.0-fPivScale)*nMotPremixR + fPivScale*(-nPivSpeed);
		int CCRL = abs((int)((nMotMixL / 1023) * 4000));
		int CCRR = abs((int)((nMotMixR / 1023) * 4000));
//			char x_x[10];
//			char y_y[10];
//			sprintf(x_x, "%.2f", x_float);
//			sprintf(y_y, "%.2f", y_float);
//			HAL_UART_Transmit(&huart5, x_x, 8, 0xFFFF);
//			HAL_UART_Transmit(&huart5, y_y, 8, 0xFFFF);
		// TIM3 CHANNEL 1 = IN1
		// TIM3 CHANNEL 2 = IN2
		// TIM3 CHANNEL 3 = IN3
		// TIME CHANNEL 4 = IN4

		// send pwm
		if (nMotMixL > 0)
		{
			TIM3->CCR3 = CCRL;
			TIM3->CCR4 = 0;
		}
		else
		{
			TIM3->CCR3 = 0;
			TIM3->CCR4 = CCRL;
		}
		if (nMotMixR > 0)
		{
			TIM3->CCR1 = CCRR;
			TIM3->CCR2 = 0;
		}
		else
		{
			TIM3->CCR1 = 0;
			TIM3->CCR2 = CCRR;
		}
	#elif MODE == 2
		/*
		 * ARM IDs:
		 * 1: Open/Close Hand
		 * 2: Wrist roll
		 * 3: Wrist pitch
		 * 4: Elbow pitch
		 * 5: Shoulder pitch (?)
		 * 6: Shoulder yaw (?)
		 */
/*		for(int i = 0; i < 100; i+=5) {
			ArmPos(i);
			HAL_Delay(120);
		}
		for(int i = 100; i > 0; i-=5) {
			ArmPos(i);
			HAL_Delay(120);
		}*/

		// Look for start character
		ret = HAL_UART_Receive(&huart5, buf, 1, 1000);
		if (ret != HAL_OK || buf[0] != '#') { continue; }

		ret = HAL_UART_Receive(&huart5, buf, 24, 1000);
		if (ret != HAL_OK) { continue; }

		int i = 0;
		int j = 0;
		while(buf[i] != '\0') {
			x_str[j] = buf[i];
			++j;
			++i;
		}
		x_str[j] = '\0';
		j = 0;
		i = 8;
		while(buf[i] != '\0') {
			y_str[j] = buf[i];
			++j;
			++i;
		}
		y_str[j] = '\0';
		j = 0;
		i = 16;
		while(buf[i] != '\0') {
			z_str[j] = buf[i];
			++j;
			++i;
		}
		z_str[j] = '\0';
//		HAL_UART_Transmit(&huart5, (const uint8_t *)x_str, 8, 0xFFFF);
//		HAL_UART_Transmit(&huart5, (const uint8_t *)y_str, 8, 0xFFFF);
//		HAL_UART_Transmit(&huart5, (const uint8_t *)z_str, 8, 0xFFFF);
		x_val = atoi(x_str);
		y_val = atoi(y_str);
		z_val = atoi(z_str);
		y_val = (abs(y_val) > 12500) ? (y_val < 0 ? -12000 : 12000) : y_val;
		y_val = (abs(y_val) < 1000) ? 0 : y_val;
		y_val = abs(y_val);
		k++;
		if(k==1){
			ArmPos((int)((y_val/12000.0)*100));
		}
		if(k == 1200){
			k = 0;
		}

		// Receive flex sensor values
		// Look for start character
		ret = HAL_UART_Receive(&huart5, buf, 1, 1000);
		if (ret != HAL_OK || buf[0] != '#') { continue; }

		ret = HAL_UART_Receive(&huart5, buf, 16, 1000);
		if (ret != HAL_OK) { continue; }

		memcpy(adc_str, buf, 16);

		// Range for flex sensor: 1400 (open) - 2000 (closed)
		int adc_val = atoi(adc_str);
		adc_val = (adc_val < 1400) ? 1400 : (adc_val > 2000 ? 2000 : adc_val);

		float normalized_adc = (adc_val - 1400.0) / 600.0;
		LX16ABus_set_servo(1, normalized_adc * 240.0, 500);
		HAL_Delay(500);
		for (int i = 0; i < 1; ++i) {}
	#else
		#error "Invalid mode"
	#endif
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 18;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STLINK_TX_Pin STLINK_RX_Pin */
  GPIO_InitStruct.Pin = STLINK_TX_Pin|STLINK_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_LPUART1;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STLK_RX_Pin STLK_TX_Pin */
  GPIO_InitStruct.Pin = STLK_RX_Pin|STLK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart5, (uint8_t *)&ch, 1, 0xFFFF);
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

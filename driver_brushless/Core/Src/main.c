/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
typedef struct
{
	GPIO_TypeDef * JoyStick_xGPIO;
	GPIO_TypeDef * JoyStick_yGPIO;
	uint16_t       JoyStick_xPIN;
	uint16_t       JoyStick_yPIN;
	ADC_TypeDef*   ADC_Instance;
	uint32_t       ADCx_CH;
	uint32_t       ADCy_CH;
}JoyStick_CfgType;

/* Private define ------------------------------------------------------------*/
#define JoyStick1    	0
#define X				0 // potentiometrul de pe oX
#define Y				1 // potentiometrul de pe oY

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
ADC_ChannelConfTypeDef sConfig = {0};
uint8_t calibrated = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
uint16_t MAP(uint16_t val, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);
void JoyStick_Init(void);
void JoyStick_Read(uint16_t* JoyStick_XY);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const JoyStick_CfgType JoyStick_CfgParam =
{
	// JoyStick unit 1 Configurations
	// se completeaza cu datele pinilor de la care se face citirea
	GPIOC,
	GPIOC,
	GPIO_PIN_0,
	GPIO_PIN_1,
	ADC1,
	ADC_CHANNEL_1,
	ADC_CHANNEL_2
};
/* USER CODE END 0 */

/**
  * @brief  Comanda motor brushless cu ajutorul driverului Seaking 40A - RTR
  * @retval int
  */
int main(void)
{
  /* variabile locale */
  /* vector cu 2 elem (un elem pentru fiecare potentiometru) */
  uint16_t JoyStick1_XY[] = {0, 0};
  /* tON al PWM-ului */
  uint16_t CH1_DC = 0;

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  JoyStick_Init();
  /* USER CODE END 2 */

  while (1)
  {
	/* citirea valorilor potentiometrelor */
	JoyStick_Read(JoyStick1_XY);

	/* comanda PWM de pe oY */
	CH1_DC = JoyStick1_XY[X];
	TIM2->CCR1 = MAP(CH1_DC, 503, 2386, 0, 65535);

	/* transmitere PWM catre motor*/
	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);

 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.DFSDMConfig = ADC_DFSDM_MODE_ENABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SMPS_EN_Pin|SMPS_V1_Pin|SMPS_SW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SMPS_EN_Pin SMPS_V1_Pin SMPS_SW_Pin */
  GPIO_InitStruct.Pin = SMPS_EN_Pin|SMPS_V1_Pin|SMPS_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SMPS_PG_Pin */
  GPIO_InitStruct.Pin = SMPS_PG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SMPS_PG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD4_Pin */
  GPIO_InitStruct.Pin = LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD4_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint16_t MAP(uint16_t val, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
	return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void JoyStick_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	if(JoyStick_CfgParam.JoyStick_xGPIO == GPIOA || JoyStick_CfgParam.JoyStick_yGPIO == GPIOA)
	{
		__HAL_RCC_GPIOA_CLK_ENABLE();
	}
	else if(JoyStick_CfgParam.JoyStick_xGPIO == GPIOB || JoyStick_CfgParam.JoyStick_yGPIO == GPIOB)
	{
		__HAL_RCC_GPIOB_CLK_ENABLE();
	}
	else if(JoyStick_CfgParam.JoyStick_xGPIO == GPIOC || JoyStick_CfgParam.JoyStick_yGPIO == GPIOC)
	{
		__HAL_RCC_GPIOC_CLK_ENABLE();
	}
	else if (JoyStick_CfgParam.JoyStick_xGPIO == GPIOD || JoyStick_CfgParam.JoyStick_yGPIO == GPIOD)
	{
		__HAL_RCC_GPIOD_CLK_ENABLE();
	}
	else if (JoyStick_CfgParam.JoyStick_xGPIO == GPIOE || JoyStick_CfgParam.JoyStick_yGPIO == GPIOE)
	{
		__HAL_RCC_GPIOD_CLK_ENABLE();
	}

	GPIO_InitStruct.Pin = JoyStick_CfgParam.JoyStick_xPIN;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(JoyStick_CfgParam.JoyStick_xGPIO, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = JoyStick_CfgParam.JoyStick_yPIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(JoyStick_CfgParam.JoyStick_yGPIO, &GPIO_InitStruct);

	hadc1.Instance = JoyStick_CfgParam.ADC_Instance;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	HAL_ADC_Init(&hadc1);
    sConfig.Channel = JoyStick_CfgParam.ADCx_CH;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	if(calibrated == 0)
	{
		HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
		calibrated = 1;
	}
}


void JoyStick_Read(uint16_t* JoyStick_XY)
{
	uint32_t AD_RES;

	// Select The JoyStick Instance ADC Channel For X
	sConfig.Channel = JoyStick_CfgParam.ADCx_CH;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	// Start ADC Conversion
	HAL_ADC_Start(&hadc1);
	// Poll ADC1 Peripheral & TimeOut = 1mSec
	HAL_ADC_PollForConversion(&hadc1, 1);
	// Read The ADC Conversion Result Write It To JoyStick X
	AD_RES = HAL_ADC_GetValue(&hadc1);
	JoyStick_XY[0] = AD_RES;

	// Select The JoyStick Instance ADC Channel For Y
	sConfig.Channel = JoyStick_CfgParam.ADCy_CH;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	// Start ADC Conversion
	HAL_ADC_Start(&hadc1);
	// Poll ADC1 Peripheral & TimeOut = 1mSec
	HAL_ADC_PollForConversion(&hadc1, 1);
	// Read The ADC Conversion Result Write It To JoyStick Y
	AD_RES = HAL_ADC_GetValue(&hadc1);
	JoyStick_XY[1] = AD_RES;
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

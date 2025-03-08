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
#include "ws28xx.h"
#include <stdio.h>
#include <stdlib.h>

/*after change in IOC, change line
 #include "tim.h"
 for
 #include "stm32f1xx_hal.h"
 in ws28xx.h
 */

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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim2_ch1;
DMA_HandleTypeDef hdma_tim2_ch3;
DMA_HandleTypeDef hdma_tim3_ch3;
DMA_HandleTypeDef hdma_tim4_ch2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint32_t tempCode;
uint8_t bitIndex;
uint8_t cmd;
uint8_t cmdli;
volatile uint32_t code;
volatile bool processIR = 0;

uint8_t animationCode = 0;

WS28XX_HandleTypeDef string1,string7,string9,string4;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void outputControl(uint8_t en);
void setAllBlue(WS28XX_HandleTypeDef ws);
void turnAllOff(WS28XX_HandleTypeDef ws);
void GlitteringWavesEffect(WS28XX_HandleTypeDef ws);
void EnchantedRippleEffect(WS28XX_HandleTypeDef ws);
void TwinklingGlowEffect(WS28XX_HandleTypeDef ws);
void CarTurnSignalEffect(WS28XX_HandleTypeDef ws);
void TestMode(WS28XX_HandleTypeDef ws);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	printf("Initialization begin\r\n");

	HAL_GPIO_WritePin(powerOn_GPIO_Port, powerOn_Pin, SET); // Power LDO Enable

	HAL_TIM_Base_Start(&htim1); // IR Receiver timer start
	__HAL_TIM_SET_COUNTER(&htim1, 0); // IR Receiver timer cnt set

	WS28XX_Init(&string1, &htim2, 72, TIM_CHANNEL_3, 8);
	WS28XX_Init(&string4, &htim4, 72, TIM_CHANNEL_2, 3);
	WS28XX_Init(&string7, &htim2, 72, TIM_CHANNEL_1, 3);
	WS28XX_Init(&string9, &htim3, 72, TIM_CHANNEL_3, 3);

	uint32_t tmheartbeat = 0, tm = 0; // Timestamp variable
	bool heartbeat = 0;
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0); //heartbeat pin

	uint8_t ledsON = 0; //Some LED should be on - enable boost
	uint32_t tmboostON = 0; //timestamp for boost enabling
	bool boostONswitch = 0;

	printf("Initialization complete\r\n");

	WS28XX_SetPixel_RGBW_565(&string1, 0, COLOR_RGB565_BLUE, 50); //default init values
	WS28XX_SetPixel_RGBW_565(&string1, 1, COLOR_RGB565_CRIMSON, 50);
	WS28XX_SetPixel_RGBW_565(&string1, 2, COLOR_RGB565_ORANGE, 50);
	WS28XX_SetPixel_RGBW_565(&string1, 7, COLOR_RGB565_ORANGE, 50);
	WS28XX_Update(&string1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		if (tmheartbeat < HAL_GetTick()) { //heartbeat LED
			if (heartbeat)
				tmheartbeat = HAL_GetTick() + 950;
			else
				tmheartbeat = HAL_GetTick() + 50;

			HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin); //heartbeat LED

			heartbeat = !heartbeat;
		}

		if (processIR == 1) { //Command received from IR decoding
			processIR = 0; //reset flag

			printf("Data: %X\r\n",(unsigned int)code); //debug terminal text

			switch (code) {
			case 0xFF8877: // "1"
				animationCode = 1;
				ledsON = 1;
				break;
			case 0xFF48B7: // "2"
				animationCode = 2;
				ledsON = 1;
				break;
			case 0xFFC837: // "3"
				animationCode = 3;
				ledsON = 1;
				break;
			case 0xFF28D7: // "4"
				animationCode = 4;
				ledsON = 1;
				break;
			case 0xFFA857: // "5"
				animationCode = 5;
				ledsON = 1;
				break;
			case 0xFF6897: // "6"
				animationCode = 6;
				ledsON = 1;
				break;
			case 0xFF58A7: // "0"
				animationCode = 0;
				ledsON = 0;
				break;
			case 0xFF50AF: // "OFF"
				printf("Power OFF\r\n");
				HAL_GPIO_WritePin(powerOn_GPIO_Port, powerOn_Pin, RESET); // Power LDO Disable, MCU suicide
				break;
			default:
				break;
			}

			//TODO enable check
			outputControl(ledsON);
		}

		if (tm < HAL_GetTick()) { //routine for effects
			tm = HAL_GetTick() + 10;

			switch (animationCode) {
			case 0: // "0"
				turnAllOff(string1);
				break;
			case 1: // "1"
				setAllBlue(string1);
				break;
			case 2: // "2"
				GlitteringWavesEffect(string1);
				break;
			case 3: // "3"
				EnchantedRippleEffect(string1);
				break;
			case 4: // "4"
				TwinklingGlowEffect(string1);
				break;
			case 5: // "5"
				CarTurnSignalEffect(string1);
				break;
			case 6: // "6"
				TestMode(string1);
				break;
			default:
				break;

			}
		}

		if (tmboostON < HAL_GetTick() && ledsON == 1) { //routine for waking up boost
			if (boostONswitch == 0) { //100ms high pulse every 20s, if LEDs are running
				tmboostON = HAL_GetTick() + 100;
				HAL_GPIO_WritePin(boostWkUp_GPIO_Port, boostWkUp_Pin,
						GPIO_PIN_SET);
			} else {
				tmboostON = HAL_GetTick() + 20000;
				HAL_GPIO_WritePin(boostWkUp_GPIO_Port, boostWkUp_Pin,
						GPIO_PIN_RESET);
			}
			boostONswitch = !boostONswitch;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, boostWkUp_Pin|powerOn_Pin|LED2_Pin|EN_5V_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IRQ_Boost_Pin SCL_Boost_Pin SDA_Boost_Pin */
  GPIO_InitStruct.Pin = IRQ_Boost_Pin|SCL_Boost_Pin|SDA_Boost_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : boostWkUp_Pin powerOn_Pin LED2_Pin EN_5V_Pin */
  GPIO_InitStruct.Pin = boostWkUp_Pin|powerOn_Pin|LED2_Pin|EN_5V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* UART PRINTF MAP */
#if defined(__GNUC__)
int _write(int fd, char *ptr, int len) {
	HAL_UART_Transmit(&huart1, (uint8_t*) ptr, len, HAL_MAX_DELAY);
	return len;
}
#elif defined (__ICCARM__)
#include "LowLevelIOInterface.h"
size_t __write(int handle, const unsigned char * buffer, size_t size)
{
	HAL_UART_Transmit(&huart1, (uint8_t *) buffer, size, HAL_MAX_DELAY);
	return size;
}
#elif defined (__CC_ARM)
int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}
#endif

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
//HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}

/* IR TIMER AND DECODING */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_1) {
		if (__HAL_TIM_GET_COUNTER(&htim1) > 8000) {
			tempCode = 0;
			bitIndex = 0;
		} else if (__HAL_TIM_GET_COUNTER(&htim1) > 1700) {
			tempCode |= (1UL << (31 - bitIndex));   // write 1
			bitIndex++;
		} else if (__HAL_TIM_GET_COUNTER(&htim1) > 1000) {
			tempCode &= ~(1UL << (31 - bitIndex));  // write 0
			bitIndex++;
		}
		//printf("Data: %d\r\n", __HAL_TIM_GET_COUNTER(&htim1)); //debug terminal text

		if (bitIndex == 32) {
			cmdli = ~tempCode; // Logical inverted last 8 bits
			cmd = tempCode >> 8; // Second last 8 bits
			if (cmdli == cmd) // Check for errors
					{
				code = tempCode; // If no bit errors
				// Do your main work HERE
				//printf("Data: %X\r\n",code);
				processIR = 1;
			}
			bitIndex = 0;
		}
		__HAL_TIM_SET_COUNTER(&htim1, 0);
	}
}

/* OUTPUT ENABLE CONTROL */
void outputControl(uint8_t en){
	if(en == 1)
		HAL_GPIO_WritePin(EN_5V_GPIO_Port, EN_5V_Pin, SET);
	else
		HAL_GPIO_WritePin(EN_5V_GPIO_Port, EN_5V_Pin, RESET);
}

/* LED EFFECTS */

void setAllBlue(WS28XX_HandleTypeDef ws) {
	for(int i = 0; i<8; i++){
	WS28XX_SetPixel_RGBW_565(&ws, i, COLOR_RGB565_CYAN, 100);
	}

	WS28XX_Update(&ws);
}

void turnAllOff(WS28XX_HandleTypeDef ws) {
	for(int i = 0; i<8; i++){
	WS28XX_SetPixel_RGBW_565(&ws, i, COLOR_RGB888_BLACK, 100);
	}
	WS28XX_Update(&ws);
}

void GlitteringWavesEffect(WS28XX_HandleTypeDef ws) {
	 static uint32_t next_update = 0;
	    static uint8_t brightness[8] = {100, 120, 140, 160, 180, 200, 220, 240}; // Initial brightness
	    static int8_t delta[8] = {10, -15, 20, -10, 15, -20, 10, -5}; // Different rates for each LED

	    if (HAL_GetTick() >= next_update) {
	        next_update = HAL_GetTick() + 5; // 5ms periodic update

	        for (int i = 0; i < 8; i++) {
	            brightness[i] += delta[i];
	            if (brightness[i] >= 255 || brightness[i] <= 100) {
	                delta[i] = -delta[i]; // Reverse direction at limits
	            }
	            WS28XX_SetPixel_RGBW_565(&ws, i, COLOR_RGB565_CYAN, brightness[i]);
	        }
		WS28XX_Update(&ws);
	}
}
void EnchantedRippleEffect(WS28XX_HandleTypeDef ws) {
	static uint32_t next_update = 0;
	    static uint8_t brightness = 0;
	    static int8_t direction = 1; // 1 for increasing, -1 for decreasing

	    if (HAL_GetTick() >= next_update) {
	        next_update = HAL_GetTick() + 5; // 5ms periodic update

	        brightness += 5 * direction;
	        if (brightness >= 255 || brightness <= 0) {
	            direction = -direction; // Reverse at boundaries
	        }

	        for (int i = 0; i < 8; i++) {
	            uint8_t adjusted_brightness = (brightness + (i * 32)) % 256; // Creates a wave effect
	            WS28XX_SetPixel_RGBW_565(&ws, i, COLOR_RGB565_BLUE, adjusted_brightness);
	        }
		WS28XX_Update(&ws);

	}
}
void TwinklingGlowEffect(WS28XX_HandleTypeDef ws) {
	 static uint32_t next_update = 0;
	    static uint8_t brightness[8] = {50, 100, 150, 200, 120, 180, 60, 90}; // Initial brightness
	    static int8_t delta[8] = {5, -3, 4, -6, 7, -2, 3, -4}; // Different speed changes

	    if (HAL_GetTick() >= next_update) {
	        next_update = HAL_GetTick() + 5; // 5ms periodic update

	        for (int i = 0; i < 8; i++) {
	            brightness[i] += delta[i];
	            if (brightness[i] >= 255 || brightness[i] <= 50) {
	                delta[i] = -delta[i]; // Reverse direction at limits
	                // Randomize the twinkle speed
	                if (brightness[i] <= 50) {
	                    delta[i] = (rand() % 5) + 3;
	                }
	            }
	            WS28XX_SetPixel_RGBW_565(&ws, i, COLOR_RGB565_AQUA, brightness[i]);
	        }
		WS28XX_Update(&ws);
	}
}

void CarTurnSignalEffect(WS28XX_HandleTypeDef ws) {
	static uint32_t next_update = 0;
	static bool leds_on = false; // State to track whether the LEDs are on or off

	if (HAL_GetTick() >= next_update) {
		next_update = HAL_GetTick() + (leds_on ? 300 : 700); // 300ms ON, 700ms OFF for classic timing

		if (leds_on) {
			// Turn off all LEDs
			for (int i = 0; i < 8; i++) {
				WS28XX_SetPixel_RGBW_565(&ws, i, COLOR_RGB565_BLACK, 0);
			}
			leds_on = false;
		} else {
			// Turn on all LEDs with Amber
			for (int i = 0; i < 8; i++) {
				WS28XX_SetPixel_RGBW_565(&ws, i, COLOR_RGB565_ORANGE, 255);
			}
			leds_on = true;
		}
		WS28XX_Update(&ws);
	}
}

void TestMode(WS28XX_HandleTypeDef ws) {static uint32_t next_update = 0;
static int led_index = 0; // Starts at LED 1
static int color_stage = 0; // 0 = Red, 1 = Green, 2 = Blue, 3 = Off
static bool all_leds_stage = false; // False = individual LED test, True = all LEDs cycle
static int all_leds_cycle_count = 0; // Tracks if the all-LED cycle has completed once

if (HAL_GetTick() >= next_update) {
    if (!all_leds_stage) {
        next_update = HAL_GetTick() + 100; // 100ms timing for individual LED cycling

        // Turn off all LEDs before updating
        for (int i = 0; i < 8; i++) {
            WS28XX_SetPixel_RGBW_565(&ws, i, COLOR_RGB565_BLACK, 0);
        }

        // Set the current LED to the correct color
        uint16_t color;
        switch (color_stage) {
            case 0: color = COLOR_RGB565_RED; break;
            case 1: color = COLOR_RGB565_GREEN; break;
            case 2: color = COLOR_RGB565_BLUE; break;
            default: color = COLOR_RGB565_BLACK; break;
        }

        WS28XX_SetPixel_RGBW_565(&ws, led_index, color, 255);
        WS28XX_Update(&ws);

        // Advance color stage
        color_stage++;
        if (color_stage > 3) { // Once Off state is reached, move to next LED
            color_stage = 0;
            led_index++;
        }

        // If all LEDs have been tested, switch to "all LEDs together" phase
        if (led_index >= 8) {
            all_leds_stage = true;
            color_stage = 0;
            all_leds_cycle_count = 0;
        }
    } else {
        next_update = HAL_GetTick() + 1000; // 1s timing for all-LEDs phase

        // Cycle all LEDs together through Red → Green → Blue
        uint16_t color;
        switch (color_stage) {
            case 0: color = COLOR_RGB565_RED; break;
            case 1: color = COLOR_RGB565_GREEN; break;
            case 2: color = COLOR_RGB565_BLUE; break;
            default: color = COLOR_RGB565_BLACK; break;
        }

        for (int i = 0; i < 8; i++) {
            WS28XX_SetPixel_RGBW_565(&ws, i, color, 255);
        }
        WS28XX_Update(&ws);

        // Advance global color stage
        color_stage++;
        all_leds_cycle_count++;

        // After one full RGB cycle, return to individual LED testing
        if (all_leds_cycle_count >= 3) {
            all_leds_stage = false;
            led_index = 0; // Restart from LED 1
            color_stage = 0;
        }
    }
}
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
	while (1) {
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

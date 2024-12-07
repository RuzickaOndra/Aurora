/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim15;
DMA_HandleTypeDef hdma_tim15_ch1_up_trig_com;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint32_t tempCode;
uint8_t bitIndex;
uint8_t cmd;
uint8_t cmdli;
volatile uint32_t code;
volatile bool processIR = 0;

uint8_t animationCode = 0;

WS28XX_HandleTypeDef ws;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM15_Init(void);
/* USER CODE BEGIN PFP */
// Function prototypes
void setAllBlue(void);
void turnAllOff(void);
void GlitteringWavesEffect(void);
void EnchantedRippleEffect(void);
void TwinklingGlowEffect(void);
void CarTurnSignalEffect(void);
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
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start(&htim3); // IR Receiver timer start
	__HAL_TIM_SET_COUNTER(&htim3, 0); // IR Receiver timer cnt set

	WS28XX_Init(&ws, &htim15, 48, TIM_CHANNEL_1, 3);

	uint32_t tmheartbeat = 0, tm = 0; // Timestamp variable
	bool heartbeat = 0;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1); //heartbeat pin

	uint8_t ledsON = 0; //Some LED should be on - enable boost
	uint32_t tmboostON = 0; //timestamp for boost enabling
	bool boostONswitch = 0;

	WS28XX_SetPixel_RGBW_565(&ws, 0, COLOR_RGB565_BLUE, 50); //default init values
	WS28XX_SetPixel_RGBW_565(&ws, 1, COLOR_RGB565_CRIMSON, 50);
	WS28XX_SetPixel_RGBW_565(&ws, 2, COLOR_RGB565_ORANGE, 50);
	WS28XX_Update(&ws);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		if (tmheartbeat < HAL_GetTick()) { //heartbeat LED
			if (heartbeat)
				tmheartbeat = HAL_GetTick() + 950;
			else
				tmheartbeat = HAL_GetTick() + 50;

			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); //heartbeat LED

			heartbeat = !heartbeat;
		}

		if (processIR == 1) { //Command received from IR decoding
			processIR = 0; //reset flag

			printf("Data: %X\r\n", code); //debug terminal text

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
			case 0xFF58A7: // "0"
				animationCode = 0;
				ledsON = 0;
				break;
			default:
				break;
			}

			if (HAL_GPIO_ReadPin(sense5V_GPIO_Port, sense5V_Pin)==0 && ledsON == 1) //output OFF, but leds should be on
					{
				HAL_GPIO_WritePin(boost_WKUP_GPIO_Port, boost_WKUP_Pin, GPIO_PIN_RESET); //turn the boost on (100ms LOW pulse)
				HAL_Delay(100);
				HAL_GPIO_WritePin(boost_WKUP_GPIO_Port, boost_WKUP_Pin,	GPIO_PIN_SET);
			}

		}

		if (tm < HAL_GetTick()) { //routine for effects
			tm = HAL_GetTick() + 10;

			switch (animationCode) {
			case 0: // "0"
				turnAllOff();
				break;
			case 1: // "1"
				setAllBlue();
				break;
			case 2: // "2"
				GlitteringWavesEffect();
				break;
			case 3: // "3"
				EnchantedRippleEffect();
				break;
			case 4: // "4"
				TwinklingGlowEffect();
				break;
			case 5: // "5"
				CarTurnSignalEffect();
				break;
			default:
				break;

			}
		}

		if (tmboostON < HAL_GetTick() && ledsON == 1) { //routine for waking up boost

			if (boostONswitch) { //100ms low pulse every 20s, if LEDs are running
				tmboostON = HAL_GetTick() + 100;
				HAL_GPIO_WritePin(boost_WKUP_GPIO_Port, boost_WKUP_Pin,
						GPIO_PIN_RESET);
			} else {
				tmboostON = HAL_GetTick() + 20000;
				HAL_GPIO_WritePin(boost_WKUP_GPIO_Port, boost_WKUP_Pin,
						GPIO_PIN_SET);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 48;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */
//
  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */
//
  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */
//
  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(boost_WKUP_GPIO_Port, boost_WKUP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : boost_WKUP_Pin */
  GPIO_InitStruct.Pin = boost_WKUP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(boost_WKUP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : sense5V_Pin */
  GPIO_InitStruct.Pin = sense5V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(sense5V_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

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
	if (GPIO_Pin == GPIO_PIN_6) {
		if (__HAL_TIM_GET_COUNTER(&htim3) > 8000) {
			tempCode = 0;
			bitIndex = 0;
		} else if (__HAL_TIM_GET_COUNTER(&htim3) > 1700) {
			tempCode |= (1UL << (31 - bitIndex));   // write 1
			bitIndex++;
		} else if (__HAL_TIM_GET_COUNTER(&htim3) > 1000) {
			tempCode &= ~(1UL << (31 - bitIndex));  // write 0
			bitIndex++;
		}
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
		__HAL_TIM_SET_COUNTER(&htim3, 0);
	}
}

/* LED EFFECTS */

void setAllBlue(void) {
	WS28XX_SetPixel_RGBW_565(&ws, 0, COLOR_RGB565_CYAN, 100);
	WS28XX_SetPixel_RGBW_565(&ws, 1, COLOR_RGB565_CYAN, 100);
	WS28XX_SetPixel_RGBW_565(&ws, 2, COLOR_RGB565_CYAN, 100);
	WS28XX_Update(&ws);
}

void turnAllOff(void) {
	WS28XX_SetPixel_RGBW_565(&ws, 0, COLOR_RGB888_BLACK, 0);
	WS28XX_SetPixel_RGBW_565(&ws, 1, COLOR_RGB888_BLACK, 0);
	WS28XX_SetPixel_RGBW_565(&ws, 2, COLOR_RGB888_BLACK, 0);
	WS28XX_Update(&ws);
}

void GlitteringWavesEffect(void) {
	static uint32_t next_update = 0;
	static uint8_t brightness[3] = { 100, 120, 140 }; // Initial brightness for each LED
	static int8_t delta[3] = { 10, -15, 20 }; // Change in brightness for each LED

	if (HAL_GetTick() >= next_update) {
		next_update = HAL_GetTick() + 5; // 5ms periodic update

		for (int i = 0; i < 3; i++) {
			brightness[i] += delta[i];
			if (brightness[i] >= 255 || brightness[i] <= 100) {
				delta[i] = -delta[i]; // Reverse direction at boundaries
			}
			WS28XX_SetPixel_RGBW_565(&ws, i, COLOR_RGB565_CYAN, brightness[i]);
		}
		WS28XX_Update(&ws);
	}
}
void EnchantedRippleEffect(void) {
	static uint32_t next_update = 0;
	static uint8_t brightness = 0;
	static int8_t direction = 1; // 1 for increasing, -1 for decreasing

	if (HAL_GetTick() >= next_update) {
		next_update = HAL_GetTick() + 5; // 5ms periodic update

		brightness += 5 * direction;
		if (brightness >= 255 || brightness <= 0) {
			direction = -direction; // Reverse at boundaries
		}

		WS28XX_SetPixel_RGBW_565(&ws, 0, COLOR_RGB565_BLUE, brightness);
		WS28XX_SetPixel_RGBW_565(&ws, 1, COLOR_RGB565_GREEN, brightness / 2);
		WS28XX_SetPixel_RGBW_565(&ws, 2, COLOR_RGB565_PURPLE, 255 - brightness);
		WS28XX_Update(&ws);

	}
}
void TwinklingGlowEffect(void) {
	static uint32_t next_update = 0;
	static uint8_t brightness[3] = { 50, 100, 150 }; // Initial brightness for each LED
	static int8_t delta[3] = { 5, -3, 4 }; // Change rate for each LED

	if (HAL_GetTick() >= next_update) {
		next_update = HAL_GetTick() + 5; // 5ms periodic update

		for (int i = 0; i < 3; i++) {
			brightness[i] += delta[i];
			if (brightness[i] >= 255 || brightness[i] <= 50) {
				delta[i] = -delta[i]; // Reverse direction at boundaries
				// Add randomness for a twinkle effect
				if (brightness[i] <= 50) {
					delta[i] = (rand() % 5) + 3; // Randomize twinkle speed
				}
			}
			WS28XX_SetPixel_RGBW_565(&ws, i, COLOR_RGB565_AQUA, brightness[i]);
		}
		WS28XX_Update(&ws);
	}
}

void CarTurnSignalEffect(void) {
	static uint32_t next_update = 0;
	static bool leds_on = false; // State to track whether the LEDs are on or off

	if (HAL_GetTick() >= next_update) {
		next_update = HAL_GetTick() + (leds_on ? 300 : 700); // 300ms ON, 700ms OFF for classic timing

		if (leds_on) {
			// Turn off all LEDs
			for (int i = 0; i < 3; i++) {
				WS28XX_SetPixel_RGBW_565(&ws, i, COLOR_RGB565_BLACK, 0);
			}
			leds_on = false;
		} else {
			// Turn on all LEDs with Amber
			for (int i = 0; i < 3; i++) {
				WS28XX_SetPixel_RGBW_565(&ws, i, COLOR_RGB565_ORANGE, 255);
			}
			leds_on = true;
		}
		WS28XX_Update(&ws);
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

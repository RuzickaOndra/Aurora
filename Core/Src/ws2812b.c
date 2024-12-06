/*

  WS2812B CPU and memory efficient library

  Date: 28.9.2016

  Author: Martin Hubacek
  	  	  http://www.martinhubacek.cz
  	  	  @hubmartin

  Licence: MIT License

 */

#include <string.h>
#include <ws2812b.h>

#include "stm32f0xx_hal.h"
#include "main.h"

extern WS2812_Struct ws2812b;

// Define source arrays for my DMAs
uint32_t WS2812_IO_High[] =  { WS2812B_PINS };
uint32_t WS2812_IO_Low[] = {WS2812B_PINS << 16};

// WS2812 framebuffer - buffer for 2 LEDs - two times 24 bits
uint16_t ws2812bDmaBitBuffer[24 * 2];

// Gamma correction table
const uint8_t gammaTable[] = {
		0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
		0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
		1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
		2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
		5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
		10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
		17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
		25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
		37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
		51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
		69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
		90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
		115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
		144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
		177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
		215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255 };

static void ws2812b_gpio_init(void)
{
	// WS2812B outputs
	WS2812B_GPIO_CLK_ENABLE();
	GPIO_InitTypeDef  GPIO_InitStruct;
	GPIO_InitStruct.Pin       = WS2812B_PINS;
	GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(WS2812B_PORT, &GPIO_InitStruct);

	// Enable output pins for debuging to see DMA Full and Half transfer interrupts
#if defined(LED_BLUE_PORT) && defined(LED_ORANGE_PORT)
	__HAL_RCC_GPIOD_CLK_ENABLE();

	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	GPIO_InitStruct.Pin = LED_BLUE_PIN;
	HAL_GPIO_Init(LED_BLUE_PORT, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = LED_ORANGE_PIN;
	HAL_GPIO_Init(LED_ORANGE_PORT, &GPIO_InitStruct);
#endif
}

TIM_HandleTypeDef    TIM15_handle;
TIM_OC_InitTypeDef tim2OC1;
TIM_OC_InitTypeDef tim2OC2;

uint32_t tim_period;
uint32_t timer_reset_pulse_period;

static void TIM15_init(void)
{
	// TIM15 Periph clock enable
	__HAL_RCC_TIM15_CLK_ENABLE();

	// This computation of pulse length should work ok,
	// at some slower core speeds it needs some tuning.
	tim_period =  SystemCoreClock / 800000; // 0,125us period (10 times lower the 1,25us period to have fixed math below)
	timer_reset_pulse_period = (SystemCoreClock / (320 * 60)); // 60us just to be sure

	uint32_t cc1 = (10 * tim_period) / 36;
	uint32_t cc2 = (10 * tim_period) / 15;

	htim15.Instance = TIM15;
	htim15.Init.Prescaler = 0;
	htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim15.Init.Period = tim_period;
	htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim15.Init.RepetitionCounter = 0;
	htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

	if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
		Error_Handler();

	TIM_OC_InitTypeDef sConfigOC = {0};

	// Configure TIM_CHANNEL_1
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = cc1; // Duty cycle for channel 1
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
		Error_Handler();

	// Configure TIM_CHANNEL_2
	sConfigOC.Pulse = cc2; // Duty cycle for channel 2
	if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
		Error_Handler();

	HAL_TIM_Base_Start(&htim15);
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
}



DMA_HandleTypeDef     dmaUpdate;
DMA_HandleTypeDef     dmaCC1;
DMA_HandleTypeDef     dmaCC2;
#define BUFFER_SIZE		(sizeof(ws2812bDmaBitBuffer)/sizeof(uint16_t))

uint32_t dummy;


static void DMA2_init(void)
{
    // Enable the DMA1 clock
    __HAL_RCC_DMA1_CLK_ENABLE();

    // DMA Configuration for TIM15 Update Event
    DMA_HandleTypeDef dmaUpdate;
    dmaUpdate.Instance = DMA1_Channel5; // Update DMA channel for TIM15 (check manual)
    dmaUpdate.Init.Direction = DMA_MEMORY_TO_PERIPH;
    dmaUpdate.Init.PeriphInc = DMA_PINC_DISABLE;
    dmaUpdate.Init.MemInc = DMA_MINC_ENABLE; // Assuming bit buffer in memory
    dmaUpdate.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    dmaUpdate.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    dmaUpdate.Init.Mode = DMA_CIRCULAR;
    dmaUpdate.Init.Priority = DMA_PRIORITY_HIGH;

    HAL_DMA_DeInit(&dmaUpdate); // Ensure clean state
    HAL_DMA_Init(&dmaUpdate);

    // Link the DMA to TIM15 Update event
    __HAL_LINKDMA(&htim15, hdma[TIM_DMA_ID_UPDATE], dmaUpdate);

    // Configure NVIC for DMA interrupts
    HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

    // Start DMA Transfer (example: bit buffer to GPIO BSRR for WS2812B)
    HAL_DMA_Start(&dmaUpdate, (uint32_t)ws2812bDmaBitBuffer, (uint32_t)&WS2812B_PORT->BSRR, BUFFER_SIZE);

    // Repeat for TIM15 CC1 and CC2 if needed
    //DMA_HandleTypeDef dmaCC1;
    /*dmaCC1.Instance = DMA1_Channel2; // Compare DMA channel for TIM15 CH1
    dmaCC1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    dmaCC1.Init.PeriphInc = DMA_PINC_DISABLE;
    dmaCC1.Init.MemInc = DMA_MINC_ENABLE;
    dmaCC1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    dmaCC1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    dmaCC1.Init.Mode = DMA_CIRCULAR;
    dmaCC1.Init.Priority = DMA_PRIORITY_HIGH;

    HAL_DMA_DeInit(&dmaCC1);
    HAL_DMA_Init(&dmaCC1);
    __HAL_LINKDMA(&htim15, hdma[TIM_DMA_ID_CC1], dmaCC1);

    HAL_DMA_Start(&dmaCC1, (uint32_t)ws2812bDmaBitBuffer, (uint32_t)&WS2812B_PORT->BSRR, BUFFER_SIZE);

    HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);*/
}






static void loadNextFramebufferData(WS2812_BufferItem *bItem, uint32_t row)
{

	uint32_t r = bItem->frameBufferPointer[bItem->frameBufferCounter++];
	uint32_t g = bItem->frameBufferPointer[bItem->frameBufferCounter++];
	uint32_t b = bItem->frameBufferPointer[bItem->frameBufferCounter++];

	if(bItem->frameBufferCounter == bItem->frameBufferSize)
		bItem->frameBufferCounter = 0;

	ws2812b_set_pixel(bItem->channel, row, r, g, b);
}


// Transmit the framebuffer
static void WS2812_sendbuf()
{
    // Transmission complete flag
    ws2812b.transferComplete = 0;

    uint32_t i;

    for (i = 0; i < WS2812_BUFFER_COUNT; i++)
    {
        ws2812b.item[i].frameBufferCounter = 0;

        loadNextFramebufferData(&ws2812b.item[i], 0); // ROW 0
        loadNextFramebufferData(&ws2812b.item[i], 1); // ROW 0
    }

    // Clear all DMA flags for DMA1
    __HAL_DMA_CLEAR_FLAG(&dmaUpdate, DMA_FLAG_TC5 | DMA_FLAG_HT5 | DMA_FLAG_TE5);
    //__HAL_DMA_CLEAR_FLAG(&dmaCC1, DMA_FLAG_TC2 | DMA_FLAG_HT2 | DMA_FLAG_TE2);
    //__HAL_DMA_CLEAR_FLAG(&dmaCC2, DMA_FLAG_TC3 | DMA_FLAG_HT3 | DMA_FLAG_TE3);

    // Configure the number of bytes to be transferred by the DMA controller
    //dmaUpdate.Instance->CNDTR = BUFFER_SIZE;
    //dmaCC1.Instance->CNDTR = BUFFER_SIZE;
    //dmaCC2.Instance->CNDTR = BUFFER_SIZE;

    // Clear all TIM15 flags
    __HAL_TIM_CLEAR_FLAG(&htim15, TIM_FLAG_UPDATE | TIM_FLAG_CC1 | TIM_FLAG_CC2);

    // Enable DMA channels
    __HAL_DMA_ENABLE(&dmaUpdate);
    //__HAL_DMA_ENABLE(&dmaCC1);
    //__HAL_DMA_ENABLE(&dmaCC2);

    // Enable the TIM15 DMA requests AFTER enabling the DMA channels
     __HAL_TIM_ENABLE_DMA(&htim15, TIM_DMA_UPDATE);
   // __HAL_TIM_ENABLE_DMA(&htim15, TIM_DMA_CC1);
    //__HAL_TIM_ENABLE_DMA(&htim15, TIM_DMA_CC2);

    // Set TIM15 counter
    TIM15->CNT = tim_period - 1;

    // Start TIM15
    __HAL_TIM_ENABLE(&htim15);
}


void DMA_TransferError(DMA_HandleTypeDef *DmaHandle)
{
	volatile int i = 0;
	i++;
}


void DMA_TransferHalfHandler(DMA_HandleTypeDef *DmaHandle)
{

	// Is this the last LED?
	if(ws2812b.repeatCounter == WS2812B_NUMBER_OF_LEDS)
	{

		// If this is the last pixel, set the next pixel value to zeros, because
		// the DMA would not stop exactly at the last bit.
		ws2812b_set_pixel(0, 0, 0, 0, 0);

	} else {
		uint32_t i;

		for( i = 0; i < WS2812_BUFFER_COUNT; i++ )
		{
			loadNextFramebufferData(&ws2812b.item[i], 0);
		}

		ws2812b.repeatCounter++;
	}



}

void DMA_TransferCompleteHandler(DMA_HandleTypeDef *DmaHandle)
{

#if defined(LED_ORANGE_PORT)
	LED_ORANGE_PORT->BSRR = LED_ORANGE_PIN;
#endif

	if(ws2812b.repeatCounter == WS2812B_NUMBER_OF_LEDS)
	{
		// Transfer of all LEDs is done, disable DMA but enable tiemr update IRQ to stop the 50us pulse
		ws2812b.repeatCounter = 0;

		// Stop timer
		TIM15->CR1 &= ~TIM_CR1_CEN;

		// Disable DMA
		__HAL_DMA_DISABLE(&dmaUpdate);
		//__HAL_DMA_DISABLE(&dmaCC1);
		//__HAL_DMA_DISABLE(&dmaCC2);

		// Disable the DMA requests
		__HAL_TIM_DISABLE_DMA(&TIM15_handle, TIM_DMA_UPDATE);
		//__HAL_TIM_DISABLE_DMA(&TIM15_handle, TIM_DMA_CC1);
		//__HAL_TIM_DISABLE_DMA(&TIM15_handle, TIM_DMA_CC2);

		// Set 50us period for Treset pulse
		//TIM2->PSC = 1000; // For this long period we need prescaler 1000
		TIM15->ARR = timer_reset_pulse_period;
		// Reset the timer
		TIM15->CNT = 0;

		// Generate an update event to reload the prescaler value immediately
		TIM15->EGR = TIM_EGR_UG;
		__HAL_TIM_CLEAR_FLAG(&TIM15_handle, TIM_FLAG_UPDATE);

		// Enable TIM2 Update interrupt for 50us Treset signal
		__HAL_TIM_ENABLE_IT(&TIM15_handle, TIM_IT_UPDATE);
		// Enable timer
		TIM15->CR1 |= TIM_CR1_CEN;

		// Manually set outputs to low to generate 50us reset impulse
		WS2812B_PORT->BSRR = WS2812_IO_Low[0];
	} else {

		// Load bitbuffer with next RGB LED values
		uint32_t i;
		for( i = 0; i < WS2812_BUFFER_COUNT; i++ )
		{
			loadNextFramebufferData(&ws2812b.item[i], 1);
		}

		ws2812b.repeatCounter++;
	}



#if defined(LED_ORANGE_PORT)
	LED_ORANGE_PORT->BSRR = LED_ORANGE_PIN << 16;
#endif

}

void DMA2_Stream2_IRQHandler(void)
{

#if defined(LED_BLUE_PORT)
	LED_BLUE_PORT->BSRR = LED_BLUE_PIN;
#endif

	// Check the interrupt and clear flag
	HAL_DMA_IRQHandler(&dmaCC2);

#if defined(LED_BLUE_PORT)
	LED_BLUE_PORT->BSRR = LED_BLUE_PIN << 16;
#endif
}

void TIM1_UP_TIM10_IRQHandler(void)
{
#if defined(LED_ORANGE_PORT)
	LED_ORANGE_PORT->BSRR = LED_ORANGE_PIN;
#endif

	HAL_TIM_IRQHandler(&TIM15_handle);

#if defined(LED_ORANGE_PORT)
	LED_ORANGE_PORT->BSRR = LED_ORANGE_PIN << 16;
#endif
}

// TIM2 Interrupt Handler gets executed on every TIM2 Update if enabled
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/*
	// I have to wait 50us to generate Treset signal
	if (ws2812b.timerPeriodCounter < (uint8_t)WS2812_RESET_PERIOD)
	{
		// count the number of timer periods
		ws2812b.timerPeriodCounter++;
	}
	else
	{
		ws2812b.timerPeriodCounter = 0;
		__HAL_TIM_DISABLE(&TIM1_handle);
		TIM1->CR1 = 0; // disable timer

		// disable the TIM2 Update
		__HAL_TIM_DISABLE_IT(&TIM1_handle, TIM_IT_UPDATE);
		// set TransferComplete flag
		ws2812b.transferComplete = 1;
	}*/

	ws2812b.timerPeriodCounter = 0;
	TIM15->CR1 = 0; // disable timer

	// disable the TIM2 Update IRQ
	__HAL_TIM_DISABLE_IT(&TIM15_handle, TIM_IT_UPDATE);

	// Set back 1,25us period
	TIM15->ARR = tim_period;

	// Generate an update event to reload the Prescaler value immediatly
	TIM15->EGR = TIM_EGR_UG;
	__HAL_TIM_CLEAR_FLAG(&TIM15_handle, TIM_FLAG_UPDATE);

	// set transfer_complete flag
	ws2812b.transferComplete = 1;

}



static void ws2812b_set_pixel(uint8_t row, uint16_t column, uint8_t red, uint8_t green, uint8_t blue)
{

	// Apply gamma
	red = gammaTable[red];
	green = gammaTable[green];
	blue = gammaTable[blue];


	uint32_t calcCol = (column*24);
	uint32_t invRed = ~red;
	uint32_t invGreen = ~green;
	uint32_t invBlue = ~blue;


#if defined(SETPIX_1)
	uint8_t i;
	uint32_t calcClearRow = ~((0x01<<row) << 0);
	for (i = 0; i < 8; i++)
	{
		// clear the data for pixel

		ws2812bDmaBitBuffer[(calcCol+i)] &= calcClearRow;
		ws2812bDmaBitBuffer[(calcCol+8+i)] &= calcClearRow;
		ws2812bDmaBitBuffer[(calcCol+16+i)] &= calcClearRow;

		// write new data for pixel
		ws2812bDmaBitBuffer[(calcCol+i)] |= (((((invGreen)<<i) & 0x80)>>7)<<(row+0));
		ws2812bDmaBitBuffer[(calcCol+8+i)] |= (((((invRed)<<i) & 0x80)>>7)<<(row+0));
		ws2812bDmaBitBuffer[(calcCol+16+i)] |= (((((invBlue)<<i) & 0x80)>>7)<<(row+0));
	}
#elif defined(SETPIX_2)
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		// Set or clear the data for the pixel

		if(((invGreen)<<i) & 0x80)
			varSetBit(ws2812bDmaBitBuffer[(calcCol+i)], row);
		else
			varResetBit(ws2812bDmaBitBuffer[(calcCol+i)], row);

		if(((invRed)<<i) & 0x80)
			varSetBit(ws2812bDmaBitBuffer[(calcCol+8+i)], row);
		else
			varResetBit(ws2812bDmaBitBuffer[(calcCol+8+i)], row);

		if(((invBlue)<<i) & 0x80)
			varSetBit(ws2812bDmaBitBuffer[(calcCol+16+i)], row);
		else
			varResetBit(ws2812bDmaBitBuffer[(calcCol+16+i)], row);

	}
#elif defined(SETPIX_3)
	ws2812bDmaBitBuffer[(calcCol+0)] |= (((((invGreen)<<0) & 0x80)>>7)<<row);
	ws2812bDmaBitBuffer[(calcCol+8+0)] |= (((((invRed)<<0) & 0x80)>>7)<<row);
	ws2812bDmaBitBuffer[(calcCol+16+0)] |= (((((invBlue)<<0) & 0x80)>>7)<<row);

	ws2812bDmaBitBuffer[(calcCol+1)] |= (((((invGreen)<<1) & 0x80)>>7)<<row);
	ws2812bDmaBitBuffer[(calcCol+8+1)] |= (((((invRed)<<1) & 0x80)>>7)<<row);
	ws2812bDmaBitBuffer[(calcCol+16+1)] |= (((((invBlue)<<1) & 0x80)>>7)<<row);

	ws2812bDmaBitBuffer[(calcCol+2)] |= (((((invGreen)<<2) & 0x80)>>7)<<row);
	ws2812bDmaBitBuffer[(calcCol+8+2)] |= (((((invRed)<<2) & 0x80)>>7)<<row);
	ws2812bDmaBitBuffer[(calcCol+16+2)] |= (((((invBlue)<<2) & 0x80)>>7)<<row);

	ws2812bDmaBitBuffer[(calcCol+3)] |= (((((invGreen)<<3) & 0x80)>>7)<<row);
	ws2812bDmaBitBuffer[(calcCol+8+3)] |= (((((invRed)<<3) & 0x80)>>7)<<row);
	ws2812bDmaBitBuffer[(calcCol+16+3)] |= (((((invBlue)<<3) & 0x80)>>7)<<row);

	ws2812bDmaBitBuffer[(calcCol+4)] |= (((((invGreen)<<4) & 0x80)>>7)<<row);
	ws2812bDmaBitBuffer[(calcCol+8+4)] |= (((((invRed)<<4) & 0x80)>>7)<<row);
	ws2812bDmaBitBuffer[(calcCol+16+4)] |= (((((invBlue)<<4) & 0x80)>>7)<<row);

	ws2812bDmaBitBuffer[(calcCol+5)] |= (((((invGreen)<<5) & 0x80)>>7)<<row);
	ws2812bDmaBitBuffer[(calcCol+8+5)] |= (((((invRed)<<5) & 0x80)>>7)<<row);
	ws2812bDmaBitBuffer[(calcCol+16+5)] |= (((((invBlue)<<5) & 0x80)>>7)<<row);

	ws2812bDmaBitBuffer[(calcCol+6)] |= (((((invGreen)<<6) & 0x80)>>7)<<row);
	ws2812bDmaBitBuffer[(calcCol+8+6)] |= (((((invRed)<<6) & 0x80)>>7)<<row);
	ws2812bDmaBitBuffer[(calcCol+16+6)] |= (((((invBlue)<<6) & 0x80)>>7)<<row);

	ws2812bDmaBitBuffer[(calcCol+7)] |= (((((invGreen)<<7) & 0x80)>>7)<<row);
	ws2812bDmaBitBuffer[(calcCol+8+7)] |= (((((invRed)<<7) & 0x80)>>7)<<row);
	ws2812bDmaBitBuffer[(calcCol+16+7)] |= (((((invBlue)<<7) & 0x80)>>7)<<row);
#elif defined(SETPIX_4)

	// Bitband optimizations with pure increments, 5us interrupts
	uint32_t *bitBand = BITBAND_SRAM(&ws2812bDmaBitBuffer[(calcCol)], row);

	*bitBand =  (invGreen >> 7);
	bitBand+=16;

	*bitBand = (invGreen >> 6);
	bitBand+=16;

	*bitBand = (invGreen >> 5);
	bitBand+=16;

	*bitBand = (invGreen >> 4);
	bitBand+=16;

	*bitBand = (invGreen >> 3);
	bitBand+=16;

	*bitBand = (invGreen >> 2);
	bitBand+=16;

	*bitBand = (invGreen >> 1);
	bitBand+=16;

	*bitBand = (invGreen >> 0);
	bitBand+=16;

	// RED
	*bitBand =  (invRed >> 7);
	bitBand+=16;

	*bitBand = (invRed >> 6);
	bitBand+=16;

	*bitBand = (invRed >> 5);
	bitBand+=16;

	*bitBand = (invRed >> 4);
	bitBand+=16;

	*bitBand = (invRed >> 3);
	bitBand+=16;

	*bitBand = (invRed >> 2);
	bitBand+=16;

	*bitBand = (invRed >> 1);
	bitBand+=16;

	*bitBand = (invRed >> 0);
	bitBand+=16;

	// BLUE
	*bitBand =  (invBlue >> 7);
	bitBand+=16;

	*bitBand = (invBlue >> 6);
	bitBand+=16;

	*bitBand = (invBlue >> 5);
	bitBand+=16;

	*bitBand = (invBlue >> 4);
	bitBand+=16;

	*bitBand = (invBlue >> 3);
	bitBand+=16;

	*bitBand = (invBlue >> 2);
	bitBand+=16;

	*bitBand = (invBlue >> 1);
	bitBand+=16;

	*bitBand = (invBlue >> 0);
	bitBand+=16;

#endif
}


void ws2812b_init()
{
	ws2812b_gpio_init();

	/*TIM2_init();
	DMA_init();*/


	DMA2_init();
	TIM15_init();


	// Need to start the first transfer
	ws2812b.transferComplete = 1;
}


void ws2812b_handle()
{
	if(ws2812b.startTransfer) {
		ws2812b.startTransfer = 0;
		WS2812_sendbuf();
	}

}

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

/**
 * The Brio Light and Sound Level Crossing (33675, very much discontinued!)
 * has a pushbutton, a reed switch (detects train's magnets passing over it),
 * two red LEDs, and a speaker.
 *
 * This replacement board attempts to improve on the audio quality, but keep the
 * basic operation almost the same. The old board joined the two switches to
 * a single pin. I'm keeping them separated so we can use the pushbutton for
 * additional functions. Like making the darn thing quiet. Right?
 *
 *
 * Basic operation -
 * Starts into a low power mode (WFE).
 * When either button is activated, plays a train crossing sound (TBD: only one
 * in memory? Multiple and cycle through them? Random selection?) and flashes
 * the LEDs alternately at a rate of about 2Hz.
 *
 * The flashing and sound should be about 5 seconds long to match the original.
 *
 * I'm using the length of the audio track (MP3 file) to determine how long
 * to flash lights.
 */




/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>

#include "audio.h"
#include "OneButton.h"
#include "signal_leds.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum LightSoundState_e{
	INACTIVE,
	ACTIVE
} LSState;

typedef enum SwitchState_e{
	RELEASED,
	PRESSED,
	LONG_PRESSED,
	DOUBLE_CLICKED
} SwState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//const uint8_t* mp3_file = (uint8_t*)(0x8010000); // offset of 64k from start of Flash
//const int filelen = 27059;

extern uint8_t audio1_data[];
extern size_t audio1_size;

extern uint8_t whistle_data[];
extern size_t whistle_size;

extern uint8_t chug_data[];
extern size_t chug_size;

#define LED_BLINK_TIME		(200u) // 200 millisecond blink

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;

UART_HandleTypeDef hlpuart1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim16;
DMA_HandleTypeDef hdma_tim6_up;

/* USER CODE BEGIN PV */
OneButton SW1;
LSState LightAndSoundState = INACTIVE;
volatile SwState ReedSwitchState = RELEASED;
volatile SwState ButtonState = RELEASED;

bool ReedSwitchLockout = false;

bool wake_stay_active = false; // set when exiting sleep to give a little time for a switch debounce
uint32_t last_wake_time = 0;
const uint32_t wake_active_time_ms = 100; // time to keep active after waking from sleep

uint32_t reed_sw_integrator = 0;
const uint32_t reed_sw_int_limit = 50; // Simple 50 ms debouncer using an integrator

unsigned int which_audio_file = 0;

volatile uint32_t LED_counter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

bool CanSleepNow(); // true iff nothing is happening and we're okay to enter low power mode
void ReEnablePLL(); // set the clock back to 64MHz after leaving deep sleep
// Pushbutton callbacks (OneButton library)
void button_pressed();
void button_long_pressed();
void button_double_clicked();
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
	SW1._initDone = false;
	// enforce system flash in address remap
	// this allows us to ignore vector table offset
	// since it defaults to zero
	__HAL_RCC_SYSCFG_CLK_ENABLE();
	SYSCFG->CFGR1 &= ~(SYSCFG_CFGR1_MEM_MODE);

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
  MX_DAC1_Init();
  MX_LPUART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM16_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  // Change low power mode to Stop 1
  // This mode has the high speed clock stopped
  // and almost all peripherals stopped. Low speed clocks
  // could keep running (although I'm not using them).
  // When returning from Stop mode, the PLL needs to be
  // reactivated. System clock will be HSISYS.
  __HAL_RCC_PWR_CLK_ENABLE();
  PWR->CR1 &= ~(PWR_CR1_LPMS_Msk);
  PWR->CR1 |= PWR_LOWPOWERMODE_STOP1; // Set PWR::CR1::LPMS to 0b001 (Stop 1)
  SCB->SCR |= (uint32_t)SCB_SCR_SLEEPDEEP_Msk;

  // Initialize Switch 1 (pushbutton) with OneButton library
  OneButtonInit(&SW1, GPIOA, 1, false, false);
  attachClick(&SW1, button_pressed);

  init_audio(&hdma_tim6_up, &htim6);

  // for now, not using the timers for LEDs. just on-off control.
  // assign LED1 and LED2 to outputs instead of alternate function

  signal_init();

  //LED1_GPIO_Port->BSRR = LED1_Pin | LED2_Pin; // LEDs are active low, set them high to turn off

  //LED1_GPIO_Port->MODER &= ~((GPIO_MODE)*(LED1_Pin*LED1_Pin));  // note: LED1_Pin is (1 << pin_number). So squaring it is (1 << (2*pin_number))
  //LED1_GPIO_Port->MODER |= (MODE_OUTPUT * (LED1_Pin*LED1_Pin));
  //LED1_GPIO_Port->MODER &= ~((GPIO_MODE)*(LED2_Pin*LED2_Pin));
  //LED1_GPIO_Port->MODER |= (MODE_OUTPUT * (LED2_Pin*LED2_Pin));

  // maximum speed (or current) to blinky the leds
  //LED1_GPIO_Port->OSPEEDR |= (GPIO_SPEED_FREQ_VERY_HIGH*(LED1_Pin*LED1_Pin)) | (GPIO_SPEED_FREQ_VERY_HIGH*(LED2_Pin*LED2_Pin));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // Enter stop mode by issuing WFE.
	  if(CanSleepNow())
	  {
		  //__WFE();

		  // We get here by waking up from stop mode.
		  // Make sure to clear any pending EXTI events.
		  EXTI->RPR1 = (1ul << 1ul) + (1ul << 5ul); // Clears bits 1 and 5. RPR1 is a write-1-to-clear

		  // Fix the clock. This will only need to be done if stop mode was
		  // actually entered. The WFE instruction could have been skipped
		  // if the event flag was already set.

		  // Check if the system clock isn't PLL.
		  if(__HAL_RCC_GET_SYSCLK_SOURCE() != (RCC_SYSCLKSOURCE_PLLCLK << RCC_CFGR_SWS_Pos))
		  {
			  ReEnablePLL();
			  // Keep awake a little while to ensure we catch a bouncing button
			  last_wake_time = HAL_GetTick();
			  wake_stay_active = true;
		  }


	  }

	  // If button debounced good, play sound and flash lights
	  if(ReedSwitchState == PRESSED || ButtonState == PRESSED)
	  {
		  // Acknowledge the pushbutton state (clear it)
		  if(ButtonState == PRESSED)
		  {
			  ButtonState = RELEASED;
		  }
		  if(!currently_playing())
		  {
			  // Lockout the reed switch. This prevents it from continuously playing
			  // while a magnet is parked on top of the switch.
			  if(ReedSwitchState == PRESSED) {
				  ReedSwitchLockout = true;
			  }
			  switch(which_audio_file) {
			  case 0:
			  	  start_playing_audio_file(audio1_data, audio1_size);
			  	  which_audio_file = 1;
			  	  break;
			  case 1:
				  start_playing_audio_file(whistle_data, whistle_size);
				  which_audio_file = 2;
				  break;
			  case 2:
				  start_playing_audio_file(chug_data, chug_size);
				  which_audio_file = 0;
				  break;

			  }

			  // Set one of the LEDs on. The systick callback will cause them
			  // both to blink, which should appear as alternating lights
			  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
			  LED_counter = 0;
		  }
	  }

	  // Clear reed switch lockout if it is not active / pressed
	  if(ReedSwitchState == RELEASED) {
		  ReedSwitchLockout = false;
	  }

	  /*
	  if(currently_playing())
	  {
		  // TODO: flash lights;
		  // use systick and our own local counter to determine when to blink
	  }
	  */

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 209700;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_7B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 7999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Ch4_7_DMAMUX1_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Ch4_7_DMAMUX1_OVR_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Ch4_7_DMAMUX1_OVR_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Amp_SD_GPIO_Port, Amp_SD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Amp_SD_Pin */
  GPIO_InitStruct.Pin = Amp_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Amp_SD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_Btn_Pin SW_Reed_Pin */
  GPIO_InitStruct.Pin = SW_Btn_Pin|SW_Reed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

bool CanSleepNow()
{
	// still in our keep awake period?
	if(wake_stay_active) {
		if (HAL_GetTick() - last_wake_time >= wake_active_time_ms) {
			// keep awake has elapsed, clear the flag
			wake_stay_active = false;
		} else {
			return false;
		}
	}

	// either of the buttons pressed?
	if ( (GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOA, SW_Btn_Pin) ) ||
			(GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOA, SW_Reed_Pin)) ) {
		return false;
	}

	// push button in the middle of a click state? the button might physically be up
	// but our button library hasn't triggered the callback yet
	if(!isIdle(&SW1))
	{
		return false;
	}

	// otherwise, just check if we're playing sound and flashing lights
	return (!currently_playing());
}

void ReEnablePLL() {
	// Set PLL multipliers, etc
	__HAL_RCC_PLL_DISABLE(); // just make sure it's really really turned off first
	while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) != 0U);

#if defined(RCC_PLLQ_SUPPORT)
	__HAL_RCC_PLL_CONFIG(RCC_PLLSOURCE_HSI,
							RCC_PLLM_DIV1,
							8,
							RCC_PLLP_DIV2,
							RCC_PLLQ_DIV2,
							RCC_PLLR_DIV2);
#else /* !RCC_PLLQ_SUPPORT */
	__HAL_RCC_PLL_CONFIG(RCC_PLLSOURCE_HSI,
							RCC_PLLM_DIV1,
							8,
							RCC_PLLP_DIV2,
							RCC_PLLR_DIV2);
#endif /* RCC_PLLQ_SUPPORT */

	__HAL_RCC_PLL_ENABLE();
	__HAL_RCC_PLLCLKOUT_ENABLE(RCC_PLLRCLK);

	// wait until PLL is ready
	while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == 0U);

	// switch system clock to PLL
	MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_SYSCLKSOURCE_PLLCLK);

	// wait until it has switched over
	while(__HAL_RCC_GET_SYSCLK_SOURCE() != (RCC_SYSCLKSOURCE_PLLCLK << RCC_CFGR_SWS_Pos));
}

void main_systick_callback() {
	// OneButton library for the pushbutton
	if(SW1._initDone) {
		tick(&SW1);
	}

	// simple integrator debouncer for the reed switch
	if(GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOA, SW_Reed_Pin)) {
		if(reed_sw_integrator < reed_sw_int_limit) {
			reed_sw_integrator++;
		}
	} else {
		if(reed_sw_integrator > 0) {
			reed_sw_integrator--;
		}
	}

	if(PRESSED == ReedSwitchState) {
		if(0 == reed_sw_integrator) {
			ReedSwitchState = RELEASED;
		}
	} else {
		if( reed_sw_int_limit <= reed_sw_integrator) {
			ReedSwitchState = PRESSED;
		}
	}

	// LED blink
	if(currently_playing()) {
		LED_counter++;
		if(LED_counter >= LED_BLINK_TIME) {
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin | LED2_Pin);
			LED_counter = 0;
		}
	}
}

void button_pressed() {
	ButtonState = PRESSED;
}
void button_long_pressed() {
	ButtonState = LONG_PRESSED;
}
void button_double_clicked() {
	ButtonState = DOUBLE_CLICKED;
}

void audio_finished() {
	// ensure LEDs are turned off
	LED1_GPIO_Port->BSRR = LED1_Pin | LED2_Pin;
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

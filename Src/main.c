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
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdarg.h>
#include <string.h>
#include <AS7343.h>
#include <C12880MA.h>
#include <cJSON.h>
#include <to_ESP32.h>
#include <I2C1.h>
#include <I2C2.h>
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

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void TIM3_Init(void);
void UART_printf(char *format, ...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int pressed = 0;
uint32_t sensor_clk_cycles;
//int start_conv_flag = 0; // for C12880MA routine
//int start_sig_done = 0;
uint16_t ADC_readings[200] = {0};
int eos;

char *txb_ptr = 0; // for transmitting to ESP32 via I2C
//int C12880MA_done = 0; // indicates entire measurement routine is done

typedef enum {
  NOT_PRESSED,
  SHORT_PRESS,
  LONG_PRESS,
} BUTTON_PRESS_T;

BUTTON_PRESS_T BUTTON_PRESS = NOT_PRESSED;

static const uint8_t banner[] =
		"\033[0m\033[2J\033[1;1H"
		"-----------------------------------------------------------------\r\n"
		"This STM interfaces with the following sensors:\r\n"
		"   >> ams OSRAM AS7343 Multispectral Sensor\r\n"
		"   >> HAMAMATSU C12880MA Hyperspectral Sensor\r\n"
		"\r\n"
		"Only the AS7343 is ready for use. To initiate a measurement,\r\n"
		"use the user button (blue button on STM board) as follows:\r\n"
		"   >> short press: take one measurement\r\n"
		"   >>  long press: take continuous, periodic measurements\r\n"
		"      every second. short press again to stop.\r\n"
		"\r\n"
		"Results will be printed.\r\n"
		"-----------------------------------------------------------------\r\n"
		"\r\n";

uint64_t AS7343_reads_cumm[12];
uint16_t AS7343_reads_curr[12];
uint16_t C12880_readings[288];
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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Transmit(&huart2, banner, sizeof(banner)-1, 1000);

  uint8_t pressed_msg[] = "\r\n\nButton was pressed. Taking measurements...\r\n";


  uint8_t vals[] = "\r\n  val";
  uint8_t labels[] = "\r\n+-----+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+"
		  	  	  	     "\r\n   nm |   405   |   425   |   450   |   475   |   515   |   550   |   555   |   600   |   640   |   690   |   745   |   855   "
  	  	  	     	   "\r\n   ch |   F1    |   F2    |   FZ    |   F3    |   F4    |   F5    |   FY    |   FXL   |   F6    |   F7    |   F8    |   NIR   \r\n";
  uint8_t divider[] = "\r\n+-----+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+";
  int reps = 20;
//  UART_printf("Initializing ESP32 comm...");
//  Init_I2C2();
//  UART_printf("DONE\r\n");
//
//  UART_printf("Initializing AS7343...");
//  Init_AS7343();
//  UART_printf("DONE\r\n");

  UART_printf("Initializing C12880MA...");
  Init_C12880MA();
  UART_printf("Now ready.\r\n");




//  pressed = 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  if (sensor_clk_cycles >= 2000000) { // should be ~every second
//		  UART_printf("2000000 sensor CC (1 sec) has passed.\r\n");
//		  NVIC_DisableIRQ(EXTI4_IRQn); // Interrupt disabled for PB4
//		  sensor_clk_cycles = 0; // reset
//		  UART_printf("Interrupt disabled for PB4.\r\n");
//	  }
	  if (BUTTON_PRESS == SHORT_PRESS) {
//		  NVIC_EnableIRQ(EXTI4_IRQn); // Interrupt enabled for PB4
		  ///////////////////////////////////////////////////////////////////////////////////
		  ///////////////////////// ADC CHARACTERIZATION ROUTINE ////////////////////////////
		  ///////////////////////////////////////////////////////////////////////////////////
//		  UART_printf("Button pressed. Initiating measurement 6000 ADC Measurements...\r\n");
//		  BUTTON_PRESS = NOT_PRESSED;
//		  int adc_char_done = 0;
//		  ADC_readings[200] = 0;
//		  sensor_clk_cycles = 0;
//		  ADC1->CR2 |= (1 << 0); // enable ADC
//		  NVIC_EnableIRQ(EXTI4_IRQn); // Interrupt enabled for PB4
//		  UART_printf("\r\n(%d) All initialization done. Recording ADC\r\n", sensor_clk_cycles);
//
//		  delay_ms(1);
//
//		  NVIC_DisableIRQ(EXTI4_IRQn); // Interrupt disabled for PB4
//		  ADC1->CR2 &= ~(1 << 0); // disable ADC
//		  UART_printf("\r\n(%d) All initialization done. Recording ADC\r\n", sensor_clk_cycles);
//		  for (int i = 0; i < 200; ++i) {
//			  UART_printf("\r\n%3d: %4d", i, ADC_readings[i]);
//		  }
//
//		  uint16_t min = minValue(ADC_readings, 200);
//		  uint16_t max = maxValue(ADC_readings, 200);
//		  uint32_t sum = arraySum(ADC_readings, 200);
//		  UART_printf("\r\nSum: %d", sum); // cant print variable avg, ewan kung baket
//		  UART_printf("\r\nMin: %7d", min);
//		  UART_printf("\r\nMax: %7d", max);
//		  UART_printf("\r\nRange: %5d", max-min);

		  ///////////////////////////////////////////////////////////////////////////////////
		  ///////////////////////////// NEW C12880MA ROUTINE ////////////////////////////////
		  ///////////////////////////////////////////////////////////////////////////////////
		  BUTTON_PRESS = NOT_PRESSED;
		  UART_printf("Button pressed. Initiating measurement with C12880MA...\r\n");
		  sensor_clk_cycles=0;
		  eos = 0;
		  C12880MA_ST(1000);

		  while (eos != 1) {}
		  // disable and deinit peripherals
		  // TIM2, TIM1 already disabled (one-pulse)
		  NVIC_DisableIRQ(TIM1_CC_IRQn);
		  NVIC_DisableIRQ(EXTI4_IRQn);
		  NVIC_DisableIRQ(EXTI3_IRQn);
		  eos = 0;
		  UART_printf("Program finished.\r\n");
		  for (int i=0; i<288; ++i) {
			UART_printf("%5d,", C12880_readings[i]);
		  }
		  ///////////////////////////////////////////////////////////////////////////////////
		  /////////////////////////////// C12880MA ROUTINE //////////////////////////////////
		  ///////////////////////////////////////////////////////////////////////////////////
//		  clear_C12880MA_readings(C12880_readings);
////		  C12880MA_done = 0;
//		  start_sig_done = 0;
//		  start_conv_flag = 0;
//
//		  UART_printf("Button pressed. Initiating measurement with C12880MA...\r\n");
////		  C12880MA_start(C12880_readings, 5000, sensor_clk_cycles);
//
//		  sensor_clk_cycles = 0; // reset to start count
//
//		  TIM1->CNT = ~(0xFFFFFFFF); // zero the counter registers
//		  TIM2->CNT = ~(0xFFFFFFFF); // zero the counter registers
//		  TIM1->ARR = 85; // wait time
//		  TIM2->ARR = 20000; // integration time (tint) to 10 ms
//
//		  ADC1->CR2 |= (1 << 0); // enable ADC
//		  ADC1->CR2 |= (0b01 << 28); // enable rising edge trigger detection
//		  // NOTE: we are enabling edge detection here, which will start the ADC
//		  // immediately, but we will not be saving the values until later
//		  UART_printf("(%u) Initializations done. Starting process\r\n", sensor_clk_cycles);
//
//		  // --------------------- START SIGNAL ---------------------
//		  TIM2->CR1 |= (1 << 0); // start timer
//		  UART_printf("(%u) START asserted. Waiting for on_time to complete.\r\n", sensor_clk_cycles);
//
//		  while (!start_sig_done) {} // wait for ST pulse to finish. this flag is set by the timer interrupt handler
//
//		  UART_printf("(%u) on_time done: START deasserted. Waiting for values to be ready, then reading ADC\r\n", sensor_clk_cycles);
//
//		  sensor_clk_cycles = 0;
//		  NVIC_EnableIRQ(EXTI4_IRQn); // enable interrupt from TRG
//
//		  // the rest of the routine is handled by EXTI4 handler
//		  delay_ms(1); // ensure done
//
//		  for (int i=0; i<288; ++i) {
//		  	UART_printf("ch %3d: %5d\r\n", i+1, C12880_readings[i]);
//		  }
//		  uint16_t min = minValue(C12880_readings, 288);
//		  uint16_t max = maxValue(C12880_readings, 288);
//		  uint32_t sum = arraySum(C12880_readings, 288);
//		  UART_printf("\r\nSum: %d", sum); // cant print variable avg, ewan kung baket
//		  UART_printf("\r\nMin: %7d", min);
//		  UART_printf("\r\nMax: %7d", max);
//		  UART_printf("\r\nRange: %5d", max-min);

		  ///////////////////////////////////////////////////////////////////////////////////
		  //////////////////////////////// AS7343 ROUTINE ///////////////////////////////////
		  ///////////////////////////////////////////////////////////////////////////////////
//		  BUTTON_PRESS = NOT_PRESSED;
// 		  HAL_UART_Transmit(&huart2, pressed_msg, sizeof(pressed_msg)-1, 1000);
//
// 		  clear_AS7343_readings(AS7343_reads_curr);
//
//// 		  AS7343_default_config();
//
// 		  AS7343_set_ATIME(1);
// 		  AS7343_set_ASTEP(26999);
// 		  AS7343_set_AGAIN(AS7343_GAIN_256X);
//
// 		  uint8_t ATIME = AS7343_get_ATIME();
// 		  uint16_t ASTEP = AS7343_get_ASTEP();
// 		  uint8_t AGAIN = AS7343_get_AGAIN();
// 		  UART_printf("\r\nATIME = %3d", ATIME);
// 		  UART_printf("\r\nASTEP = %3d", ASTEP);
// 		  UART_printf("\r\nAGAIN = %3d\n", AGAIN);
//
// 		  UART_printf("\r\nNow finding raw spectrum (unoptimized)...\n");
//
// 		  // averaging {reps} number of repeated measurements
// 		  for (int rep = 0; rep < reps; ++rep) {
// 	 		  AS7343_get_raw_spectrum(AS7343_reads_curr);
// 	 		  // print iteration
// 	 		  UART_printf("\r\n   %2d", rep+1);
// 	 		  for (int i = 0; i < 12; ++i) {
// 	 			  UART_printf(" | %7d", AS7343_reads_curr[i]); // print
// 	 			  AS7343_reads_cumm[i] += (uint64_t) (AS7343_reads_curr[i]); // add
// 	 		  }
// 		  }
// //		  AS7343_readings[0] = AS7343_read_2b(AS7343_CH12_DATA_L);
// //		  AS7343_readings[1] = AS7343_read_2b(AS7343_CH6_DATA_L);
//
// 		  HAL_UART_Transmit(&huart2, divider, sizeof(divider)-1, 1000);
// 		  UART_printf("\r\n  avg");
// 		  for (int i = 0; i < 12; ++i) {
// 			  UART_printf(" | %7d", (int) ((int)AS7343_reads_cumm[i])/reps);
// 		  }
// 		  HAL_UART_Transmit(&huart2, labels, sizeof(labels)-1, 1000);


// 		  UART_printf("\nNow finding raw spectrum (optimized). Resetting first... ");
// 		  AS7343_reset();
// 		  AS7343_default_config();
// 		  UART_printf("reset and config done\r\n");
// 		  AS7343_get_raw_spectrum_optimized(AS7343_readings, 10);
// 		  AS7343_readings[0] = AS7343_read_2b(AS7343_CH12_DATA_L);
// 		  AS7343_readings[1] = AS7343_read_2b(AS7343_CH6_DATA_L);
//
// 		  HAL_UART_Transmit(&huart2, vals, sizeof(vals)-1, 1000);
// 		  for (int i = 0; i < 12; ++i) {
// 			  UART_printf(" | %5d", AS7343_readings[i]);
// 		  }
// 		  HAL_UART_Transmit(&huart2, labels, sizeof(labels)-1, 1000);

		  ///////////////////////////////////////////////////////////////////////////////////
		  //////////////////////////////// ESP32 ROUTINE ////////////////////////////////////
		  ///////////////////////////////////////////////////////////////////////////////////
//		  BUTTON_PRESS = NOT_PRESSED;
//		  UART_printf("Button pressed. Sending to ESP32\r\n");
//		  uint8_t sample_msg[] = "trial1234";
//		  uint8_t * msg_ptr = &sample_msg[0];
//		  I2C1_send_buf(ESP32_SLAVE_ADDR_WRITE, msg_ptr, sizeof(sample_msg));

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 49;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10009;
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
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim3, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
  sConfigOC.Pulse = 5000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart2.Init.BaudRate = 38400;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN) {
	if( GPIO_PIN == GPIO_PIN_13 ) {
		BUTTON_PRESS = SHORT_PRESS;
		delay_ms(100); // debouncing
	} else {
	      __NOP();
	}
}

void TIM3_Init(void) {
	RCC->APB1ENR |= (1 << 1); 	// enable TIM3 (16-bit counter)
								// this is configured to 100 MHz (APB1 Timer CLK)

	TIM3->PSC |= (9999 << 0); // prescaler = 99999 + 1 so that f_eff = 10 kHz

	TIM3->CR1 &= ~(1 << 4); // upcounter
	TIM3->CR1 |=  (1 << 3); // one-pulse mode: stop counter at update event (UEV)

	TIM3->CCR3 	&= ~(0xFFFFFFFF << 0); 	// this sets t_delay before output is toggled
	TIM3->CCMR2 |=  (1 << 7); 			// OC3CE: OC3 is cleared on HIGH level of ETRF
	TIM3->CCMR2 |=  (0b111 << 4); 		// OC3M: set OC3 to HIGH at CCR3 match (posedge of ST), then low at ARR match (negedge of ST)
	TIM3->CCMR2 &= ~(1 << 2);			// OC3FE: CC3 behaves normally; TRGI will not act as compare match
	TIM3->CCMR2 &= ~(0b11 << 0); 		// CC3S: capture/compare 3 configured as OUTPUT
	TIM3->CCER  &= ~(1 << 9);			// CC3P: OC3 active high
	TIM3->CCER 	|=  (1 << 8); 			// CC3E: OC3 is output on corresponding pin (PB10)
	TIM3->ARR 	|=  (10000); 			// pulse duration default: 5ms (x0.5µs)

	TIM3->CR2 |= (0b010 << 4); // Master Mode: UEV as TRGO

	TIM3->DIER |= (1 << 0); // Update Interrupt Enable

	TIM3->CNT &= ~(0xFFFFFFFF); // set count to 0 (TIM2 is 32-bit)
	TIM3->CR1 &= ~(1 << 0); // disable TIM2 for now
}

void TIM1_CC_IRQHandler(void) {
  TIM1->DIER &= ~(1 << 3); // disable interrupt on compare
//  GPIOB->ODR |= (1 << 5); // set output as HIGH
  ADC1->CR2 |= (1 << 0);
  NVIC_EnableIRQ(EXTI4_IRQn); // enable saving of values
}

void EXTI4_IRQHandler(void) {
	EXTI->PR |= (1 << 4); // clear flag
	if (sensor_clk_cycles <= 6000) {
		ADC_readings[sensor_clk_cycles] = ADC1->DR;
	}
	++sensor_clk_cycles;
//	if (start_sig_done && (sensor_clk_cycles == 10)) {
//		start_conv_flag = 1; // we can now start storing the result of AD conversion
//	}
//	if (start_sig_done && start_conv_flag && (sensor_clk_cycles != 298)) {
//		C12880_readings[sensor_clk_cycles-10] = ADC1->DR;
//	}
//	if (start_sig_done && start_conv_flag && (sensor_clk_cycles >= 298)) {
//	  	UART_printf("(%d) Finished. Turning off peripherals.\r\n", sensor_clk_cycles);
//
//	  	NVIC_DisableIRQ(EXTI4_IRQn);
//	  	ADC1->CR2 &= ~(0b11 << 28); // disable rising edge trigger detection
//	  	ADC1->CR2 &= ~(1 << 0);
//	}
}

void EXTI3_IRQHandler(void) {
	EXTI->PR |= (1 << 3); // clear flag
//	GPIOB->ODR &= ~(1 << 5); // toggle output low
	eos = 1;
}

void UART_printf(char *format, ...) {
	char buffer[256];
	va_list args;
	va_start(args, format);
	vsnprintf(buffer, sizeof(buffer), format, args);
	va_end(args);
	HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 1000);
}

void delay_ms(uint16_t ms) {
	uint32_t tick = 0;
	uint32_t tick_per_ms = 5000;
	while (ms > 0) {
		if (tick >= tick_per_ms) {
			--ms;
			tick=0;
		} else { ++tick; }
	}
}

void clear_C12880MA_readings(uint16_t C12880_readings[288]) {
  for (int i=0; i < 288; ++i) {
    C12880_readings[i] = 0;
  }
}

void clear_AS7343_readings(uint64_t AS7343_readings[12]) {
  for (int i=0; i < 12; ++i) {
    AS7343_readings[i] = 0;
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

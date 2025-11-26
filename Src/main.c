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
#include <stdint.h>
#include <inttypes.h>
#include <stdarg.h>
#include <string.h>
#include <ext_btn.h>
#include <AS7343.h>
//#include <C12880MA.h>
//#include <cJSON.h>
//#include <to_ESP32.h>
//#include <I2C1.h>
//#include <I2C2.h>
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void UART_printf(char *format, ...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//uint32_t sensor_clk_cycles;
button_t button;


static const uint8_t banner[] =
		"\033[0m\033[2J\033[1;1H"
		"-----------------------------------------------------------------\r\n"
		"This STM interfaces with the asm OSRAM AS7343 Multispectral Sensor\r\n"
		"\r\n"
		"To initiate a measurement, press the external black button.\r\n"
		"   >> short press: take one measurement\r\n"
		"   >>  long press: take one measurement and save as baseline\r\n"
		"\r\n"
		"Output will be printed.\r\n"
		"-----------------------------------------------------------------\r\n"
		"\r\n";

uint64_t AS7343_reads_cumm[12];
uint16_t AS7343_reads_curr[12];
float		 AS7343_reads_abs[12];
float 	 AS7343_reads_blank[12] = {
		0,		// 400nm
		0,		// 425nm
		20143,// 450nm
		12158,// 475nm
		15981,// 515nm
		4979, // 550nm
		21873,// 555nm
		15797,// 600nm
		9618,	// 640nm
		2649,	// 690nm
		8456,	// 745nm
		3166,	// 855nm
};

AS7343_reading AS7343_readings;
//// C12880MA-related variables
// uint16_t C12880MA_readings[288];
// uint16_t ADC_readings[300];
// uint16_t readings[300];
int eos;
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
  /* USER CODE BEGIN 2 */

  HAL_UART_Transmit(&huart2, banner, sizeof(banner)-1, 1000);

  UART_printf("Initializing external button...");
  EXT_BTN_init(button);
	UART_printf("DONE\r\n");

	UART_printf("Initializing AS7343...");
	Init_AS7343();
	UART_printf("DONE\r\n");

  //  UART_printf("Initializing C12880MA...");
  //  Init_C12880MA(C12880MA_readings);
  //  UART_printf("Now ready.\r\n");

  //  UART_printf("Initializing ESP32 comm...");
  //  Init_I2C2();
  //  UART_printf("DONE\r\n");


  uint8_t pressed_msg[] = "\r\n\nButton was pressed. Taking measurements...\r\n";
//  int reps = 20;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (button.status == SHORT_PRESS) {
		  ///////////////////////////////////////////////////////////////////////////////////
		  ///////////////////////////// NEW C12880MA ROUTINE ////////////////////////////////
		  ///////////////////////////////////////////////////////////////////////////////////
//		  BTN_STATUS = NOT_PRESSED;
//		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
//		  UART_printf("\r\n\nButton pressed. Initiating measurement with C12880MA...\r\n");
//		  sensor_clk_cycles=0;
//		  eos = 0;
//		  C12880MA_ST(1000);
//
//		  while (eos != 1) {}
//
//		  // disable and deinit peripherals
//		  // TIM2, TIM1 already disabled (one-pulse)
//			// ADC1, DMA2 automatically disabled
//		  NVIC_DisableIRQ(TIM1_CC_IRQn);
//		  NVIC_DisableIRQ(EXTI3_IRQn);
//
//		  eos = 0;
//
//		  UART_printf("Program finished.\r\n");
//		  for (int i=0; i<288; ++i) {
//			  UART_printf("%4d,", C12880MA_readings[i]);
//		  }
//		  UART_printf("DONE.");

		  ///////////////////////////////////////////////////////////////////////////////////
		  //////////////////////////////// ESP32 ROUTINE ////////////////////////////////////
		  ///////////////////////////////////////////////////////////////////////////////////
	  	EXT_BTN_reset(button);
	  	button.status = NOT_PRESSED;

 		  HAL_UART_Transmit(&huart2, pressed_msg, sizeof(pressed_msg)-1, 1000);

 		  clear_AS7343_readings(AS7343_readings);

 		  UART_printf("Reading buffer cleared. Configuring AS7343... ");

 		  AS7343_default_config();

 		  AS7343_set_ATIME(9);
 		  AS7343_set_ASTEP(11999);
 		  AS7343_set_AGAIN(AS7343_GAIN_256X);

 		  UART_printf("DONE\r\n");

 		  // UART_printf("\r\nATIME = %3d", ATIME);
 		  // UART_printf("\r\nASTEP = %3d", ASTEP);
 		  // UART_printf("\r\nAGAIN = %3d\n", AGAIN);
 		  // UART_printf("\r\nT_int = %f\n", ((float) ATIME+1)*((float) ASTEP+1)*0.00278);

 		  UART_printf("\r\nNow finding raw spectrum (unoptimized)...\r\n");

 		  AS7343_get_raw_spectrum(AS7343_readings);

 		  for (int i = 0; i < 10; ++i) { // we excluded last 2 channels which are out of range of reference MSI spectro
 			  UART_printf("%d\r\n", AS7343_readings.raw_count[i]);
 		  }
 		  //print parameters right after
 		  UART_printf("%d\r\n", 2 << (AS7343_readings.ADC_GAIN-2));
 		  UART_printf("%f\r\n", AS7343_readings.t_int);

	  } else if (button.status == LONG_PRESS) {
	  	button.status = NOT_PRESSED;
	  	UART_printf("\r\nLong press detected. No action.\r\n");
	  	EXT_BTN_reset(button);
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

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//------------------------------------------------------------
// PUSH BUTTON-RELATED INTERRUPT HANDLERS
//------------------------------------------------------------
void EXTI1_IRQHandler(void) {
	EXTI->PR |= (1 << 1); // clear flag
	button.status = SHORT_PRESS;

	if (button.event == BTN_RELEASE) { // if previously released
		button.event = BTN_PRESS;

		// wait for button release
		EXTI->RTSR |=  (1 << 1); // enable rising edge detection
		EXTI->FTSR &= ~(1 << 1); // disable falling edge detection

		TIM5->CNT = 0;
		TIM5->CR1 |= (1 << 0); // start timer
	}
	else if (button.event == BTN_PRESS) { // if previously pressed
		button.event = BTN_RELEASE;
		// revert to button press detection
		EXTI->RTSR &= ~(1 << 1); // no rising edge detection
		EXTI->FTSR |=  (1 << 1); // enable falling edge detection

		if (button.long_press == 1) {
			button.status = LONG_PRESS;
			TIM5->CR1 &= ~(1 << 0); // stop timer
		} else {
			button.status = SHORT_PRESS;
			TIM5->CR1 &= ~(1 << 0); // stop timer
		}
	}
}

void TIM5_IRQHandler(void) {
	TIM5->SR &= ~(1 << 0); // clear interrupt flag
	button.long_press = 1; // raise long press flag
}

//------------------------------------------------------------
// C12880MA-RELATED INTERRUPT HANDLERS
//------------------------------------------------------------

void TIM1_CC_IRQHandler(void) {
  TIM1->DIER &= ~(1 << 3); // disable interrupt on compare
  // GPIOB->ODR |= (1 << 5); // set output as HIGH
  ADC1->CR2 |= (1 << 0); // enable ADC
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
//  HAL_ADC_Start_DMA(&hadc1, readings, 288);
  // NVIC_EnableIRQ(EXTI4_IRQn); // enable saving of values
}

void DMA2_Stream0_IRQHandler(void) {
	DMA2->LIFCR = 0xFFFFFFFF; // clear all interrupt flags
}

/**
 * Detects EOS signal and raises flag
 * */
void EXTI3_IRQHandler(void) {
	EXTI->PR |= (1 << 3); // clear flag
//	GPIOB->ODR &= ~(1 << 5); // toggle output low
	ADC1->CR2 &= ~(1 << 0); // disable ADC
	eos = 1;
}


//------------------------------------------------------------
// OTHER FUNCTIONS
//------------------------------------------------------------
void UART_printf(char *format, ...) {
	char buffer[256];
	va_list args;
	va_start(args, format);
	vsnprintf(buffer, sizeof(buffer), format, args);
	va_end(args);
	HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 1000);
}

void delay_ms(uint16_t ms) { // im not sure if this works as intended
	uint32_t tick = 0;
	uint32_t tick_per_ms = 5000;
	while (ms > 0) {
		if (tick >= tick_per_ms) {
			--ms;
			tick=0;
		} else { ++tick; }
	}
}

void clear_C12880MA_readings(uint16_t C12880MA_readings[288]) {
  for (int i=0; i < 288; ++i) {
    C12880MA_readings[i] = 0;
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

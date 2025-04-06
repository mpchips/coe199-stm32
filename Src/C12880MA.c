/**
 * C12880MA.c
 *
 *  Created on: Feb 6, 2025
 *      Author: moreypiatos
 *
 *
 * PIN LAYOUT
 *    PC7  : LED control (unused)
 *    PA9  : video output
 *    PA8  : CLK out (hardwired by STM32)
 *    PB10 : START
 *    PB4  : TRIGGER
 *    PB5  : end-of-scan (EOS)
 * 	  PA7  : video output (must be shorted with PA9)
 *    PA11 : TRIGGER (must be shorted with PB4)
 * 
 * NOTES ON REDUNDANCY OF PINS
 * >> Pins of video output and TRIGGER are redundant. The first
 * set of pins are side-by-side for easy connection, but they
 * are also shorted to additional pins that are capable of the
 * necessary special functionality (ADC and EXTI11, respectively)
 * required for those signals. Specific functionalities are tied 
 * by default to certain pins in the NUCLEO development board. 
 * The first set of pins, which were selected due to proximity,
 * cannot be used with the aforementioned functionalities, so
 * additional pins were configured to allow for it. It is
 * necessary to short the corresponding pins to ensure proper
 * behavior.
 * 
 */


#include <stdio.h>
#include <stm32f4xx.h>
#include <stm32f411xe.h>
#include <C12880MA.h>
#include <main.h>

void Init_C12880MA() {

	  UART_printf("\r\n\tCLKOUT...");
	  Init_C12880MA_CLKOUT();
	  UART_printf("DONE");
	  UART_printf("\r\n\tGPIO...");
	  Init_C12880MA_GPIO();
	  UART_printf("DONE");
	  UART_printf("\r\n\tEXTI...");
	  Init_C12880MA_EXTI();
	  UART_printf("DONE");
	  UART_printf("\r\n\tADC...");
	  Init_C12880MA_ADC();
	  UART_printf("DONE");
	  UART_printf("\r\n\tTIM2...");
	  Init_TIM2();
	  UART_printf("\r\nDONE\r\n");
}

void Init_C12880MA_CLKOUT() {

	/* CLOCK CONFIGURATION
	 *
	 * PLLCLK source: HSI
	 *  - multiplier: x200
	 *  - master pre-scaler: /8
	 *  - PLL pre-scaler: /4
	 *  - f_eff = 100MHz
	 * ADCCLK source: PLLCLK
	 *  - AHB pre-scaler: /1
	 *  - APB1 pre-scaler: /2
	 *  - f_eff: 50MHz
	 * CLKOUT source: HSE
	 *  - pre-scaler: /4
	 *  - f_eff = 2 MHz
	 * SYSCLK source: PLLCLK
	 *
	 * Sensor CLK: 2 MHz
	 * ADC CLK: 50MHz (much, much faster)
	 *
	 * */

	RCC->CR |= (1 << 24); // PLL ON
	RCC->CR |= (1 << 0);  // enable HSI
	RCC->CR |= (1 << 16); // enable HSE

	// idk if this is needed kinuha ko sa internet habang may tino-troubleshoot
	FLASH->ACR |= (1 << 8); // enable pre-fetch
	FLASH->ACR |= (0x1 << 0); // flash wait state = 1

	RCC->PLLCFGR &= ~(1 << 22); // PLL source = HSI

	RCC->PLLCFGR &= ~(1 << 17); // PLLP division factor = /4
	RCC->PLLCFGR |=  (1 << 16);

	RCC->PLLCFGR &= ~(1 << 14);   // PLLN multiplication factor = x200
	RCC->PLLCFGR &= ~(0xFF << 6); // clear first
	RCC->PLLCFGR |=  (200 << 6);  // set d200 = 0b1100_1000 = 0xC8

	RCC->PLLCFGR &= ~(0b111111 << 0); // clear first
	RCC->PLLCFGR |=  (0b001000 << 0); // PLLM main division factor = /8

//	RCC->CFGR &= ~(1 << 29); // MCO2 pre-scaler: NONE

	RCC->CFGR |=  (0b110 << 24); // MCO1 pre-scaler: division by 4

	RCC->CFGR |=  (0b11 << 30); // select SYSCLK as MCO2 source (UNCOMMENT. FOR TESTING ONLY)
//	RCC->CFGR &= ~(0b11 << 21); // select HSI as MCO1 source

	RCC->CFGR |=  (0b10 << 21); // select HSE as MCO1 source
//	RCC->CFGR |=  (0b11 << 21); // select PLL as MCO1 source

	RCC->CFGR &= ~(0b111 << 13); // APB2 pre-scaler = none, 50 MHz

	RCC->CFGR &= ~(0b111 << 10);
	RCC->CFGR |=  (0b100 << 10); // APB1 pre-scaler = /2, 25 MHz

	RCC->CFGR &= ~(1 << 7); // AHB pre-scaler = none, 50 MHz

	RCC->CFGR |=  (1 << 1); // use PLL as system clock
	RCC->CFGR &= ~(1 << 0);

//	RCC->CFGR &= ~(11 << 0); // use HSI as system clock

} // Init_C12880MA_CLKOUT()

void Init_C12880MA_GPIO() {

	RCC->AHB1ENR |= (1 << 0); // enable GPIOA
	RCC->AHB1ENR |= (1 << 1); // enable GPIOB
	RCC->AHB1ENR |= (1 << 2); // enable GPIOC

	// C12880MA LED pin (PC7)
	GPIOC->MODER &= ~(1 << 15); // set PC7 as output mode
	GPIOC->MODER |=  (1 << 14);

	GPIOC->OTYPER &= ~(1 << 7); // push-pull type

	GPIOC->OSPEEDR |=  (1 << 15); // fast speed output
	GPIOC->OSPEEDR &= ~(1 << 14);

	GPIOC->PUPDR &= ~(1 << 15); // no pull-up, pull-down
	GPIOC->PUPDR &= ~(1 << 14);

	GPIOC->ODR &= ~(1 << 10); // set as initially LOW

	// C12880MA video pin (PA9) <- SEE Init_ADC()
	GPIOA->MODER |= (0x11 << 18); // set PA9 as analog mode

	// C12880MA video pin (PA7) << source of analog for ADC
	GPIOA->MODER |= (0x11 << 14); // set PA7 as analog mode

	// C12880MA CLK pin (PA8)
	GPIOA->MODER |= (1 << 17); // set PA8 as alternate function
	GPIOA->MODER &= ~(1 << 16);

	GPIOA->AFR[1] &= ~(0x00000001); // set to MCO1 function (AF0 = 0b0000)

	GPIOA->OTYPER &= ~(1 << 8);

	GPIOA->OSPEEDR |= (0b11 << 16); // set as high speed output

	// C12880MA start pin (PB10)
	GPIOB->MODER &= ~(1 << 21); // set PB10 as output mode
	GPIOB->MODER |= (1 << 20);

	GPIOB->OTYPER &= ~(1 << 10); // push-pull type

	GPIOB->OSPEEDR |= (1 << 21); // high speed output
	GPIOB->OSPEEDR |= (1 << 20);

	GPIOB->PUPDR &= ~(1 << 21); // no pull-up, pull-down
	GPIOB->PUPDR &= ~(1 << 20);

	GPIOB->ODR &= ~(1 << 10); // set as initially LOW

	// C12880MA TRG pin (PB4)
	GPIOB->MODER &= ~(1 << 9); // set PB4 as input mode
	GPIOB->MODER &= ~(1 << 8);

	// C12880MA EOS pin (PB5)
	GPIOB->MODER &= ~(1 << 11); // set PB5 as input mode
	GPIOB->MODER &= ~(1 << 10);

} // Init_C12880MA_GPIO()

void Init_C12880MA_EXTI() {
	RCC->APB2ENR |= (1 << 14); // enable EXTI (SYSCFG)

	// NOTE: we are configuring interrupt for PB4 pin.
	// 		 This is for counting the sensor CLK cycles, which
	//		 will be the basis of the timing of other signals.
	// 		 EXTI handler is responsible for incrementing count.

	SYSCFG->EXTICR[1] |= (0x1 << 0); // enable EXTI for PB4 (TRG pin)

	EXTI->IMR |= (1 << 4); // disable mask on EXTI4
	EXTI->RTSR |=  (1 << 4); //  enable  rising edge trigger
	EXTI->FTSR &= ~(1 << 4); // disable falling edge trigger

	NVIC_SetPriority(EXTI4_IRQn, 2);  // Set Priority

	// NOTE: we are configuring interrupt for PA11 pin (to trigger ADC)
	// 		 This is to allow interrupt-based triggering of ADC.

	SYSCFG->EXTICR[2] &= ~(0xF << 12); // enable EXTI for PA11 (TRG pin)
	EXTI->IMR |= (1 << 11); // disable mask on EXTI11

} // Init_C12880MA_EXTI()


void Init_C12880MA_ADC() {
	RCC->APB2ENR |= (1 << 8); // enable adc @ 80 MHz

	/* CLOCK CONFIGURATION SUMMARY
	 * ADCCLK @ 50 MHz
	 *   - source: APB2 PCLK (100 MHz)
	 *   - prsclr: /2 (minimum)
	 * CONVERSION TIME
	 *   = sampling time + bit resolution
	 *   = 3 cc + 12 cc = 15 cc (300 ns)
	 *   NOTE: 300 ns < T_period of sensor CLK
	 *   	   300 ns < 500 ns = 1/2 MHz
	 * */

	ADC->CCR &= ~(0x11 << 17); // prescaler: ADCCLK = APB2CLK/2 = 50 MHz

	ADC1->CR1 &= ~(0x11 << 24); // ADC resolution = 12-bit

	ADC1->SMPR2 &= ~(0x111 << 3); // sampling time = 3 ADCCLK cycles

	//Configure to Scan mode
	ADC1->CR1 |= (1 << 8);

	//total number of conversions in the channel conversion sequence
	ADC1->SQR1 &= ~(0xF << 20); // 1 conversion only

	//assign channel for first conversion
	ADC1->SQR3 |= (0b0111 << 0); // first (and only) conversion to channel 7 (PA7)

	ADC1->CR2 |= (0xF << 24); // EXTSEL: pick EXTI line 11 as source of trigger

	//end of conversion selection
	ADC1->CR2 &= ~(1 << 10); // enable EOC signal after conversion

	ADC1->CR2 &= ~(1 << 11); // right-align data

	ADC1->CR2 &= ~(1 << 1); // single conversion mode

	ADC1->CR2 &= ~(1 << 0); // ADC off for now

} // Init_C12880MA_ADC()

void Init_TIM2() {
	RCC->APB1ENR |= (1 << 0); 	// enable TIM2 (32-bit counter)
								// this is configured to 160 MHz (APB1 Timer CLK)

	TIM2->CR1 &= ~(1 << 4); // upcounter
	TIM2->CR1 &= ~(1 << 3); // one-pulse mode: clear counter every update event

	TIM2->CR2 &= ~(1 << 7); // CH1 is connected to TI1
}

/**
 * @brief  Initiates measurement with C12880MA spectrometer.
 * @param  channel_readings (uint16_t) 288-long array where each pixel's output value will
 * 		   be stored.
 * @param  on_time (uint32_t) [6, 0xFFFFF] On time of start pulse in terms of
 * 		   sensor clock cycles.
 * @param  sensor_clk_cycles (uint32_t) variable counting number of sensor clock cycles that
 * 		   has passed. Will be modified by the function (reset) and the
 * 		   interrupt handler (increment).
 * @retval none
 * */
void C12880MA_start(uint16_t channel_readings[288], uint32_t on_time, uint32_t sensor_clk_cycles) {
	ADC1->CR2 |= (1 << 0); // enable ADC
	ADC1->CR2 |= (0b01 << 28); // enable rising edge trigger detection
	// NOTE: we are enabling edge detection here, which will start the ADC
	// immediately, but we will not be saving the values until later
//	int EOS = 0; // end of scan flag.
	sensor_clk_cycles = 0; // reset to start count
	UART_printf("(%d) Initializations done. Starting process\r\n", sensor_clk_cycles);

	// --------------------- START SIGNAL ---------------------
	GPIOB->ODR |= (1 << 10); // assert START pin
	NVIC_EnableIRQ(EXTI4_IRQn); // start monitoring of TRG signal

	UART_printf("(%d) START asserted. Waiting for on_time to complete.\r\n", sensor_clk_cycles);

	while (sensor_clk_cycles <= on_time) {} // wait
	
	sensor_clk_cycles = 0; 		// reset again
	GPIOB->ODR &= ~(1 << 10); 	// deassert START pin
	
	UART_printf("(%d) on_time done: START deasserted. Waiting for values to be ready, then reading ADC\r\n", sensor_clk_cycles);

	while (sensor_clk_cycles < 87) {} // wait (87 cc as per datasheet)

	while (sensor_clk_cycles < 288) {
		while (!(ADC1->SR & (1 << 1))) {}
		channel_readings[sensor_clk_cycles] = ADC1->DR;
	}

	NVIC_DisableIRQ(EXTI4_IRQn);
	ADC1->CR2 &= ~(0b11 << 28); // disable rising edge trigger detection
	ADC1->CR2 &= ~(1 << 0);
}

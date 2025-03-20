/*
 * C12880MA.c
 *
 *  Created on: Feb 6, 2025
 *      Author: moreypiatos
 *
 *
 * PIN LAYOUT
 *    PC7  : LED control (unused)
 *    PA9  : video output
 *    PA8  : CLK out
 *    PB10 : START
 *    PB4  : TRIGGER
 *    PB5  : end-of-scan (EOS)
 */


#include <stdio.h>
#include <stm32f4xx.h>
#include <stm32f411xe.h>
#include <C12880MA.h>

void Init_C12880MA() {
	Init_C12880MA_CLKOUT();
	Init_C12880MA_GPIO();
	Init_C12880MA_ADC();
	Init_TIM2();
}

void Init_C12880MA_CLKOUT() {

	/* CLOCK CONFIGURATION
	 *
	 * PLLCLK source: HSI
	 *  - multiplier: x160
	 *  - master prescaler: /8
	 *  - PLL prescaler: /2
	 *  - f_eff = 100MHz
	 * ADCCLK source: PLLCLK
	 *  - AHB prescaler: /1
	 *  - APB1 prescaler: /2
	 *  - f_eff: 50MHz
	 * CLKOUT source: HSI
	 *  - prescaler: /5
	 *  - f_eff = 3.2Mhz
	 * SYSCLK source: PLLCLK
	 *
	 * Sensor CLK: 3.2MHz
	 * ADC CLK: 50MHz (much, much faster)
	 * other CLK: 50MHz
	 *
	 * */

	RCC->CR |= (1 << 24); // PLL ON
	RCC->CR |= (1 << 0);  // enable HSI
	RCC->CR |= (1 << 16); // enable HSE

	FLASH->ACR |= (1 << 8); // enable prefetch
	FLASH->ACR |= (0x1 << 0); // flash wait state = 1

	RCC->PLLCFGR &= ~(1 << 22); // PLL source = HSI

	RCC->PLLCFGR &= ~(1 << 17); // PLLP division factor = /4
	RCC->PLLCFGR |=  (1 << 16);

	RCC->PLLCFGR &= ~(1 << 14);   // PLLN multiplication factor = x160. PLLCFGR[14:6] = 0 1010 0000
	RCC->PLLCFGR &= ~(0xFF << 6); // clear
	RCC->PLLCFGR |=  (0xA0 << 6); // set 1010 0000

	RCC->PLLCFGR &= ~(0b111111 << 0); // PLLM main division factor = /8
	RCC->PLLCFGR |=  (0b001000 << 0);

	RCC->CFGR &= ~(1 << 29); // MCO2 prescaler: NONE

//	RCC->CFGR |=  (1 << 26); // MCO1 prescaler: division by 4
//	RCC->CFGR |=  (1 << 25); // note: bit 26 = 0 means no division
	RCC->CFGR |=  (0b110 << 24);

	RCC->CFGR |=  (0b11 << 30); // select SYSCLK as MCO2 source (UNCOMMENT. FOR TESTING ONLY)
//	RCC->CFGR &= ~(0b11 << 21); // select HSI as MCO1 source

	RCC->CFGR |=  (0b10 << 21); // select HSE as MCO1 source
//	RCC->CFGR |=  (0b11 << 21); // select PLL as MCO1 source

	RCC->CFGR &= ~(0b111 << 13); // APB2 prescaler = none, 80MHz

	RCC->CFGR &= ~(0b111 << 10);
	RCC->CFGR |=  (0b100 << 10); // APB1 prescaler = /2, 40MHz

	RCC->CFGR &= ~(1 << 7); // AHB prescaler = none, 100MHz

//	RCC->CFGR |=  (1 << 1); // use PLL as system clock
//	RCC->CFGR &= ~(1 << 0);

	RCC->CFGR &= ~(11 << 0); // use HSI as system clock

}

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
	GPIOA->MODER |= (1 << 19); // set PA9 as analog mode
	GPIOA->MODER |= (1 << 18);

	// C12880MA CLK pin (PA8)
	GPIOA->MODER |= (1 << 17); // set PA8 as alternate function
	GPIOA->MODER &= ~(1 << 16);

	GPIOA->AFR[0] &= (0x00000000); // set to MCO1 function (AF0 = 0b0000)

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

	// C12880MA trig pin (PB4)
	GPIOB->MODER &= ~(1 << 9); // set PB4 as input mode
	GPIOB->MODER &= ~(1 << 8);

	// C12880MA EOS pin (PB5)
	GPIOB->MODER &= ~(1 << 11); // set PB5 as input mode
	GPIOB->MODER &= ~(1 << 10);

}


void Init_C12880MA_ADC() {
	RCC->APB2ENR |= (1 << 8); // enable adc @ 80 MHz

	/* CLOCK CONFIGURATION SUMMARY
	 * ADCCLK @ 40 MHz
	 *   - source: APB2 PCLK
	 *   - prsclr: /2 (minimum)
	 * CONVERSION TIME
	 *   = sampling time + bit resolution
	 *   = 3 cc + 8 cc = 11 cc (275 ns)
	 *   NOTE: 275 ns < T_period of sensor CLK
	 *   	   275 ns < 312.5 ns
	 * */

	ADC->CCR &= ~(0x11 << 17); // prescaler: ADCCLK = APB2CLK/2 = 40 MHz

	ADC1->CR1 |= (0x10 << 24); // ADC resolution = 8-bit

	ADC1->SMPR2 &= ~(0x111 << 3); // sampling time = 3 ADCCLK cycles

	//Configure to Scan mode
	ADC1->CR1 |= (1 << 8);

//	//Enable Interrupt for EOC
//	ADC1->CR1 |= (1 << 5);

	//end of conversion selection
	ADC1->CR2 &= ~(1 << 10); // enable EOC signal after conversion

	ADC1->CR2 &= ~(1 << 11); // right-align data

	ADC1->CR2 &= ~(1 << 1); // single conversion mode

	//total number of conversions in the channel conversion sequence
	ADC1->SQR1 &= ~(0b1111 << 20);

	//assign channel for first conversion
	ADC1->SQR3 &= ~(0b1111 << 0); // first (and only) conversion to first channel

	ADC1->CR2 |= (1 << 1); // single conversion mode

	ADC1->CR2 &= ~(1 << 0); // disable for now

}

void Init_TIM2() {
	RCC->APB1ENR |= (1 << 0); 	// enable TIM2 (32-bit counter)
								// this is configured to 160 MHz (APB1 Timer CLK)

	TIM2->CR1 &= ~(1 << 4); // upcounter
	TIM2->CR1 &= ~(1 << 3); // one-pulse mode: clear counter every update event

	TIM2->CR2 &= ~(1 << 7); // CH1 is connected to TI1
}

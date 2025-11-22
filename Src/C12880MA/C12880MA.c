/**
 * C12880MA.c
 *
 *  Created on: Feb 6, 2025
 *      Author: moreypiatos
 *
 *
 * PIN LAYOUT
 *    PA0  : START (TIM2_CH4)
 *    PA8  : CLK out (hardwired by STM32)
 *    PA11 : TRG (must be shorted with PB4; connected to EXTI11, for ADC trigger)
 *    PB4  : TRG (connected to EXTI4, for incrementing of sensor_clk_cycles)
 *    PB3  : end-of-scan (EOS)
 * 	  PA6  : video output (ADC1_ch6)
 *    PC7  : LED control (unused)
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

#define ST_DELAY 3 // 3 cc delay between calling C12880MA_ST() and ST signal posedge

void Init_C12880MA(uint16_t C12880MA_readings[288]) {

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
	  UART_printf("\r\n\tTIM...");
	  Init_C12880MA_TIM();
	  UART_printf("DONE\r\n");
	  UART_printf("\r\n\tDMA...");
	  Init_C12880MA_DMA(C12880MA_readings);
	  UART_printf("DONE\r\n");
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

	// idk if this FLASH eme is needed kinuha ko sa internet habang may tino-troubleshoot
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

	GPIOC->ODR &= ~(1 << 7); // set as initially LOW

	UART_printf("\r\n\tPC7  OK...");

	// C12880MA CLK pin (PA8)
	GPIOA->MODER |= (1 << 17); // set PA8 as alternate function
	GPIOA->MODER &= ~(1 << 16);

	GPIOA->AFR[1] &= ~(0x00000001); // set to MCO1 function (AF0 = 0b0000)

	GPIOA->OSPEEDR |= (0b11 << 16); // set as high speed output

	UART_printf("\r\n\tPA8  OK...");

	// C12880MA ST pin (PA0)
	GPIOA->MODER |= (0b10 << 0); // set PA0 as alternate function

	GPIOA->AFR[0] |= (0x00000001); // set to TIM2_CH1 function (AF1 = 0b0001)

	UART_printf("\r\n\tPA0  OK...");

	// C12880MA temp output pin (of TIM1) PA10)
	GPIOA->MODER |= (0b10 << 20); // set PA10 as alternate function

	GPIOA->AFR[1] |= (0x00000100); // set to TIM1_CH3 function (AF1 = 0b0001)

	UART_printf("\r\n\tPA10 OK...");

	// PB5: temporary output to check if timer interrupt handler is being called
	GPIOB->MODER |= (0b01 << 10);	// set PB5 as output mode
	GPIOB->OTYPER &= ~(1 << 5);		// push-pull type
	GPIOB->OSPEEDR |= (0b10 << 10); // fast speed output
	GPIOB->PUPDR &= ~(0b11 << 10);	// no pull-up, pull-down

	GPIOB->ODR &= ~(1 << 5); // set as initially LOW

	UART_printf("\r\n\tPB5  OK...");

	// C12880MA video pin (PA6) << source of analog input for ADC
	GPIOA->MODER |= (0b11 << 12); // set PA6 as analog mode

	UART_printf("\r\n\tPA6  OK...");

	// C12880MA TRG pin (PA11), for ADC triggering
	GPIOA->MODER &= ~(0b11 << 22); // set PA11 as input mode
	GPIOA->PUPDR |=  (0b10 << 22); // pull-down

	UART_printf("\r\n\tPA11  OK...");

	// C12880MA TRG pin (PB4), for saving ADC result
	GPIOB->MODER &= ~(0b11 << 8); // set PB4 as input mode
	GPIOB->PUPDR |=  (0b10 << 8); // pull-down

	UART_printf("\r\n\tPB4  OK...");

	// C12880MA EOS pin (PB3)
	GPIOB->MODER &= ~(0b11 << 6); // set PB3 as input mode
	GPIOB->PUPDR |=  (0b10 << 6); // pull-down

	UART_printf("\r\n\tPB3  OK...");
	UART_printf("ALL OK.");

} // Init_C12880MA_GPIO()

void Init_C12880MA_EXTI() {
	RCC->APB2ENR |= (1 << 14); // enable EXTI (SYSCFG)

	//-------------------------------------------------------------------
	// NOTE: we are configuring interrupt for PB4 pin.
	// 		 This is for counting the sensor CLK cycles, and
	//		 and saving ADC values
	// 		 EXTI handler is responsible for incrementing count.
	//-------------------------------------------------------------------

	SYSCFG->EXTICR[1] |= (0x1 << 0); // enable EXTI for PB4 (TRG pin)
	EXTI->IMR |= (1 << 4); // disable mask on EXTI4

	EXTI->RTSR |=  (1 << 4); //  enable  rising edge trigger
	EXTI->FTSR &= ~(1 << 4); // disable falling edge trigger

	NVIC_SetPriority(EXTI4_IRQn, 3);  // Set Priority

	//-------------------------------------------------------------------
	// NOTE: we are configuring interrupt for PA11 pin (to trigger ADC)
	// 		 This is to allow interrupt-based triggering of ADC.
	//-------------------------------------------------------------------

	// SYSCFG->EXTICR[2] &= ~(0xF << 12); // enable EXTI for PA11 (TRG pin)
	// EXTI->IMR |= (1 << 11); // disable mask on EXTI11

	// no need to configure edge trigger here

	//-------------------------------------------------------------------
	// NOTE: we are configuring interrupt for PB3 pin (EOS)
	// 		 This is to allow interrupt-based end of program.
	//-------------------------------------------------------------------

	SYSCFG->EXTICR[0] |= (0x1 << 12); // enable EXTI for PB3 (EOS pin)
	EXTI->IMR |= (1 << 3); // disable mask on EXTI4

	EXTI->RTSR |=  (1 << 3); //  enable  rising edge trigger
	EXTI->FTSR &= ~(1 << 3); // disable falling edge trigger

	NVIC_SetPriority(EXTI3_IRQn, 2);  // Set Priority

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
	ADC1->SQR3 |= (0b00110 << 0); // first (and only) conversion to channel 6 (PA6)

	ADC1->CR2 |= (0b01 << 28); // trigger ADC on rising edge of EXTI

	ADC1->CR2 |= (0xF << 24); // EXTSEL: pick EXTI line 11 as source of trigger

	//end of conversion selection
	ADC1->CR2 &= ~(1 << 10); // enable EOC signal after conversion

	ADC1->CR2 &= ~(1 << 11); // right-align data

	ADC1->CR2 |=  (1 << 9); // DMA request issued as long as request is made and bit 8 is set
	ADC1->CR2 |=  (1 << 8); // DMA mode enabled

	ADC1->CR2 &= ~(1 << 1); // single conversion mode

	ADC1->CR2 &= ~(1 << 0); // ADC off for now

} // Init_C12880MA_ADC()

void Init_C12880MA_TIM() {
	//////////////////////////////////////////////////////////////////////////////////////////
	RCC->APB1ENR |= (1 << 0); // enable TIM2 (32-bit counter)
							  // this is configured to 100 MHz (APB1 Timer CLK)

	TIM2->PSC |= (49 << 0); // pre-scaler = 49 + 1 so that f_eff = 2 MHz (same as sensor CLK)

	TIM2->CR1 &= ~(0b11 << 8); // clock division: none
	TIM2->CR1 &= ~(0b11 << 5); // edge-aligned mode (follow DIR bit)
	TIM2->CR1 &= ~(1 << 4); // DIR: up-counting
	TIM2->CR1 |=  (1 << 3); // one-pulse mode: disable counter at update event (UEV)

	TIM2->CCR1 	 = (10 << 0); 	// this sets t_delay before output is toggled
	TIM2->ARR 	 = (99 + 10); 	// pulse duration default: 5ms (x0.5µs)
	TIM2->CCMR1 |= (0b111 << 4); 	// OC1M: PWM 2 (high on CNT > CCR, low otherwise);
	TIM2->CCMR1 &= ~(0b11 << 0); 	// CC1S: configured as OUTPUT
	TIM2->EGR	|= (1 << 0);		// UG: ENABLED

	TIM2->CCER  &= ~(1 << 1);		// CC1P: OC1 active high
	TIM2->CCER 	|=  (1 << 0); 		// CC1E: OC1 is output on corresponding pin (PA0)

	TIM2->CR2 |= (0b010 << 4); // Master Mode: UEV as TRGO

	TIM2->CNT &= ~(0xFFFFFFFF); // set count to 0 (TIM2 is 32-bit)
	TIM2->CR1 &= ~(1 << 0); // disable TIM2 for now

	//////////////////////////////////////////////////////////////////////////////////////////
	RCC->APB2ENR |= (1 << 0); 	// enable TIM1 (32-bit counter)
								// this is configured to 100 MHz (APB2 Timer CLK)

	TIM1->PSC |= (9 << 0); // pre-scaler = 9 + 1 so that f_eff = 10 MHz (faster than sensor CLK)
							// 0.1µs per clock cycle

	TIM1->CR1 &= ~(0b11 << 8); // clock division: none
	TIM1->CR1 &= ~(0b11 << 5); // edge-aligned mode (follow DIR bit)
	TIM1->CR1 &= ~(1 << 4); // DIR: up-counting
	TIM1->CR1 |=  (1 << 3); // one-pulse mode: disable counter at update event (UEV)

	TIM1->CCR3 	 = (444); 	// 43.5 µs
	TIM1->ARR 	 = (445); 	//
	TIM1->CCMR2 |= (0b110 << 4); 	// OC3M: PWM 1 (high on CNT < CCR, low otherwise);
	TIM1->CCMR2 &= ~(0b11 << 0); 	// CC3S: configured as OUTPUT
	TIM1->EGR	|= (1 << 0);		// UG: ENABLED

	TIM1->CCER  |=  (1 << 9);		// CC3P: OC3 active low
	TIM1->CCER 	|=  (1 << 8); 		// CC3E: OC3 is output on corresponding pin (PA0)

	TIM1->BDTR |= (1 << 15); // Main Output Enable: enable

	TIM1->SMCR |= (1 << 15); // non-inverted ETR
	TIM1->SMCR |= (0b001 << 4); // ITR1 (TIM2_TRGO) as trigger source
	TIM1->SMCR |= (0b110 << 0); // Slave Mode: Trigger Mode - start timer at posedge of TRGI

	TIM1->DIER |= (1 << 3); // generate interrupt on compare

	TIM1->CNT &= ~(0xFFFFFFFF); // set count to 0 (TIM1 is 32-bit)
	TIM1->CR1 &= ~(1 << 0); // disable TIM1 for now

	NVIC_SetPriority(TIM1_CC_IRQn, 5);  // Set Priority
} // Init_C12880MA_TIM()

void Init_C12880MA_DMA(uint16_t C12880MA_readings[288]) {
	RCC->AHB1ENR |= (1 << 22); // enable DMA2 clk

	DMA2_Stream0->CR = (uint32_t)
			(0b000 << 25) 	// channel 0 (ADC1)
		|	(0b10  << 16)	// high priority
		|	(0b01  << 13)	// memory data size: half-word (16 bits)
		|	(0b01  << 11)	// periph data size: half-word (16 bits)
		|	(0b1   << 10)	// increment memory address after every transfer
		|	(0b0   <<  9)	// DONT increment periph address after transfer
		|	(0b00  <<  6)	// direction: peripheral-to-memory
		|	(0b0   <<  5)	// DMA is the flow controller
		|	(0b1   <<  4)	// enable interrupt on transfer complete
		| (0b1   <<  2) // enable interrupt on transfer error
	; // this is = 0x00022C30
	// NOTE: burst transfer config automatically forced to 00 for Direct Mode.

	DMA2_Stream0->NDTR = (uint32_t)288; // store 288 pixels worth of data
	DMA2_Stream0->PAR = (uint32_t)&ADC1->DR;
	DMA2_Stream0->M0AR = (uint32_t)&C12880MA_readings[0];
	// NOTE: M1AR only needed for double buffer mode. no need to configure
	// NOTE: Direct mode is enabled by default. no need to configure

  NVIC_SetPriority(DMA2_Stream0_IRQn, 5);
  NVIC_EnableIRQ(DMA2_Stream0_IRQn);
} // Init_C12880MA_DMA()

void DMA2_S0_Set_DstAddr(uint32_t DstAddr) {
	DMA2_Stream0->M0AR = DstAddr;
}

void DMA2_S0_Set_SrcAddr(uint32_t SrcAddr) {
	DMA2_Stream0->PAR = SrcAddr;
}

void DMA2_S0_Set_NumofDataTransfers(uint32_t NumofDataTransfers) {
	DMA2_Stream0->NDTR = NumofDataTransfers;
}

void C12880MA_set_tint(uint32_t tint) {
	// NOTE: x2 factor is due to time period of sensor CLK
	// sensor CLK = 2 MHz = 0.5 µs
	// 1µs/0.5µs = 2 
	if (tint < 50) {
		TIM2->ARR = 50*2;
	} else if (tint > 1000000) {
		TIM2->ARR = 1000000*2;
	} else {
		TIM2->ARR = tint*2;
	}
}

void C12880MA_ST(uint32_t st_pulse_width) {
	Init_C12880MA_TIM();

	TIM2->CR1 &= ~(1 << 0); // disable timer
	TIM2->PSC |=  (49 << 0); // pre-scaler = 49 + 1; f_eff = 2 MHz (same as sensor CLK)
	TIM2->CNT &= ~(0xFFFFFFFF); // set count to 0 (TIM2 is 32-bit)
	TIM2->ARR  =  (st_pulse_width - 1 + 10); 	// pulse duration default: 5ms (x0.5µs)

	TIM1->CR1 &= ~(1 << 0); // disable timer
	TIM1->PSC |= (9 << 0); // pre-scaler = 9 + 1 so that f_eff = 10 MHz
	TIM1->CNT &= ~(0xFFFFFFFF); // set count to 0 (TIM2 is 32-bit)
  TIM1->DIER |= (1 << 3); // enable interrupt on compare

	NVIC_EnableIRQ(TIM1_CC_IRQn); // enable tim1 interrupt
	NVIC_EnableIRQ(EXTI3_IRQn); // enable EOS interrupt now

	DMA2_Stream0->CR = (uint32_t) DMA_Config_DisableDMA; // ensure DMA is disabled before configuration
  DMA2_S0_Set_NumofDataTransfers(288);

	DMA2_Stream0->CR = (uint32_t) DMA_Config_EnableDMA; // enable DMA now

	TIM2->CR1 |=  (1 << 0); // enable timer
} // C12880MA_ST()

/* DEPRECATED */ 
void _C12880MA_start(uint16_t channel_readings[288], uint32_t on_time, uint32_t sensor_clk_cycles) {
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

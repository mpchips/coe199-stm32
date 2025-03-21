/*
 * I2C2.c
 *
 *  Created on: Mar 21, 2025
 *      Author: moreypiatos
 *
 * Uses STM32F411 I2C2 peripheral in standard mode.
 *
 * PIN LAYOUT
 *    PB10  : SDA
 *    PB11  : SCL
 */

#include <I2C2.h>
#include <stdio.h>
#include <stdint.h>
#include <stm32f4xx.h>
#include <stm32f411xe.h>


/* DEFINE THE FF VALUES BELOW FOR INITIALIZATION */
#define I2C2_SDA_PIN 10 // will automatically use GPIOB
#define I2C2_SCL_PIN 11 // will automatically use GPIOB

#define APB1_CLK_FREQ_MHZ 40 // configured clock frequency of APB1 (default = 16)
#define I2C2_SCL_FREQ_KHZ 2500 // target I2C2_SCL frequency

uint16_t CCR_val = APB1_CLK_FREQ_MHZ*1000/(2*I2C2_SCL_FREQ_KHZ);

/* CLOCK CALCULATIONS
 *
 * @warn Ensure that I2C2_SCL_FREQ_KHZ can be obtained by
 * 		 dividing APB1_CLK_FREQ_MHZ*1000 with a multiple of 2
 * 		 (maximum: 8192, minimum: 8).
 *
 * 		 Failure to do so will result in HIGHER SCL value.
 *
 * T_pclk = 1/APB1_CLK_FREQ_MHZ (µ sec)
 * T_SCL  = 1/I2C2_SCL_FREQ_KHZ (m sec)
 *
 * T_high = 0.5 * T_SCL
 *
 *  ->CCR = T_high/T_apb1
 *  	  = APB1_CLK_FREQ_MHZ*1000 / 2*I2C2_SCL_FREQ_KHZ
 *
 *  Alternatively, ensure that CCR (given above) is a whole number.
 * */


void Init_I2C2() {
	// 1. enable I2C2 clock
	RCC->APB1ENR |= (1 << 22);  // enable I2C2 CLOCK

	// 2. configure I2C2 pins
	GPIOB->MODER |= (2 << (I2C2_SDA_PIN*2)); // PB8
	GPIOB->MODER |= (2 << (I2C2_SCL_PIN*2)); // PB9

	GPIOB->OTYPER |= (1 << I2C2_SDA_PIN); // open-drain
	GPIOB->OTYPER |= (1 << I2C2_SCL_PIN); // open-drain

	GPIOB->OSPEEDR |= (3 << (I2C2_SDA_PIN*2)) | (3 << (I2C2_SCL_PIN*2)); // high-speed output

	GPIOB->PUPDR |= (1 << (I2C2_SDA_PIN*2)) | (1 << (I2C2_SCL_PIN*2)); // pull-up

	if (I2C2_SDA_PIN >= 8) {
		GPIOB->AFR[1] |= 4 << ((I2C2_SDA_PIN-8)*4); // alternate function 4
	} else {
		GPIOB->AFR[0] |= 4 << (I2C2_SDA_PIN*4); // alternate function 4
	}

	if (I2C2_SCL_PIN >= 8) {
		GPIOB->AFR[1] |= 4 << ((I2C2_SCL_PIN-8)*4); // alternate function 4
	} else {
		GPIOB->AFR[0] |= 4 << (I2C2_SCL_PIN*4); // alternate function 4
	}

	// 3. reset I2C2
	I2C2->CR1 &= ~(1 << 0);	// peripheral disable (reset)
	if (!(I2C2->CR1 & (1 << 0))) {} // wait 3 clk cycles (recommended by ref. manual)

	// 4. configure clock
	I2C2->CR2 |=  APB1_CLK_FREQ_MHZ; // freq = 40 MHz (slow APB clk), t=25 ns

	I2C2->CCR &= ~(1 << 15); // Standard Mode I2C
	I2C2->CCR |=  CCR_val; // CCR * t_pclk = t_high = t_low = 2500 ns

//	I2C2->CCR |=  (1 << 15); // Fast Mode I2C
//	I2C2->CCR &= ~(1 << 14); // (In FM) 2:1 of SCL t_low:t_high
//	I2C2->CCR |=  (50 << 0); // CCR * t_pclk = t_high = 1000 ns

	I2C2->TRISE |= CCR_val+1; // = (T_rise,max/T_pclk) + 1 = 1000ns/25ns + 1


	// THIS SHOULD BE THE **LAST** THING YOU DO
	I2C2->CR1 |= (1 << 0); // peripheral enable

}

void I2C2_start() {

	I2C2->CR1 |= (1 << 8); // generate START
	while (!(I2C2->SR1 & (1 << 0))); // wait for START condition to finish
}

void I2C2_stop() {
	I2C2->CR1 |= (1 << 9); // generate STOP
						   // this is automatically cleared after
}

void I2C2_transmit(uint8_t data) {
	I2C2->DR = data;
	while (!I2C2_transmit_done());
}

void I2C2_transmit_addr(uint8_t addr) {
	I2C2->DR = (addr << 0);
	while (!(I2C2->SR1 & (1 << 1))); // ADDR flag: "address successfully transmitted"
	uint16_t _dummy_read = I2C2->SR1 | I2C2->SR2; // dummy read to clear the flag
}

int I2C2_transmit_done() {
	return (I2C2->SR1 & (1 << 2)); // BTF flag: "byte transfer finished, DR empty"
}

uint8_t I2C2_receive() {
	while (!(I2C2->SR1 & (1 << 6))); // (RxNE) wait to receive

	return I2C2->DR; // read received value
}

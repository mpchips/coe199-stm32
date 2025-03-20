/*
 * I2C.c
 *
 *  Created on: Feb 6, 2025
 *      Author: moreypiatos
 */

#include <stdio.h>
#include <stdint.h>
#include <stm32f4xx.h>
#include <stm32f411xe.h>
#include <I2C.h>


void Init_I2C() {

	/* SOME NOTES
	 *  - I2C automatically configures as Master Mode when sending START signal (through CR1 register)
	 *  - 7-bit addressing mode
	 *  - I2C->CR1[12] = STOP
	 *  - I2C->CR1[13] = START
	 *  - I2C->TXDR = byte to be transmitted
	 *  - I2C->RXDR = byte received
	 * SEQUENCE FOR MASTER MODE:
	 *  - program peripheral input clock (CR2)
	 *  - configure SCL (TIMINGR)
	 * */

	// 1. enable I2C clock
	RCC->APB1ENR |= (1<<21);  // enable I2C CLOCK

	// 2. configure I2C pins
	GPIOB->MODER |= (2<<16); // PB8
	GPIOB->MODER |= (2<<18); // PB9

	GPIOB->OTYPER |= (1<<8); // open-drain
	GPIOB->OTYPER |= (1<<9); // open-drain

	GPIOB->OSPEEDR |= (3<<16) | (3<<18); // high-speed output

	GPIOB->PUPDR |= (1<<16) | (1<<18); // pull-up

	GPIOB->AFR[1] |= (4<<0) | (4<<4); // alternate function 4

	// 3. reset I2C
	I2C1->CR1 &= ~(1 << 0);	// peripheral disable (reset)
	if (!(I2C1->CR1 & (1 << 0))) {} // wait 3 clk cycles (recommended by ref. manual)

	// 4. configure clock
	I2C1->CR2 |=  (40 << 0); // freq = 40 MHz (slow APB clk), t=25 ns

	I2C1->CCR &= ~(1 << 15); // Standard Mode I2C
	I2C1->CCR |=  (100 << 0); // CCR * t_pclk = t_high = t_low = 2500 ns

//	I2C1->CCR |=  (1 << 15); // Fast Mode I2C
//	I2C1->CCR &= ~(1 << 14); // (In FM) 2:1 of SCL t_low:t_high
//	I2C1->CCR |=  (50 << 0); // CCR * t_pclk = t_high = 1000 ns

	I2C1->TRISE |= (101 << 0); // = (T_rise,max/T_pclk) + 1 = 1000ns/25ns + 1


	// THIS SHOULD BE THE **LAST** THING YOU DO
	I2C1->CR1 |= (1 << 0); // peripheral enable

//	//these can be changed AFTER enabling I2C
//	I2C1->CR2 &= ~(1 << 11); // 7-bit addressing mode

}

void I2C_start() {

	I2C1->CR1 |= (1 << 8); // generate START
	while (!(I2C1->SR1 & (1 << 0))); // wait for START condition to finish
}

void I2C_stop() {
	I2C1->CR1 |= (1 << 9); // generate STOP
						   // this is automatically cleared after
}

void I2C_transmit(uint8_t data) {
	I2C1->DR = data;
	while (!I2C_transmit_done());
}

void I2C_transmit_addr(uint8_t addr) {
	I2C1->DR = (addr << 0);
	while (!(I2C1->SR1 & (1 << 1))); // ADDR flag: "address successfully transmitted"
	uint16_t _dummy_read = I2C1->SR1 | I2C1->SR2; // dummy read to clear the flag
}

int I2C_transmit_done() {
	return (I2C1->SR1 & (1 << 2)); // BTF flag: "byte transfer finished, DR empty"
}

uint8_t I2C_receive() {
	while (!(I2C1->SR1 & (1 << 6))); // (RxNE) wait to receive

	return I2C1->DR; // read received value
}

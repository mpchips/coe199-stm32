/*
 * I2C1.c
 *
 *  Created on: Feb 6, 2025
 *      Author: moreypiatos
 *
 * Uses STM32F411 I2C1 peripheral
 *
 * PIN LAYOUT
 *    PB7  : SDA
 *    PB6  : SCL
 */

#include <I2C1.h>
#include <stdio.h>
#include <stdint.h>
#include <stm32f4xx.h>
#include <stm32f411xe.h>


void Init_I2C1() {
	// 1. enable I2C clock
	RCC->APB1ENR |= (1<<21);  // enable I2C CLOCK

	// 2. configure I2C pins
	GPIOB->MODER |= (2<<14); // PB7
	GPIOB->MODER |= (2<<12); // PB6

	GPIOB->OTYPER |= (1<<7); // open-drain
	GPIOB->OTYPER |= (1<<6); // open-drain

	GPIOB->OSPEEDR |= (3<<14) | (3<<12); // high-speed output

	GPIOB->PUPDR |= (1<<14) | (1<<12); // pull-up

	GPIOB->AFR[0] |= (4<<24); // alternate function 4
	GPIOB->AFR[0] |= (4<<28); // alternate function 4

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

void I2C1_start() {

	I2C1->CR1 |= (1 << 8); // generate START
	while (!(I2C1->SR1 & (1 << 0))); // wait for START condition to finish
}

void I2C1_stop() {
	I2C1->CR1 |= (1 << 9); // generate STOP
						   // this is automatically cleared after
}

void I2C1_send_buf(uint8_t SLAVE_WRITE_ADDR, uint8_t *buf, uint32_t len) {
	uint8_t *current_char = buf;

	I2C1_start();

	I2C1_transmit_addr(SLAVE_WRITE_ADDR);

	for (uint32_t remaining = len; remaining > 0; --remaining, ++current_char) {
		I2C1_transmit(*current_char);
	}

	I2C1_stop();
}

void I2C1_transmit(uint8_t data) {
	I2C1->DR = data;
	while (!I2C1_transmit_done());
}

void I2C1_transmit_addr(uint8_t addr) {
	I2C1->DR = (addr << 0);
	while (!(I2C1->SR1 & (1 << 1))); // ADDR flag: "address successfully transmitted"
	uint16_t _dummy_read = I2C1->SR1 | I2C1->SR2; // dummy read to clear the flag
}

int I2C1_transmit_done() {
	return (I2C1->SR1 & (1 << 2)); // BTF flag: "byte transfer finished, DR empty"
}

uint8_t I2C1_receive() {
	while (!(I2C1->SR1 & (1 << 6))); // (RxNE) wait to receive

	return I2C1->DR; // read received value
}

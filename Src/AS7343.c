/*
 * C12880MA.c
 *
 *  Created on: Feb 6, 2025
 *      Author: moreypiatos
 *
 * Designed to interface using GroupGets LLC.
 * C12880MA Breakout Board
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
#include <stdint.h>
#include <stm32f4xx.h>
#include <stm32f411xe.h>
#include <AS7343.h>
#include <I2C1.h>

/**
 * @brief List of channels in the vis-nir range
 */
AS7343_color_channel_t vis_nir_channels[12] = {
  AS7343_CHANNEL_F1,
  AS7343_CHANNEL_F2,
  AS7343_CHANNEL_FZ,
  AS7343_CHANNEL_F3,
  AS7343_CHANNEL_F4,
  AS7343_CHANNEL_F5,
  AS7343_CHANNEL_FY,
  AS7343_CHANNEL_FXL,
  AS7343_CHANNEL_F6,
  AS7343_CHANNEL_F7,
  AS7343_CHANNEL_F8,
  AS7343_CHANNEL_NIR,
};

/**
 * @brief List of channels' corresponding bands
 * 	      in the vis-nir range.
 */
uint16_t vis_nir_bands[12] = {
  405,
  425,
  450,
  475,
  515,
  550,
  555,
  600,
  640,
  690,
  745,
  855
};


void Init_AS7343() {
	Init_AS7343_GPIO();
	Init_I2C();
//	Init_AS7343_I2C(); // not needed for F411 implementation
}

void Init_AS7343_GPIO() {
	if (!(RCC->AHB1ENR & (1 << 1))) { // enable GPIOB
		RCC->AHB1ENR |= (1 << 1);
	}

	// AS7343L SCL pin (PB8)
	GPIOB->MODER |= (1 << 17); // set PB8 as alternate function
	GPIOB->MODER &= ~(1 << 16);

	GPIOB->AFR[1] |= (0x00000004); // set to I2C1 SCL function (AF4 = 0b0100)

	GPIOB->OTYPER |= (1 << 8); // open drain type

	GPIOB->OSPEEDR |= (1 << 17); // high speed output
	GPIOB->OSPEEDR |= (1 << 16);

	GPIOB->PUPDR &= ~(1 << 17); // pull-up
	GPIOB->PUPDR |=  (1 << 16);

	// AS7343L SDA pin (PB9)
	GPIOB->MODER |= (1 << 19); // set PB9 as alternate function
	GPIOB->MODER &= ~(1 << 18);

	GPIOB->AFR[1] |= (0x00000040); // set to I2C1 SDA function (AF4 = 0b0100)

	GPIOB->OTYPER |= (1 << 9); // open drain type

	GPIOB->OSPEEDR |= (1 << 19); // high speed output
	GPIOB->OSPEEDR |= (1 << 18);

	GPIOB->PUPDR &= ~(1 << 19); // pull-up
	GPIOB->PUPDR |=  (1 << 18);

}

void AS7343_enable() {
	AS7343_conf(0b10000000, 0x03);
}

void AS7343_disable() {
	AS7343_conf(AS7343_ENABLE, 0x00);
}

void AS7343_reset() {
	AS7343_conf(AS7343_CONTROL, 1 << 3);
}

void AS7343_conf(uint8_t reg_addr, uint8_t data) {
	I2C_start();

	I2C_transmit_addr(AS7343_CHIP_ADDR_WRITE); // chip-addresswrite

	while (!(I2C1->SR1 & (1 << 7))); // TxE: wait for Tx to Empty
	I2C_transmit(reg_addr);

	I2C_transmit(data);

	I2C_stop();
}

void AS7343_auto_smux(auto_smux_mode auto_smux_mode) {
	switch (auto_smux_mode) {
		case mode0_6ch:
			AS7343_conf(AS7343_CFG20, (0b00 << 5));
		case mode1_12ch:
			AS7343_conf(AS7343_CFG20, (0b10 << 5));
		case mode2_18ch:
			AS7343_conf(AS7343_CFG20, (0b11 << 5));
	}
}

void AS7343_set_ASTEP(uint16_t astep) {
	if (astep == 0) {
		if (AS7343_read(AS7343_ATIME) == 0) { // do not allow ATIME = ASTEP = 0 as per datasheet
			AS7343_conf(AS7343_ASTEP_L, 0x01);
			AS7343_conf(AS7343_ASTEP_H, 0x00);
		}
	} else {
		AS7343_conf(AS7343_ASTEP_L, (uint8_t) astep);
		AS7343_conf(AS7343_ASTEP_L, (uint8_t) astep >> 8);
	}
}


void AS7343_set_ATIME(uint8_t atime) {
	if (atime == 0) {
		if (AS7343_read_2b(AS7343_ASTEP_L) == 0) {
			AS7343_conf(AS7343_ATIME, 0x01); // do not allow ATIME = ASTEP = 0 as per datasheet
		}
	} else {
		AS7343_conf(AS7343_ATIME, atime);
	}
}


int AS7343_done() {
	int reg_val = (int) AS7343_read(AS7343_STATUS2);
	int return_val = reg_val & 0b00001000;
	return (return_val >> 3);
}

int AS7343_DSat() {
	int reg_val = (int) AS7343_read(AS7343_STATUS2);
	int return_val = reg_val & 0b00010000;
	return (return_val >> 4);
}

void AS7343_get_spectrum(uint16_t channel_readings[12]) {
	AS7343_auto_smux(mode2_18ch);

	AS7343_enable();

	while (!AS7343_done()); // wait for measurement to finish

	AS7343_read_spectrum(channel_readings);

	AS7343_disable();
}


void AS7343_read_spectrum(uint16_t channel_readings[12]) {
	int i;
	for (i = 0; i < 12; i++) {
		channel_readings[i] = AS7343_read_channel(vis_nir_channels[i]);
	}
}

uint16_t AS7343_read_channel(AS7343_color_channel_t channel) {
	uint16_t return_val = 0x0000;
	uint8_t reg_read_val = 0x00;

	// channels are arranged in spectral order, not accdg to data register order
	switch (channel)
	{
		case AS7343_CHANNEL_F1:
			reg_read_val = AS7343_read(AS7343_CH12_DATA_L);
			return_val |= (reg_read_val << 0);
			reg_read_val = AS7343_read(AS7343_CH12_DATA_H);
			return_val |= (reg_read_val << 8);
			break;

		case AS7343_CHANNEL_F2:
			reg_read_val = AS7343_read(AS7343_CH6_DATA_L);
			return_val |= (reg_read_val << 0);
			reg_read_val = AS7343_read(AS7343_CH6_DATA_H);
			return_val |= (reg_read_val << 8);
			break;

		case AS7343_CHANNEL_FZ:
			reg_read_val = AS7343_read(AS7343_CH0_DATA_L);
			return_val |= (reg_read_val << 0);
			reg_read_val = AS7343_read(AS7343_CH0_DATA_H);
			return_val |= (reg_read_val << 8);
			break;

		case AS7343_CHANNEL_F3:
			reg_read_val = AS7343_read(AS7343_CH7_DATA_L);
			return_val |= (reg_read_val << 0);
			reg_read_val = AS7343_read(AS7343_CH7_DATA_H);
			return_val |= (reg_read_val << 8);
			break;

		case AS7343_CHANNEL_F4:
			reg_read_val = AS7343_read(AS7343_CH8_DATA_L);
			return_val |= (reg_read_val << 0);
			reg_read_val = AS7343_read(AS7343_CH8_DATA_H);
			return_val |= (reg_read_val << 8);
			break;

		case AS7343_CHANNEL_F5:
			reg_read_val = AS7343_read(AS7343_CH13_DATA_L);
			return_val |= (reg_read_val << 0);
			reg_read_val = AS7343_read(AS7343_CH13_DATA_H);
			return_val |= (reg_read_val << 8);
			break;

		case AS7343_CHANNEL_FY:
			reg_read_val = AS7343_read(AS7343_CH1_DATA_L);
			return_val |= (reg_read_val << 0);
			reg_read_val = AS7343_read(AS7343_CH1_DATA_H);
			return_val |= (reg_read_val << 8);
			break;

		case AS7343_CHANNEL_FXL:
			reg_read_val = AS7343_read(AS7343_CH2_DATA_L);
			return_val |= (reg_read_val << 0);
			reg_read_val = AS7343_read(AS7343_CH2_DATA_H);
			return_val |= (reg_read_val << 8);
			break;

		case AS7343_CHANNEL_F6:
			reg_read_val = AS7343_read(AS7343_CH9_DATA_L);
			return_val |= (reg_read_val << 0);
			reg_read_val = AS7343_read(AS7343_CH9_DATA_H);
			return_val |= (reg_read_val << 8);
			break;

		case AS7343_CHANNEL_F7:
			reg_read_val = AS7343_read(AS7343_CH14_DATA_L);
			return_val |= (reg_read_val << 0);
			reg_read_val = AS7343_read(AS7343_CH14_DATA_H);
			return_val |= (reg_read_val << 8);
			break;

		case AS7343_CHANNEL_F8:
			reg_read_val = AS7343_read(AS7343_CH15_DATA_L);
			return_val |= (reg_read_val << 0);
			reg_read_val = AS7343_read(AS7343_CH15_DATA_H);
			return_val |= (reg_read_val << 8);
			break;

		case AS7343_CHANNEL_NIR:
			reg_read_val = AS7343_read(AS7343_CH3_DATA_L);
			return_val |= (reg_read_val << 0);
			reg_read_val = AS7343_read(AS7343_CH3_DATA_H);
			return_val |= (reg_read_val << 8);
			break;

		// OTHER CHANNELS --------
		case AS7343_CHANNEL_CLR_0:
			reg_read_val = AS7343_read(AS7343_CH16_DATA_L);
			return_val |= (reg_read_val << 0);
			reg_read_val = AS7343_read(AS7343_CH16_DATA_H);
			return_val |= (reg_read_val << 8);
			break;

		case AS7343_CHANNEL_CLR_1:
			reg_read_val = AS7343_read(AS7343_CH10_DATA_L);
			return_val |= (reg_read_val << 0);
			reg_read_val = AS7343_read(AS7343_CH10_DATA_H);
			return_val |= (reg_read_val << 8);
			break;

		case AS7343_CHANNEL_CLR_2:
			reg_read_val = AS7343_read(AS7343_CH4_DATA_L);
			return_val |= (reg_read_val << 0);
			reg_read_val = AS7343_read(AS7343_CH4_DATA_H);
			return_val |= (reg_read_val << 8);
			break;

		case AS7343_CHANNEL_FD_0:
			reg_read_val = AS7343_read(AS7343_CH17_DATA_L);
			return_val |= (reg_read_val << 0);
			reg_read_val = AS7343_read(AS7343_CH17_DATA_H);
			return_val |= (reg_read_val << 8);
			break;

		case AS7343_CHANNEL_FD_1:
			reg_read_val = AS7343_read(AS7343_CH11_DATA_L);
			return_val |= (reg_read_val << 0);
			reg_read_val = AS7343_read(AS7343_CH11_DATA_H);
			return_val |= (reg_read_val << 8);
			break;

		case AS7343_CHANNEL_FD_2:
			reg_read_val = AS7343_read(AS7343_CH5_DATA_L);
			return_val |= (reg_read_val << 0);
			reg_read_val = AS7343_read(AS7343_CH5_DATA_H);
			return_val |= (reg_read_val << 8);
			break;
	}

	return return_val;
}

uint8_t AS7343_read(uint8_t reg_addr) {
	I2C_start(); // start
	I2C_transmit_addr(AS7343_CHIP_ADDR_WRITE); // slave addr (write)
	I2C_transmit(reg_addr); // reg address

	I2C_start(); // restart
	I2C_transmit_addr(AS7343_CHIP_ADDR_READ); // slave addr (read)

	uint8_t ret_val = I2C_receive();


	return ret_val;

}

uint16_t AS7343_read_2b(uint8_t reg_addr_lower_byte) {
	uint16_t return_val = 0x0000;
	uint8_t reg_read_val = 0x00;
	reg_read_val = AS7343_read(reg_addr_lower_byte);
	return_val |= (reg_read_val << 0);
	reg_read_val = AS7343_read(reg_addr_lower_byte + 0x01);
	return_val |= (reg_read_val << 8);

	return return_val;
}

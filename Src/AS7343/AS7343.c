/*
 * AS7343.c
 *
 *  Created on: Feb 6, 2025
 *      Author: moreypiatos
 *
 * For the ams OSRAM AS7343 Spectrometer Sensor
 *
 * PIN LAYOUT
 * 	  PB5: LED 3.3V
 *    PB8: I2C1_SCL
 *    PB9: I2C1_SDA
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
	855,
};

AS7343_gain_t AGAIN_lookup[13] = {
	AS7343_GAIN_0_5X,
	AS7343_GAIN_1X,
	AS7343_GAIN_2X,
	AS7343_GAIN_4X,
	AS7343_GAIN_8X,
	AS7343_GAIN_16X,
	AS7343_GAIN_32X,
	AS7343_GAIN_64X,
	AS7343_GAIN_128X,
	AS7343_GAIN_256X,
	AS7343_GAIN_512X,
	AS7343_GAIN_1024X,
	AS7343_GAIN_2048X,
};


void Init_AS7343() {
//	Init_AS7343_GPIO();
	Init_I2C1();
}

void Init_AS7343_GPIO() {
	if (!(RCC->AHB1ENR & (1 << 1))) { // enable GPIOB
		RCC->AHB1ENR |= (1 << 1);
	}

	// GENERAL LED out pin (PB5)
	GPIOB->MODER &= ~(1 << 11); // set PB5 as output mode
	GPIOB->MODER |=  (1 << 10);

	GPIOB->OSPEEDR |=  (0b11 << 10); // high speed output

	GPIOB->ODR &= ~(1 << 5); // set low for now

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

void set_LED_on() {
	GPIOB->ODR |=  (1 << 5); // set high
}

void set_LED_off() {
	GPIOB->ODR &= ~(1 << 5); // set low
}

void AS7343_default_config() {
//	UART_printf("\r\nLoading Default configuration.");

//	UART_printf("\r\n1. Disabling AS7343...");
	AS7343_disable(); // disable and turn off
//	UART_printf("DONE");
	/* Integration time and ADC sensitivity */
//	UART_printf("\r\n2. Setting ATIME to 0...");
	AS7343_set_ATIME(0);
//	UART_printf("DONE");

//	UART_printf("\r\n2. Setting ASTEP to 999...");
	AS7343_set_ASTEP(999);
//	UART_printf("DONE");

//	UART_printf("\r\n3. Setting AGAIN to 256X...");
	AS7343_set_AGAIN(AS7343_GAIN_256X);
//	UART_printf("DONE");

	/* Auto SMUX mode */
//	UART_printf("\r\n4. Setting SMUX to 18-channel mode...");
	AS7343_auto_smux(mode2_18ch);
//	UART_printf("DONE");

	/* Enable Wait between measurements */
//	UART_printf("\r\n5. Enabling wait between measurements...");
	AS7343_wait_enable();
//	UART_printf("DONE\r\n");
}

void AS7343_enable() {
	AS7343_write(0b10000000, 0x13);
}

void AS7343_disable() {
	AS7343_write(AS7343_ENABLE, 0x00);
}

void AS7343_reset() {
	AS7343_write(AS7343_CONTROL, 1 << 3);
}

void AS7343_wait_enable() {
	uint8_t temp = AS7343_read(AS7343_ENABLE);
	uint8_t WEN = 0x08;
	AS7343_write(AS7343_ENABLE, temp | WEN);
}

void AS7343_wait_disable() {
	uint8_t temp = AS7343_read(AS7343_ENABLE);
	uint8_t WEN = 0x08;
	AS7343_write(AS7343_ENABLE, temp | WEN);
}

void AS7343_set_wait_time(uint8_t WTIME) {
	AS7343_write(AS7343_WTIME, WTIME);
}

void AS7343_enable_LED() {
	uint8_t LED_strength = AS7343_get_LED_strength();
	uint8_t LED_enable = 0b10000000;
	AS7343_write(AS7343_LED, LED_enable | LED_strength);
}

void AS7343_disable_LED() {
	uint8_t LED_strength = AS7343_get_LED_strength();
	uint8_t LED_disable = 0b00000000;
	AS7343_write(AS7343_LED, LED_disable | LED_strength);
}

void AS7343_set_LED_strength(uint8_t LED_strength) {
	AS7343_write(AS7343_LED, LED_strength & 0x7F);
}

void AS7343_auto_smux(auto_smux_mode auto_smux_mode) {
	switch (auto_smux_mode) {
		case mode0_6ch:
			AS7343_write(AS7343_CFG20, (0b00 << 5));
		case mode1_12ch:
			AS7343_write(AS7343_CFG20, (0b10 << 5));
		case mode2_18ch:
			AS7343_write(AS7343_CFG20, (0b11 << 5));
	}
}

void AS7343_set_ASTEP(uint16_t astep) {
	if ((astep == 0) && (AS7343_get_ATIME() == 0)) {
		AS7343_write_2b(AS7343_ASTEP_L, (uint16_t) 0x0001);
	} else { AS7343_write_2b(AS7343_ASTEP_L, astep); }
}


void AS7343_set_ATIME(uint8_t atime) {
	if ((atime == 0) && (AS7343_get_ASTEP() == 0)) {
			AS7343_write(AS7343_ATIME, 0x01); // do not allow ATIME = ASTEP = 0 as per datasheet
	} else { AS7343_write(AS7343_ATIME, atime); }
}

void AS7343_set_AGAIN(AS7343_gain_t gain) {
	AS7343_write(AS7343_CFG1, gain);
}


int AS7343_done() {
	int reg_val = (int) AS7343_read(AS7343_STATUS2);
	int ret_val = reg_val & 0b01000000;
	return (ret_val >> 3);
}

int AS7343_DSat() {
	int reg_val = (int) AS7343_read(AS7343_STATUS2);
	int ret_val = reg_val & 0b00010000;
	return (ret_val >> 4);
}

int AS7343_ASat() {
	int reg_val = (int) AS7343_read(AS7343_STATUS2);
	int ret_val = reg_val & 0b00001000;
	return (ret_val >> 3);
}

void AS7343_get_basic_spectrum_optimized(float channel_readings[12], int max_loops) {
	uint16_t raw_spectrum[12];
	AS7343_get_raw_spectrum_optimized(raw_spectrum, max_loops);
	AS7343_raw_to_basic(raw_spectrum, channel_readings);
}

void AS7343_get_raw_spectrum_optimized(uint16_t channel_readings[12], int max_loops) {
	AS7343_enable();
	UART_printf("\r\n\nOPTIMIZED ROUTINE START ----------------------------------------");
	AS7343_default_config(); // reset config

	// for tracking variables
	uint8_t  	  curr_ATIME = AS7343_get_ATIME();
	uint16_t 	  curr_ASTEP = AS7343_get_ASTEP();
	AS7343_gain_t curr_AGAIN = AS7343_get_AGAIN();

	float factor; // = ADC_MAX/OUT_MAX, measure of how far output is from max possible

	AS7343_get_raw_spectrum(channel_readings);
	uint16_t OUT_MAX = maxValue(channel_readings, 12);

	UART_printf("\r\nInitial measurement done.");
//	UART_printf("\rCurrent max: %d\n", OUT_MAX);

	int isDigSat = AS7343_DSat(); // digital saturation
	int isAnaSat = AS7343_ASat(); // analog saturation

	// flags indicating if parameter can be/was adjusted accordingly
	int okATIME = 1;
	int okASTEP = 1;
	int okAGAIN = 1;

	int loop = 0;

	while (loop < max_loops) {
		++loop;
		UART_printf("\r\n\nITERATION #%d", loop);
		UART_printf("\r\nCURRENT MAX: %d", OUT_MAX);

//		UART_printf("\r\nLoop %2d:", loop);
//		UART_printf("\r\nATIME: %3d", curr_ATIME);
//		UART_printf("\r\nASTEP: %3d", curr_ASTEP);
//		UART_printf("\r\nAGAIN: %3d\n", curr_AGAIN);
//		UART_printf("\r\nCurrent max: %d", OUT_MAX);

		// NO SATURATION OCCURED ----------------------------------------------------------------//
		if (!isDigSat && !isAnaSat) { // if no saturation occurs
			if (OUT_MAX > 30000) {
				UART_printf("\rLoop exited: Sufficiently large values.");
				break; } // if output is sufficiently large, we are done
			if (loop >= max_loops) {
				UART_printf("\rLoop exited: Max number of loops.");
				break; } // if already at limit, do not further adjust values
											  // and risk saturating final output. stick to latest
											  // non-saturated measurements.

			// ELSE, DO RE-CONFIGURATION --------------------------------------------------------//
			if (OUT_MAX <= 100) { factor = 10; }
			else { factor = 0xFFFF/OUT_MAX * 0.9; }

			// try to adjust ASTEP first
			if (curr_ASTEP == 0xFFFE) { okASTEP = 0; } // if already max, adjust ATIME instead
			else if (u16_f_mulOvf(curr_ASTEP + 1, factor)) { // check first if will overflow
				curr_ASTEP = 0xFFFE;					 	 // if yes, set to max
				AS7343_set_ASTEP(curr_ASTEP);
				okASTEP = 1;
			} else {
				curr_ASTEP = (curr_ASTEP + 1)*factor - 1;
				AS7343_set_ASTEP(curr_ASTEP);
				okASTEP = 1;
			}

			// try to adjust ATIME
			if (!okASTEP) { // ONLY if ASTEP couldn't be adjusted
				if (curr_ATIME == 0xFF) { okATIME = 0; } // if also already max, adjust AGAIN instead
				else if (u8_f_mulOvf(curr_ATIME + 1, factor)) { // check if will overflow
					curr_ATIME = 0xFF;						 // if yes, set to max
					AS7343_set_ATIME(curr_ATIME);
					okATIME = 1;
				} else {
					curr_ATIME = (curr_ATIME + 1)*factor - 1;
					AS7343_set_ATIME(curr_ATIME);
					okATIME = 1;
				}
			}

			// try to adjust AGAIN
			if (!okATIME) { // ONLY if ATIME couldn't be adjusted
				if (curr_AGAIN == 12) { okAGAIN = 0; } // if also already max, update flag
				else {
					curr_AGAIN += 1;
					AS7343_set_AGAIN(curr_AGAIN);
					okAGAIN = 1;
				}
			}

			if (!okASTEP && !okATIME && !okAGAIN) {
				UART_printf("\r\nLoop exited: (sat:none) Can no longer adjust values.\n");
				break; } // if nothing can be adjusted, tama na

		// SOME SATURATION OCCURED --------------------------------------------------------------//
		} else {
			// check for both digital and analog saturation
			if (isDigSat) { // if digital saturated
				if (curr_AGAIN == 0) { okAGAIN = 0; } // if already min (unlikely), adjust T_int (ASTEP or ATIME)
				else {
					curr_AGAIN += 1;
					AS7343_set_AGAIN(curr_AGAIN);
					okAGAIN = 1;
				}
			}

			if (isAnaSat || !okAGAIN) { // if analog saturated, or AGAIN could not be adjusted
				// try to adjust ASTEP first
				if (curr_ASTEP == 0) { okASTEP = 0; } // if already min, adjust ATIME instead
				else {
					curr_ASTEP /= 2;
					AS7343_set_ASTEP(curr_ASTEP);
					okASTEP = 1;
				}

				// try to adjust ATIME
				if (!okASTEP) { // ONLY if ASTEP couldn't be adjusted
					if (curr_ATIME == 0) { okATIME = 0; } // if already min, update flag
					else {
						curr_ATIME /= 2;
						AS7343_set_ASTEP(curr_ATIME);
						okATIME = 1;
					}
				}

				// scenarios in which kailangan na natin 'to itigil
				if (!isDigSat && !okASTEP && !okATIME) {
					UART_printf("\rLoop exited: (sat:ana) values can no longer be adjusted.");
					break;} // only analog saturation, but ATIME & ASTEP could not be adjusted
				if (!okAGAIN && !okASTEP && !okATIME) {
					UART_printf("\rLoop exited: (sat:both) values can no longer be adjusted.");
					break;} // both saturation, none can be adjusted
			}
		}

		// take new measurement; update vals and flags
		AS7343_get_raw_spectrum(channel_readings);
		OUT_MAX = maxValue(channel_readings, 12);

		isDigSat = AS7343_DSat(); // digital saturation
		isAnaSat = AS7343_ASat(); // analog saturation

	} // end of while{}

	UART_printf("\r\n\nOptimized configuration -----------------");
	UART_printf("\r\nMeasurements: %d", loop);
	UART_printf("\r\nATIME: %3d", curr_ATIME);
	UART_printf("\r\nASTEP: %3d", curr_ASTEP);
	UART_printf("\r\nAGAIN: %3d\n", curr_AGAIN);

} // AS7343_get_raw_spectrum_optimized()

void AS7343_get_basic_spectrum(float basic_spectrum[12]) {
	uint16_t raw_spectrum[12];
	AS7343_get_raw_spectrum(raw_spectrum);
	AS7343_raw_to_basic(raw_spectrum, basic_spectrum);
}

void AS7343_get_raw_spectrum(uint16_t channel_readings[12]) {
	AS7343_enable();

	while (!AS7343_done()); // wait for measurement to finish
	AS7343_read_spectrum(channel_readings);

	AS7343_disable();
} // AS7343_get_raw_spectrum()


void AS7343_read_spectrum(uint16_t channel_readings[12]) {
	while (!AS7343_readings_valid()) {} // wait for readings to be valid
	int i;
	for (i = 0; i < 12; i++) {
		channel_readings[i] = AS7343_read_channel(vis_nir_channels[i]);
	}
}

void AS7343_raw_to_basic(uint16_t raw_spectrum[12], float basic_spectrum[12]) {
	AS7343_gain_t gain = AS7343_get_AGAIN();
	float t_int = (AS7343_get_ATIME() + 1) * (AS7343_get_ASTEP() + 1);

	for (int i=0 ; i < sizeof(raw_spectrum)/2; ++i) {
		basic_spectrum[i] = (float) raw_spectrum[i]/(gain*t_int); // formula from EVK user manual
	}
}

uint16_t AS7343_read_channel(AS7343_color_channel_t channel) {
	uint16_t ret_val = 0x0000;
	uint8_t reg_read_val = 0x00;

	// channels are arranged in spectral order, not accdg to data register order
	switch (channel)
	{
		case AS7343_CHANNEL_F1:
			ret_val = AS7343_read_2b(AS7343_CH12_DATA_L);
			break;

		case AS7343_CHANNEL_F2:
			ret_val = AS7343_read_2b(AS7343_CH6_DATA_L);
			break;

		case AS7343_CHANNEL_FZ:
			ret_val = AS7343_read_2b(AS7343_CH0_DATA_L);
			break;

		case AS7343_CHANNEL_F3:
			ret_val = AS7343_read_2b(AS7343_CH7_DATA_L);
			break;

		case AS7343_CHANNEL_F4:
			ret_val = AS7343_read_2b(AS7343_CH8_DATA_L);
			break;

		case AS7343_CHANNEL_F5:
			reg_read_val = AS7343_read(AS7343_CH13_DATA_L);
			ret_val |= (reg_read_val << 0);
			reg_read_val = AS7343_read(AS7343_CH13_DATA_H);
			ret_val |= (reg_read_val << 8);
			break;

		case AS7343_CHANNEL_FY:
			reg_read_val = AS7343_read(AS7343_CH1_DATA_L);
			ret_val |= (reg_read_val << 0);
			reg_read_val = AS7343_read(AS7343_CH1_DATA_H);
			ret_val |= (reg_read_val << 8);
			break;

		case AS7343_CHANNEL_FXL:
			reg_read_val = AS7343_read(AS7343_CH2_DATA_L);
			ret_val |= (reg_read_val << 0);
			reg_read_val = AS7343_read(AS7343_CH2_DATA_H);
			ret_val |= (reg_read_val << 8);
			break;

		case AS7343_CHANNEL_F6:
			reg_read_val = AS7343_read(AS7343_CH9_DATA_L);
			ret_val |= (reg_read_val << 0);
			reg_read_val = AS7343_read(AS7343_CH9_DATA_H);
			ret_val |= (reg_read_val << 8);
			break;

		case AS7343_CHANNEL_F7:
			reg_read_val = AS7343_read(AS7343_CH14_DATA_L);
			ret_val |= (reg_read_val << 0);
			reg_read_val = AS7343_read(AS7343_CH14_DATA_H);
			ret_val |= (reg_read_val << 8);
			break;

		case AS7343_CHANNEL_F8:
			reg_read_val = AS7343_read(AS7343_CH15_DATA_L);
			ret_val |= (reg_read_val << 0);
			reg_read_val = AS7343_read(AS7343_CH15_DATA_H);
			ret_val |= (reg_read_val << 8);
			break;

		case AS7343_CHANNEL_NIR:
			reg_read_val = AS7343_read(AS7343_CH3_DATA_L);
			ret_val |= (reg_read_val << 0);
			reg_read_val = AS7343_read(AS7343_CH3_DATA_H);
			ret_val |= (reg_read_val << 8);
			break;

		// OTHER CHANNELS --------
		case AS7343_CHANNEL_CLR_0:
			reg_read_val = AS7343_read(AS7343_CH16_DATA_L);
			ret_val |= (reg_read_val << 0);
			reg_read_val = AS7343_read(AS7343_CH16_DATA_H);
			ret_val |= (reg_read_val << 8);
			break;

		case AS7343_CHANNEL_CLR_1:
			reg_read_val = AS7343_read(AS7343_CH10_DATA_L);
			ret_val |= (reg_read_val << 0);
			reg_read_val = AS7343_read(AS7343_CH10_DATA_H);
			ret_val |= (reg_read_val << 8);
			break;

		case AS7343_CHANNEL_CLR_2:
			reg_read_val = AS7343_read(AS7343_CH4_DATA_L);
			ret_val |= (reg_read_val << 0);
			reg_read_val = AS7343_read(AS7343_CH4_DATA_H);
			ret_val |= (reg_read_val << 8);
			break;

		case AS7343_CHANNEL_FD_0:
			reg_read_val = AS7343_read(AS7343_CH17_DATA_L);
			ret_val |= (reg_read_val << 0);
			reg_read_val = AS7343_read(AS7343_CH17_DATA_H);
			ret_val |= (reg_read_val << 8);
			break;

		case AS7343_CHANNEL_FD_1:
			reg_read_val = AS7343_read(AS7343_CH11_DATA_L);
			ret_val |= (reg_read_val << 0);
			reg_read_val = AS7343_read(AS7343_CH11_DATA_H);
			ret_val |= (reg_read_val << 8);
			break;

		case AS7343_CHANNEL_FD_2:
			reg_read_val = AS7343_read(AS7343_CH5_DATA_L);
			ret_val |= (reg_read_val << 0);
			reg_read_val = AS7343_read(AS7343_CH5_DATA_H);
			ret_val |= (reg_read_val << 8);
			break;
	}

	return ret_val;
}

int AS7343_readings_valid() {
	return (AS7343_read(AS7343_STATUS2) & 0x40) >> 6;
}

void AS7343_get_readings_params(uint16_t ASTEP, uint8_t ATIME, AS7343_gain_t AGAIN) {
	ASTEP = AS7343_get_ASTEP();
	ATIME = AS7343_get_ATIME();
	AGAIN = AS7343_get_AGAIN();
}

uint16_t AS7343_get_ASTEP() {
	uint16_t ret_val = AS7343_read_2b(AS7343_ASTEP_L);
	return ret_val;
}

uint8_t AS7343_get_ATIME() {
	return AS7343_read(AS7343_ATIME);
}

AS7343_gain_t AS7343_get_AGAIN() {
	return (AS7343_read(AS7343_CFG1) & 0x0F); // mask reserved bits
}

uint8_t AS7343_get_LED_strength() {
	return (AS7343_read(AS7343_LED) & 0x7F);
}

void AS7343_write_2b(uint8_t reg_addr_low, uint16_t data) {
	uint8_t data_low_byte = data | 0x00;
	uint8_t data_high_byte = (data>>8) | 0x00;

	AS7343_write(reg_addr_low, data_low_byte);
	AS7343_write(reg_addr_low+1, data_high_byte);
}

void AS7343_write(uint8_t reg_addr, uint8_t data) {
	I2C1_start();

	I2C1_transmit_addr(AS7343_CHIP_ADDR_WRITE); // chip-addresswrite

	while (!(I2C1->SR1 & (1 << 7))); // TxE: wait for Tx to Empty
	I2C1_transmit(reg_addr);

	I2C1_transmit(data);

	I2C1_stop();
}

uint16_t AS7343_read_2b(uint8_t reg_addr_lower_byte) {
	uint16_t ret_val = 0x0000;
	uint8_t low_byte = AS7343_read(reg_addr_lower_byte);
	uint8_t high_byte = AS7343_read(reg_addr_lower_byte+1);
	ret_val = (high_byte << 8) | low_byte;

	return ret_val;
}

uint8_t AS7343_read(uint8_t reg_addr) {
	I2C1_start(); // start
	I2C1_transmit_addr(AS7343_CHIP_ADDR_WRITE); // slave addr (write)
	I2C1_transmit(reg_addr); // reg address

	I2C1_start(); // restart
	I2C1_transmit_addr(AS7343_CHIP_ADDR_READ); // slave addr (read)

	uint8_t ret_val = I2C1_receive();

	return ret_val;

}

uint16_t maxValue(uint16_t array[], size_t size) {
    size_t i;
    int maxValue = (int) array[0];

    for (i = 1; i < size; ++i) {
        if ( array[i] > maxValue ) {
            maxValue = (int) array[i];
        }
    }
    return (uint16_t) maxValue;
}

uint16_t minValue(uint16_t array[], size_t size) {
    size_t i;
    int minValue = (int) array[0];

    for (i = 1; i < size; ++i) {
        if ( array[i] < minValue ) {
            minValue = (int) array[i];
        }
    }
    return (uint16_t) minValue;
}

uint32_t arraySum(uint16_t array[], size_t size) {
    size_t i;
    uint32_t sum = 0x00000000;

    for (i = 0; i < size; ++i) {
        sum += (uint32_t) array[i];
    }
    return sum;
}


int u16_f_mulOvf(uint16_t a, float b) {
	return (a > (0xFFFE/b));
}

int u8_f_mulOvf(uint8_t a, float b) {
	return (a > (0xFF/b));
}

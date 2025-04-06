/*
 * AS7343.h
 *
 *  Created on: Feb 6, 2025
 *      Author: moreypiatos
 */

#ifndef AS7343_H_
#define AS7343_H_

#include <stdint.h>
#include <stm32f4xx.h>
#include <stm32f411xe.h>

#define AS7343_SLAVE_ADDR 0x39 		///< AS7343 i2c slave address
#define AS7343_CHIP_ADDR_WRITE 0x72 ///< AS7343 i2c addr for write (slave_addr << 1)
#define AS7343_CHIP_ADDR_READ  0x73 ///< AS7343 i2c addr for read  (slave_addr << 1) + 0x01
#define AS7343_CHIP_ID 0x81    		///< AS7343 default device id from datasheet

#define AS7343_WHOAMI 0x5A ///< Chip ID register

#define AS7343_CONFIG 0x70 ///< Enables LED control and sets light sensing mode
#define AS7343_STAT 0x71   ///< AS7343_STAT (unused)
#define AS7343_EDGE 0x72   ///< AS7343_EDGE (unused)
#define AS7343_GPIO 0x73   ///< Connects photo diode to GPIO or INT pins
#define AS7343_LED 0xCD    ///< LED Register; Enables and sets current limit
#define AS7343_ENABLE                                                          \
  0x80 ///< Main enable register. Controls SMUX, Flicker Detection, Spectral
       ///< Measurements and Power
#define AS7343_ATIME 0x81       ///< Sets ADC integration step count
#define AS7343_WTIME 0x83		///< Sets wait time between consecutive measurements
#define AS7343_SP_LOW_TH_L 0x84 ///< Spectral measurement Low Threshold low byte
#define AS7343_SP_LOW_TH_H                                                     \
  0x85 ///< Spectral measurement Low Threshold high byte
#define AS7343_SP_HIGH_TH_L                                                    \
  0x86 ///< Spectral measurement High Threshold low byte
#define AS7343_SP_HIGH_TH_H                                                    \
  0x87                    ///< Spectral measurement High Threshold low byte
#define AS7343_AUXID 0x58 ///< AS7343_AUXID (unused)
#define AS7343_REVID 0x59 ///< AS7343_REVID (unused)
#define AS7343_ID 0x92    ///< AS7343_ID (unused)
#define AS7343_STATUS                                                          \
  0x93 ///< Interrupt status registers. Indicates the occurence of an interrupt
#define AS7343_ASTATUS_ 0x94   ///< AS7343_ASTATUS, same as 0x60 (unused)
#define AS7343_CH0_DATA_L 0x95 ///< ADC Channel Data
#define AS7343_CH0_DATA_H 0x96 ///< ADC Channel Data
#define AS7343_CH1_DATA_L 0x97 ///< ADC Channel Data
#define AS7343_CH1_DATA_H 0x98 ///< ADC Channel Data
#define AS7343_CH2_DATA_L 0x99 ///< ADC Channel Data
#define AS7343_CH2_DATA_H 0x9A ///< ADC Channel Data
#define AS7343_CH3_DATA_L 0x9B ///< ADC Channel Data
#define AS7343_CH3_DATA_H 0x9C ///< ADC Channel Data
#define AS7343_CH4_DATA_L 0x9D ///< ADC Channel Data
#define AS7343_CH4_DATA_H 0x9E ///< ADC Channel Data
#define AS7343_CH5_DATA_L 0x9F ///< ADC Channel Data
#define AS7343_CH5_DATA_H 0xA0 ///< ADC Channel Data
#define AS7343_CH6_DATA_L 0xA1 ///< ADC Channel Data
#define AS7343_CH6_DATA_H 0xA2 ///< ADC Channel Data
#define AS7343_CH7_DATA_L 0xA3 ///< ADC Channel Data
#define AS7343_CH7_DATA_H 0xA4 ///< ADC Channel Data
#define AS7343_CH8_DATA_L 0xA5 ///< ADC Channel Data
#define AS7343_CH8_DATA_H 0xA6 ///< ADC Channel Data
#define AS7343_CH9_DATA_L 0xA7 ///< ADC Channel Data
#define AS7343_CH9_DATA_H 0xA8 ///< ADC Channel Data
#define AS7343_CH10_DATA_L 0xA9 ///< ADC Channel Data
#define AS7343_CH10_DATA_H 0xAA ///< ADC Channel Data
#define AS7343_CH11_DATA_L 0xAB ///< ADC Channel Data
#define AS7343_CH11_DATA_H 0xAC ///< ADC Channel Data
#define AS7343_CH12_DATA_L 0xAD ///< ADC Channel Data
#define AS7343_CH12_DATA_H 0xAE ///< ADC Channel Data
#define AS7343_CH13_DATA_L 0xAF ///< ADC Channel Data
#define AS7343_CH13_DATA_H 0xB0 ///< ADC Channel Data
#define AS7343_CH14_DATA_L 0xB1 ///< ADC Channel Data
#define AS7343_CH14_DATA_H 0xB2 ///< ADC Channel Data
#define AS7343_CH15_DATA_L 0xB3 ///< ADC Channel Data
#define AS7343_CH15_DATA_H 0xB4 ///< ADC Channel Data
#define AS7343_CH16_DATA_L 0xB5 ///< ADC Channel Data
#define AS7343_CH16_DATA_H 0xB6 ///< ADC Channel Data
#define AS7343_CH17_DATA_L 0xB7 ///< ADC Channel Data
#define AS7343_CH17_DATA_H 0xB8 ///< ADC Channel Data

#define AS7343_STATUS2 0x90 ///< Measurement status flags; saturation, validity
#define AS7343_STATUS3                                                         \
  0x91 ///< Spectral interrupt source, high or low threshold
#define AS7343_STATUS5 0xBB ///< AS7343_STATUS5 (unused)
#define AS7343_STATUS4 0xBC ///< AS7343_STATUS6 (unused)
#define AS7343_CFG0                                                            \
  0xBF ///< Sets Low power mode, Register bank, and Trigger lengthening
#define AS7343_CFG1 0xC6 ///< Controls ADC Gain
#define AS7343_CFG3 0xC7 ///< AS7343_CFG3 (unused)
#define AS7343_CFG6 0xF5 ///< Used to configure Smux
#define AS7343_CFG8 0xC9 ///< AS7343_CFG8 (unused)
#define AS7343_CFG9                                                            \
  0xCA ///< Enables flicker detection and smux command completion system
       ///< interrupts
#define AS7343_CFG10 0x65 ///< AS7343_CFG10 (unused)
#define AS7343_CFG12                                                           \
  0x66 ///< Spectral threshold channel for interrupts, persistence and auto-gain
#define AS7343_CFG20 0xD6 //< FIFO and auto SMUX
#define AS7343_PERS                                                            \
  0xCF ///< Number of measurement cycles outside thresholds to trigger an
       ///< interupt
#define AS7343_GPIO2                                                           \
  0x6B ///< GPIO Settings and status: polarity, direction, sets output, reads
       ///< input
#define AS7343_ASTEP_L 0xD4      ///< Integration step size low byte
#define AS7343_ASTEP_H 0xD5      ///< Integration step size high byte
#define AS7343_AGC_GAIN_MAX 0xD7 ///< AS7343_AGC_GAIN_MAX (unused)
#define AS7343_AZ_CONFIG 0xDE    ///< AS7343_AZ_CONFIG (unused)
#define AS7343_FD_TIME1 0xE0 ///< Flicker detection integration time low byte
#define AS7343_FD_TIME2 0xE2 ///< Flicker detection gain and high nibble
#define AS7343_FD_CFG0 0xDF  ///< AS7343_FD_CFG0 (unused)
#define AS7343_FD_STATUS                                                       \
  0xE3 ///< Flicker detection status; measurement valid, saturation, flicker
       ///< type
#define AS7343_INTENAB 0xF9  ///< Enables individual interrupt types
#define AS7343_CONTROL 0xFA  ///< Auto-zero, fifo clear, clear SAI active
#define AS7343_FIFO_MAP 0xFC ///< AS7343_FIFO_MAP (unused)
#define AS7343_FIFO_LVL 0xFD ///< AS7343_FIFO_LVL (unused)
#define AS7343_FDATA_L 0xFE  ///< AS7343_FDATA_L (unused)
#define AS7343_FDATA_H 0xFF  ///< AS7343_FDATA_H (unused)

#define AS7343_SPECTRAL_INT_HIGH_MSK                                           \
  0b00100000 ///< bitmask to check for a high threshold interrupt
#define AS7343_SPECTRAL_INT_LOW_MSK                                            \
  0b00010000 ///< bitmask to check for a low threshold interrupt

/**
 * @brief Allowable gain multipliers for `setGain`
 *
 */
typedef enum {
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
} AS7343_gain_t;


/**
 * @brief Available SMUX configuration commands
 *
 */
typedef enum {
  AS7343_SMUX_CMD_ROM_RESET, ///< ROM code initialization of SMUX
  AS7343_SMUX_CMD_READ,      ///< Read SMUX configuration to RAM from SMUX chain
  AS7343_SMUX_CMD_WRITE, ///< Write SMUX configuration from RAM to SMUX chain
} AS7343_smux_cmd_t;
/**
 * @brief ADC Channel specifiers for configuration
 *
 */
typedef enum {
  AS7343_ADC_CHANNEL_0,
  AS7343_ADC_CHANNEL_1,
  AS7343_ADC_CHANNEL_2,
  AS7343_ADC_CHANNEL_3,
  AS7343_ADC_CHANNEL_4,
  AS7343_ADC_CHANNEL_5,
} AS7343_adc_channel_t;
/**
 * @brief Spectral Channel specifiers for configuration and reading
 *
 */
typedef enum {
  AS7343_CHANNEL_FZ,	///< ch00
  AS7343_CHANNEL_FY,	///< ch01
  AS7343_CHANNEL_FXL,	///< ch02
  AS7343_CHANNEL_NIR,	///< ch03
  AS7343_CHANNEL_CLR_2,	///< ch04
  AS7343_CHANNEL_FD_2,	///< ch05
  AS7343_CHANNEL_F2,	///< ch06
  AS7343_CHANNEL_F3,	///< ch07
  AS7343_CHANNEL_F4,	///< ch08
  AS7343_CHANNEL_F6,	///< ch09
  AS7343_CHANNEL_CLR_1,	///< ch10
  AS7343_CHANNEL_FD_1,	///< ch11
  AS7343_CHANNEL_F1,	///< ch12
  AS7343_CHANNEL_F5,	///< ch13
  AS7343_CHANNEL_F7,	///< ch14
  AS7343_CHANNEL_F8,	///< ch15
  AS7343_CHANNEL_CLR_0,	///< ch16
  AS7343_CHANNEL_FD_0,	///< ch17
} AS7343_color_channel_t;

/**
 * @brief The number of measurement cycles with spectral data outside of a
 * threshold required to trigger an interrupt
 *
 */
typedef enum {
  AS7343_INT_COUNT_ALL, ///< 0
  AS7343_INT_COUNT_1,   ///< 1
  AS7343_INT_COUNT_2,   ///< 2
  AS7343_INT_COUNT_3,   ///< 3
  AS7343_INT_COUNT_5,   ///< 4
  AS7343_INT_COUNT_10,  ///< 5
  AS7343_INT_COUNT_15,  ///< 6
  AS7343_INT_COUNT_20,  ///< 7
  AS7343_INT_COUNT_25,  ///< 8
  AS7343_INT_COUNT_30,  ///< 9
  AS7343_INT_COUNT_35,  ///< 10
  AS7343_INT_COUNT_40,  ///< 11
  AS7343_INT_COUNT_45,  ///< 12
  AS7343_INT_COUNT_50,  ///< 13
  AS7343_INT_COUNT_55,  ///< 14
  AS7343_INT_COUNT_60,  ///< 15
} AS7343_int_cycle_count_t;

/**
 * @brief Pin directions to set how the GPIO pin is to be used
 *
 */
typedef enum {
  AS7343_GPIO_OUTPUT, ///< The GPIO pin is configured as an open drain output
  AS7343_GPIO_INPUT,  ///< The GPIO Pin is set as a high-impedence input
} AS7343_gpio_dir_t;

/**
 * @brief Wait states for async reading
 */
typedef enum {
  AS7343_WAITING_START, //
  AS7343_WAITING_LOW,   //
  AS7343_WAITING_HIGH,  //
  AS7343_WAITING_DONE,  //
} AS7343_waiting_t;

typedef enum {
  mode0_6ch,	///<          FZ, FY, FXL, NIR, 2xVIS, Clear
  mode1_12ch,	///< Cycle 1: FZ, FY, FXL, NIR, 2xVIS, Clear
  	  	  	  	///< Cycle 2: F2, F3,  F4,  F6, 2xVIS, Clear
  mode2_18ch,	///< Cycle 1: FZ, FY, FXL, NIR, 2xVIS, Clear
	  			///< Cycle 2: F2, F3,  F4,  F6, 2xVIS, Clear
				///< Cycle 3: F1, F5,  F7,  F8, 2xVIS, Clear
} auto_smux_mode;



////////////////////////////////////////////////////////////////////////////////
///////////////////////////// FUNCTION DEFINITIONS /////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void Init_AS7343();

void Init_AS7343_GPIO();

void AS7343_default_config();

/**
 * @brief Simultaneously powers on AS7343 and
 * 	      enables spectral measurement.
 * */
void AS7343_enable();

/**
 * @brief Simultaneously powers off AS7343 and
 * 	      disables spectral measurement.
 * */
void AS7343_disable();

/**
 * @brief Initiates reset for AS7343.
 * */
void AS7343_reset();

void AS7343_wait_enable();

void AS7343_wait_disable();

void AS7343_set_wait_time(uint8_t WTIME);

void AS7343_enable_LED();

void AS7343_disable_LED();

void AS7343_set_LED_strength(uint8_t LED_strength);


void AS7343_auto_smux(auto_smux_mode auto_smux_mode);

/**
 * @brief Sets the integration time per step in increments
 * 		  of 2.78 μs. The default value is 999.
 * */
void AS7343_set_ASTEP(uint16_t astep);

/**
 * @brief Sets the number of integration steps from
 * 		  1 to 255.
 * */
void AS7343_set_ATIME(uint8_t atime);

/**
 * @brief Sets the ADC gain.
 * */
void AS7343_set_AGAIN(AS7343_gain_t gain);

/**
 * @brief Checks whether current set of measured values
 * 	      have completed ADC and are ready to be read.
 * */
int AS7343_done();

/**
 * @brief Checks checks for analog saturation.
 * */
int AS7343_ASat();

/**
 * @brief Checks checks for digital saturation.
 * */
int AS7343_DSat();

void AS7343_get_basic_spectrum_optimized(float channel_readings[12], int max_loops);

/**
 * @brief Finds the best parameter configuration to take
 * 		    measurements with. Returns a set of measurements
 * 		    using the best parameters found.
 * @param channel_readings 12-long array of uint16_t where
 *        channel readings will be stored.
 * @param max_loops The maximum number of times the routine
 *        will retake measurements and readjust parameters.
 * @return none
 * */
void AS7343_get_raw_spectrum_optimized(uint16_t channel_readings[12], int max_loops);

void AS7343_get_basic_spectrum(float basic_spectrum[12]);

void AS7343_get_raw_spectrum(uint16_t channel_readings[12]);

/**
 * @brief Read all channels in the vis-NIR spectrum.
 * 		  This excludes the 3 clear and 3 dark channels.
 *
 * @param channel_readings
 * 		  Array of data type uint16_t and length 12 that
 * 		  will hold the channel data readings.
 * */
void AS7343_read_spectrum(uint16_t channel_readings[12]);

void AS7343_raw_to_basic(uint16_t raw_spectrum[12], float basic_spectrum[12]);

uint16_t AS7343_read_channel(AS7343_color_channel_t channel);

/**
 * @brief Obtains the settings (parameters) applied to the
 * 		  most recent measurement done.
 * @param ASTEP (uint16_t) where ASTEP will be stored
 * @param ATIME (uint8_t) where ATIME will be stored
 * @param AGAIN (AS7343_gain_t) where AGAIN will be stored
 */
void AS7343_get_readings_params(uint16_t ASTEP, uint8_t ATIME, AS7343_gain_t AGAIN);

uint16_t AS7343_get_ASTEP();

uint8_t AS7343_get_ATIME();

AS7343_gain_t AS7343_get_AGAIN();

uint8_t AS7343_get_LED_strength();

void AS7343_write(uint8_t reg_addr, uint8_t data);

uint8_t AS7343_read(uint8_t reg_addr);

uint16_t AS7343_read_2b(uint8_t reg_addr_lower_byte);

uint16_t maxValue(uint16_t array[], size_t size);

int u16_f_mulOvf(uint16_t a, float b);

int u8_f_mulOvf(uint8_t a, float b);

#endif /* AS7343_H_ */

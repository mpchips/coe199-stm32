/*
 * MCP4725.h
 *
 *  Created on: Mar 21, 2025
 *      Author: trishamarieherras
 *
 * Some definitions are adapted from https://github.com/SMotlaq/mcp4725
 *
 */

#ifndef INC_MCP4725_H_
#define INC_MCP4725_H_

#include <stdint.h>


/* dac addresses */
typedef enum
{
	MCP4725A0_ADDR_A00         = 0x60,		//i2c address, A0 = 0
	MCP4725A0_ADDR_A01         = 0x61,		//i2c address, A0 = 1

	MCP4725A1_ADDR_A00         = 0x62,		//i2c address, A0 = 0
	MCP4725A1_ADDR_A01         = 0x63,		//i2c address, A0 = 1

	MCP4725A2_ADDR_A00         = 0x64,		//i2c address, A0 = 0
	MCP4725A2_ADDR_A01         = 0x65		//i2c address, A0 = 1
} MCP4725Ax_ADDRESS;

typedef struct MCP
{
	uint8_t		_i2cAddress;
	float       _refVoltage;
	uint16_t    _bitsPerVolt;
} MCP4725;

/* Some arithmatic functions */
#define lowByte(x)		((uint8_t)(x>>0))
#define highByte(x)		((uint8_t)(x>>8))

/* dac general call command */
#define MCP4725_GENCALL_ADDRESS 	0x00                         //general call address
#define MCP4725_GENCALL_RESET   	0x06                         //general call hard reset command
#define MCP4725_GENCALL_WAKEUP      0x09                         //general call wake-up command

/* dac misc. */
#define MCP4725_RESOLUTION           12                           //resolution 12-bit
#define MCP4725_STEPS                4096 //pow(2, (MCP4725_RESOLUTION)) //quantity of DAC steps 2^12-bits = 4096
#define MCP4725_EEPROM_WRITE_TIME    25                           //non-volatile memory write time, maximum 50 msec

#define MCP4725_REFERENCE_VOLTAGE    3.30                         //supply-reference votltage
#define MCP4725_MAX_VALUE            4095 //((MCP4725_STEPS) - 1)
#define MCP4725_ERROR                0xFFFF                       //returns 65535, if communication error is occurred

/* function definitions */

MCP4725 	MCP4725_init(I2C_HandleTypeDef* hi2c, MCP4725Ax_ADDRESS addr, float refV);

void		MCP4725_setReferenceVoltage(MCP4725* _MCP4725, float value);
float		MCP4725_getReferenceVoltage(MCP4725* _MCP4725);

#endif /* INC_MCP4725_H_ */

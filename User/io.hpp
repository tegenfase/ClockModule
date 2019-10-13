/*
 * 	io.hpp
 *
 *  Created on: Sep 25, 2019
 *      Author: delta
 */

#ifndef IO_HPP_
#define IO_HPP_

#include "stm32f4xx_hal.h"
#include "main.h"

#define BIT_RW 			0b10000000
#define BIT_REG2 		0b00100000
#define BIT_REG1 		0b00010000
#define BIT_REG0 		0b00001000
#define BIT_A2			0b00000100
#define BIT_A1			0b00000010
#define BIT_A0			0b00000001

#define REGISTER_BITS 		0b00111000
#define DAC_REGISTER		0b00000000
#define RANGE_REGISTER		0b00001000
#define POWER_REGISTER		0b00010000
#define CONTROL_REGISTER	0b00011000

#define CHANNEL_BITS		0b00000111
#define DAC_CHANNEL_A				0b00000000
#define DAC_CHANNEL_B				0b00000001
#define DAC_CHANNEL_C				0b00000010
#define DAC_CHANNEL_D				0b00000011
#define DAC_CHANNEL_ALL				0b00000111

// These bits are set in the LSB byte.
#define RANGE_PLUS_5			0b00000000
#define	RANGE_PLUS_10			0b00000001
#define RANGE_PLUS_10_8 		0b00000010
#define RANGE_PLUS_MINUS_5		0b00000011
#define RANGE_PLUS_MINUS_10 	0b00000100
#define RANGE_PLUS_MINUS_10_8	0b00000101

#define POWER_UP_ALL_DACS		0b00001111


class IO{
public:
	IO(SPI_HandleTypeDef* spi);
	~IO(void);
	void setDacOutput(uint8_t chan, int16_t val);
	void setDacRegister(uint8_t reg, uint8_t chan, uint16_t val);
	void setDigitalOuts(uint8_t val, uint8_t n);
	void writeDacs(void);
	void writeDigitalOuts(void);
	void initDacs(uint8_t range);

private:
	uint16_t dacOutBuffer[4];
	uint8_t dacSpiBuffer[3];
	uint8_t digitalOutBuffer[2];
	SPI_HandleTypeDef* hspi;


};

#endif /* IO_HPP_ */

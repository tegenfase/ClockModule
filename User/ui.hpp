/*
 * ui.hpp
 *
 *  Created on: Feb 4, 2019
 *      Author: delta
 */
#include "stm32f4xx_hal.h"
#include <clock.hpp>
#include <vector>
#include <cstring>
#include <stdlib.h>


#ifndef UI_HPP_
#define UI_HPP_

#define NUM_OF_REGISTERS 3

void shiftRegInit(void);

enum PWM_MODE{
	OFF = 0,
	DIM = 2,
	HALF = 25,
	FULL = 127
};

enum PULSE_MODE{
	PULSE_MODE_OFF = 0,
	PULSE_MODE_16TH = 2,
	PULSE_MODE_8TH = 4,
	PULSE_MODE_4TH = 8,
	PULSE_MODE_2ND = 16
};


class Switches{
public:
	uint8_t offset;
	uint8_t numOfRegisters;
	uint8_t numOfStates;

	SPI_HandleTypeDef* spi_interface;

	std::vector<uint8_t> index;
	std::vector< std::vector<uint8_t> > state;
	std::vector<uint8_t> debouncedState;
	std::vector<uint8_t> changed;

	// Increment every update if button is not released;
	std::vector<uint8_t> holdTimer;

	std::vector<uint8_t> leds;
	std::vector<uint8_t> ledPWM;
	std::vector<uint8_t> ledPulse;

	Switches(uint8_t nRegisters, uint8_t nStates, uint8_t srOffset, SPI_HandleTypeDef* spi);
	~Switches(void);

	void readWriteRegisters(void);

	void spi(void);
	void update(int subTick);

	void setLed(int n, PWM_MODE pwm);
	void setLed(int n, PWM_MODE pwm, PULSE_MODE pulse);
	void setLedAll(PWM_MODE pwm);
	void setLedAll(PULSE_MODE pulse);
	void setLedAll(PWM_MODE pwm, PULSE_MODE pulse);
	void setLedByte(int offset,uint8_t byte, PWM_MODE pwm, PULSE_MODE pulse);
	void setLedArray(int offset, int n, uint8_t* mask, PWM_MODE pwm, PULSE_MODE pulse);

	bool pressed(void);
	uint8_t returnPressed(uint8_t index);
	bool released(void);
	uint8_t released(uint8_t);
	void currentState(uint8_t* state);
	bool hasChanged(void);
	uint8_t firstChanged(void);

};



#endif /* UI_HPP_ */


/*
 * clock.cpp
 *
 *  Created on: Jul 20, 2018
 *      Author: Douwe
 *
 *       Clock class syncs the master clock (TIM2) to an incoming rising edge signal
 *      or generates it's own interrupt ussing CCR1.
 *
 *      TIM2's period is then subdivided tp the slave clock (TIM5) which in its IRQ
 *      updates the sequencer step
 *
 *      see stm32f4xx_it.cpp for the routines in use
 *
 */
#include "main.h"
#include "stm32f4xx.h"


#ifndef CLOCK_HPP_
#define CLOCK_HPP_


enum ClockSource{
		INTERNAL,
		EXTERNAL
};

class Clock{
public:
	unsigned int period, subDiv, master, sub;
	unsigned char nthMeasurement;
	TIM_HandleTypeDef* masterTimer;
	TIM_HandleTypeDef* slaveTimer;
	ClockSource clockSource;
	bool lockState;

	// Constructor
	Clock(void);
	// Destructor
	~Clock(void);

	void setTimer(TIM_HandleTypeDef* master, TIM_HandleTypeDef* slave);

	// Set a period manually
	void setPeriod(void);
	void setPeriod(unsigned int value);

	void setSlaveDivision(unsigned int division);
	void setSource(ClockSource source);

	// Increment master clock;
	void masterTick(void);
	void subTick(void);
	// Get averaged period if locked, else return manually defined period
	volatile unsigned int getPeriod(void);

	// Is the clock locked?
	void lock(void);
	void unlock(void);
	bool isLocked(void);

private:
	unsigned int hello;
};




#endif /* CLOCK_HPP_ */

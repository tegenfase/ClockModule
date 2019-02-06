/*
 * clock.cpp
 *
 *  Created on: Jul 20, 2018
 *      Author: Douwe
 *
 *       Clock class syncs the master clock (TIM2) to an incoming rising edge signal
 *      or generates it's own interrupt using CCR1.
 *
 *
 *      TIM2's period is then subdivided to the slave clock (TIM5) which in its IRQ
 *      updates the sequencer's step state and outputs
 *
 *      see stm32f4xx_it.cpp for the routines in use
 *
 *      Improvements to be made:
 *      - Add swing
 *      - Add midi sync
 *      - Add i2c sync
 *      - Average input clock for stability?
 *
 */
#include "clock.hpp"
#include "stm32f4xx.h"

// Constructor
Clock::Clock(){
	;
}
// Destructor
Clock::~Clock(void){
	;
}
void Clock::masterTick(void){
	master++;
	sub = 0;
}
void Clock::subTick(void){
	if(sub < subDiv){
		sub++;
	}
	else{
		sub = subDiv;
	}
}
// This should happen in the constructor!
void Clock::setTimer(TIM_HandleTypeDef* master, TIM_HandleTypeDef* slave){
	masterTimer = master;
	slaveTimer = slave;
}

// Implement averaging routine! and Reset on master pulse.
// Maybe inline these functions!
void Clock::lock(){
	if(!lockState){
		lockState = true;
		sub = 0;
	}
}
void Clock::unlock(){
	lockState = false;
}
bool Clock::isLocked(void){
	return lockState;
}
void Clock::setPeriod(void){
	if(clockSource == EXTERNAL){
			period = masterTimer->Instance->CCR1;
	}
}
void Clock::setPeriod(unsigned int value){
	if(clockSource == INTERNAL){
		period = value;
		masterTimer->Instance->CCR2 = period;
	}
	else if(clockSource == EXTERNAL){
		period = masterTimer->Instance->CCR1;
		// masterTimer->Instance->CCR2 = period;
	}
}

void Clock::setSource(enum ClockSource source){
	clockSource = source;
	if(source == INTERNAL){
		// Do not use the external trigger
		HAL_TIM_IC_Stop_IT(masterTimer, TIM_CHANNEL_1);
		HAL_TIM_OC_Start_IT(masterTimer, TIM_CHANNEL_2);

	}
	else if(source == EXTERNAL){
		HAL_TIM_OC_Stop_IT(masterTimer, TIM_CHANNEL_2);
		HAL_TIM_IC_Start_IT(masterTimer, TIM_CHANNEL_1);

	}
}
volatile unsigned int Clock::getPeriod(void){
	return period;
}

void Clock::setSlaveDivision(unsigned int division){
	subDiv = division;
	// Bit hacky, to make sure there are only 'division' amounts of pulses in a period.
	// This may be necessary if a shoddy clock input is used
	slaveTimer->Instance->CNT = 0;
	slaveTimer->Instance->ARR = ((period)/division);//+(period%division);


}

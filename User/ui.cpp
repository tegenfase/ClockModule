/*
 * ui.cpp
 *
 *  Created on: Feb 4, 2019
 *      Author: delta
 */

#include "ui.hpp"
#include "stm32f4xx_hal.h"

uint8_t ui_spi_in[NUM_OF_REGISTERS];
uint8_t ui_spi_out[NUM_OF_REGISTERS];

void shiftRegInit(void){
	HAL_GPIO_WritePin(SRCLR_GPIO_Port,SRCLR_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LD_GPIO_Port,LD_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(RCLK_GPIO_Port, RCLK_Pin,GPIO_PIN_SET);
}
// Switch debouncer
Switches::Switches(uint8_t nRegisters, uint8_t nStates, uint8_t srOffset, SPI_HandleTypeDef* spi){
	numOfRegisters = nRegisters;
	numOfStates = nStates;
	offset = srOffset;

	spi_interface = spi;

	state.resize(numOfRegisters, std::vector<uint8_t>(numOfStates));
	debouncedState.resize(numOfRegisters);
	changed.resize(numOfRegisters);
	index.resize(numOfRegisters);

	holdTimer.resize(numOfRegisters*8);

	leds.resize(numOfRegisters);
	ledPWM.resize(numOfRegisters*8, 0);
	ledPulse.resize(numOfRegisters*8, 0);

}

Switches::~Switches(void){
	;
}

void Switches::spi(void){
		// Clear output shift register, unnecessary...
		// HAL_GPIO_WritePin(SRCLR_GPIO_Port,SRCLR_Pin,GPIO_PIN_RESET);
		// HAL_GPIO_WritePin(SRCLR_GPIO_Port,SRCLR_Pin,GPIO_PIN_SET);

		// Latch inputs



		HAL_SPI_TransmitReceive(spi_interface, ui_spi_out, ui_spi_in, NUM_OF_REGISTERS, 10);

		// Transfer shift register contents to storage register
		HAL_GPIO_WritePin(LD_GPIO_Port,LD_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD_GPIO_Port,LD_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(RCLK_GPIO_Port, RCLK_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RCLK_GPIO_Port, RCLK_Pin,GPIO_PIN_SET);
}
void Switches::update(int subTick){

	uint8_t lastDebouncedState[numOfRegisters];
	uint8_t ledMask[numOfRegisters];
	uint8_t i, x, j;
	static uint8_t PWMcounter;
	int shift;
	uint8_t step = subTick/12;

	PWMcounter++;
	for(x = 0; x < numOfRegisters; x++){
		// Dim and pulse leds
		for(j = 0; j<8; j++){
			shift = (x*8)+j;

			if(ledPulse[shift]){
				if((step%ledPulse[shift])){
					if(PWMcounter<ledPWM[shift]){
						ledMask[x] |= (1 << j);
					}
					else{
						ledMask[x] |= (1 << j);
						// ledMask[x] &= ~(1 << j);
					}
				}
				else{
					ledMask[x] |= (1 << j);
					// ledMask[x] &= ~(1 << j);
				}
			}
			else{
				if(PWMcounter<ledPWM[shift]){
					ledMask[x] |= (1 << j);
				}
				else{
					//ledMask[x] |= (1 << j);
					ledMask[x] &= ~(1 << j);
				}

			}
		}

		if(PWMcounter == 127){
			PWMcounter = 0;
		}

		// Push ledstate to spi out buffer
		ui_spi_out[NUM_OF_REGISTERS-(1+offset+x)] = leds[x] | ledMask[x];

		lastDebouncedState[x] = debouncedState[x];
		debouncedState[x] = 0xFF;

		// Read from the spi input buffer
		state[x][index[x]] = ui_spi_in[x+offset];

		// Debounce switches
		for(i = 0; i < numOfStates; i++){
			debouncedState[x] &= state[x][i];
		}

		index[x]++;
		if(index[x] >= numOfStates){
			index[x] = 0;
		}

		changed[x] = debouncedState[x] ^ lastDebouncedState[x];

	}
}

// ---------------------------- SWITCH FUNCTIONS  ----------------------------

bool Switches::hasChanged(void){
	uint8_t change = 0;
	for(uint8_t i = 0; i < numOfRegisters; i++){
		change |= changed[i];
	}
	if(change)
		return true;
	else
		return false;
}
uint8_t Switches::firstChanged(void){
	for(uint8_t i = 0; i < numOfRegisters; i++){
		for(int j = 0; j < 8; j++){
			if((changed[i] & (1 << j))){
				return (i*8)+j;
			}
		}
	}
	return -1;
}

uint8_t Switches::returnPressed(uint8_t index){
	return changed[index] & debouncedState[index];
}

bool Switches::pressed(void){
	uint8_t pressed = 0;
	for(uint8_t i = 0; i < numOfRegisters; i++){
		pressed |= changed[i] & debouncedState[i];
	}
	if(pressed != 0)
		return true;
	else
		return false;
}

bool Switches::released(void){
	uint8_t released = 0;
	for(uint8_t i = 0; i < numOfRegisters; i++){
		released |= changed[i] & ~(debouncedState[i]);
	}
	if(released)
		return true;
	else
		return false;
}
uint8_t Switches::released(uint8_t index){
	return changed[index] & ~(debouncedState[index]);
}



void Switches::currentState(uint8_t* state){
	for(uint8_t i = 0; i < numOfRegisters; i++){
		*(state+i) = debouncedState[i];
	}
}
void Switches::setLed(int n, PWM_MODE pwm){
	ledPWM[n] = pwm;
}
void Switches::setLed(int n, PWM_MODE pwm, PULSE_MODE pulse){
	ledPWM[n] = pwm;
	ledPulse[n] = pulse;
}

void Switches::setLedArray(int offset, int n, uint8_t* mask, PWM_MODE pwm, PULSE_MODE pulse){
	for(int i = offset; i < offset+n; i++){
		for(int j = 0; j < 8; j++){
			if(mask[i] & (1 << j)){
				int step = i*8+j;
				ledPulse[step] = pulse;
				ledPWM[step] = pwm;
			}
		}
	}
}
void Switches::setLedByte(int offset,uint8_t mask, PWM_MODE pwm, PULSE_MODE pulse){
		for(int j = 0; j < 8; j++){
			if(mask & (1 << j)){
				int step = (offset)*8+j;
				ledPulse[step] = pulse;
				ledPWM[step] = pwm;
			}
		}
}
void Switches::setLedAll(PWM_MODE pwm){
	for(int i = 0; i < numOfRegisters*8; i++){
		ledPWM[i] = pwm;
		ledPulse[i] = PULSE_MODE_OFF;
	}
}
void Switches::setLedAll(PULSE_MODE pulse){
	for(int i = 0; i < numOfRegisters*8; i++){
		ledPulse[i] = pulse;
		ledPWM[i] = FULL;
	}
}
void Switches::setLedAll(PWM_MODE pwm, PULSE_MODE pulse){
	for(int i = 0; i < numOfRegisters*8; i++){
		ledPWM[i] = pwm;
		ledPulse[i] = pulse;
	}
}

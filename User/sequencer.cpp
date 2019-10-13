/*
 * sequencer.cpp
 *
 *  Created on: Feb 4, 2019
 *      Author: delta
 */
#include <vector>
#include "sequencer.hpp"

Sequencer::Sequencer(uint8_t menuRegisters, uint8_t menuOffset, uint8_t stepRegisters, uint8_t stepOffset, SPI_HandleTypeDef* spi)
: stepSwitches(stepRegisters,4,stepOffset, spi), menuSwitches(menuRegisters,4,menuOffset, spi)
{
	// DEFAULT: all lanes play;
	playingLanes = 0xFFFF;

	for(int i = 0; i < 16; i++){
		for(int j = 0; j < 16; j++){
			song[0].lastStep[i][j] = 16;
		}

	}



}
void Sequencer::readWrite(void){
	// Check whether a function or step switch has been pressed

	uint16_t step = 0;
	if(menuSwitches.pressed()){
			switch(menuSwitches.returnPressed(0)){
						case PLAY:
							if(transportState == TRANSPORT_STATE_STOP){
								transportState = TRANSPORT_STATE_RESET;
								menuSwitches.leds[0] |= PLAY;
							}
							else{
								transportState = TRANSPORT_STATE_STOP;
								menuSwitches.leds[0] &= ~PLAY;
							}
							break;
						case PAUSE:
							break;
						case COPY:
							break;
						case PASTE:
							break;
						case SELECT_BLOCK:
							menuSwitches.setLedByte(0,SELECT_BLOCK|SELECT_LANE|SELECT_NOTE|SELECT_SONG,OFF, PULSE_MODE_OFF);
							menuSwitches.setLedByte(0,SELECT_BLOCK,HALF,PULSE_MODE_4TH);
							stepSwitches.setLedAll(OFF,PULSE_MODE_OFF);
							stepSwitches.setLed(playingBlock, HALF, PULSE_MODE_4TH);
							stepSwitchMode = STEPSWITCH_SELECT_BLOCK;
							break;
						case SELECT_LANE:
							menuSwitches.setLedByte(0,SELECT_BLOCK|SELECT_LANE|SELECT_NOTE|SELECT_SONG,OFF,PULSE_MODE_OFF);
							menuSwitches.setLedByte(0,SELECT_LANE,HALF,PULSE_MODE_8TH);
							stepSwitchMode = STEPSWITCH_SELECT_LANE;
							stepSwitches.setLedAll(OFF,PULSE_MODE_OFF);
							stepSwitches.setLed(selectedLane, HALF, PULSE_MODE_8TH);
							break;
						case SELECT_NOTE:
							menuSwitches.setLedByte(0,SELECT_BLOCK|SELECT_LANE|SELECT_NOTE|SELECT_SONG,OFF,PULSE_MODE_OFF);
							menuSwitches.setLedByte(0,SELECT_NOTE,HALF,PULSE_MODE_16TH);
							stepSwitchMode = STEPSWITCH_SELECT_NOTE;
							stepSwitches.setLedAll(OFF,PULSE_MODE_OFF);
							stepSwitches.setLed(selectedStep, HALF, PULSE_MODE_16TH);
							break;
						case SELECT_SONG:
							stepSwitchMode = STEPSWITCH_SELECT_SONG;
							stepSwitches.setLedAll(OFF,PULSE_MODE_OFF);
							stepSwitches.setLed(selectedSong, HALF, PULSE_MODE_2ND);
							break;
						default:
							break;
			}
	}
	switch (stepSwitchMode){
	case STEPSWITCH_SELECT_BLOCK:
		// stepSwitches.setLedAll(OFF);
		// stepSwitches.setLed(selectedBlock, FULL, PULSE_MODE_4TH);
		if(stepSwitches.pressed()){
			selectedBlock = stepSwitches.firstChanged();
			stepSwitches.setLedAll(OFF,PULSE_MODE_OFF);
			stepSwitchMode = STEPSWITCH_EDIT_BLOCK;
		}
		break;
	case STEPSWITCH_SELECT_LANE:
		// stepSwitches.setLedAll(OFF);
		// stepSwitches.setLed(selectedBlock, FULL, PULSE_MODE_8TH);
		if(stepSwitches.pressed()){
			selectedLane = stepSwitches.firstChanged();
			stepSwitches.setLedAll(OFF,PULSE_MODE_OFF);
			stepSwitchMode = STEPSWITCH_EDIT_BLOCK;
		}

		break;
	case STEPSWITCH_SELECT_NOTE:
		stepSwitches.setLedAll(OFF,PULSE_MODE_OFF);
		stepSwitches.setLed(selectedStep, FULL, PULSE_MODE_16TH);
		if(stepSwitches.released()){
					selectedStep = stepSwitches.firstChanged();
					stepSwitches.setLedAll(OFF,PULSE_MODE_OFF);
					stepSwitchMode = STEPSWITCH_EDIT_BLOCK;
					break;
		}
		break;

	case STEPSWITCH_SELECT_SONG:
		stepSwitches.setLedAll(OFF,PULSE_MODE_OFF);
		stepSwitches.setLed(selectedSong, FULL);
		break;
	case STEPSWITCH_EDIT_BLOCK:
		for(int i=0;i < stepSwitches.numOfRegisters;i++){
			for(int j=0;j<8;j++){
				step = (i*8)+j;
				// Set note
				if(stepSwitches.returnPressed(i) & (1 << j)){
						song[selectedSong].subSteps[selectedLane][selectedBlock][step] ^= (1 << 0);
				}
				// Set leds
				if(step == song[selectedSong].activeStep[selectedLane]){
					if(song[selectedSong].subSteps[selectedLane][selectedBlock][step]){
							stepSwitches.ledPWM[step] = DIM;
					}
					else{
							stepSwitches.ledPWM[step] = HALF;
					}
				}
				else if(song[selectedSong].subSteps[selectedLane][selectedBlock][step]){
					if(step == selectedStep){
						stepSwitches.setLed(selectedStep,HALF,PULSE_MODE_16TH);
					}
					else{
						stepSwitches.setLed(step,HALF);
					}
				}
				else{
					if(step == selectedStep){
						stepSwitches.setLed(selectedStep,DIM,PULSE_MODE_OFF);
					}
					else{
						stepSwitches.setLed(step,OFF);
					}
				}
			}
		}
		break;
	default:
		break;
	}
}
// Advance to next step depending on playState
void Sequencer::subStepAdvance(Clock* clock){
	switch(transportState){
	case TRANSPORT_STATE_PLAY:
		song[playingSong].activeSubStep=clock->sub%12;
		if(clock->sub == 0){
			playingBlock = selectedBlock;
		}

		if(song[playingSong].activeSubStep == 0){
			for(int i = 0; i < 16; i++){
				if (++song[playingSong].activeStep[i] == song[playingSong].lastStep[i][playingBlock]){
					song[playingSong].activeStep[i] = 0;
				}
			}
		}
		/*
		sprintf(uartBuffer,"%c%c%c",'S',clock->sub%24,clock->sub/24);
		HAL_UART_Transmit(&huart2,(uint8_t*)uartBuffer,3,10);
		sprintf(uartBuffer,"%c%c",'M',clock->master);
		HAL_UART_Transmit(&huart2,(uint8_t*)uartBuffer,2,10);
		*/
		break;
	case TRANSPORT_STATE_RESET:
		if(clock->isLocked()){
			for(int i = 0; i < 16; i++){
				song[playingSong].activeStep[i] = 0;
			}
			song[playingSong].activeSubStep = clock->sub%24;
			transportState = TRANSPORT_STATE_PLAY;
		}

		break;
	case TRANSPORT_STATE_RESUME:
		break;
	case TRANSPORT_STATE_HOLD:
		break;
	case TRANSPORT_STATE_STOP:

		clock->unlock();
		break;
	default:
		break;
	}
}
void Sequencer::writeTrigPattern(void){
	uint8_t step;

	static uint8_t previousUartTrigger;
	char uartBuffer[3];

	if(transportState == TRANSPORT_STATE_PLAY){
		uint8_t trigger = 0;
		for(int i = 0; i < 2; i++){
			step = song[playingSong].activeStep[i];

			if((song[playingSong].subSteps[i][playingBlock][step]) & (1 <<  song[playingSong].activeSubStep)){
				HAL_GPIO_WritePin(GPIOB, (1 << i), GPIO_PIN_SET);
				trigger |= (1 << i);
			}
			else{
				HAL_GPIO_WritePin(GPIOB, (1 << i), GPIO_PIN_RESET);
			}
		}
		char uartTrigger = 0;
		for(int i = 0; i < 8; i++){
			step = song[playingSong].activeStep[i];
			if((song[playingSong].subSteps[i][playingBlock][step]) & (1 <<  song[playingSong].activeSubStep)){
				uartTrigger |= (1 << i);

			}
		}
		if(uartTrigger != previousUartTrigger){
			previousUartTrigger = uartTrigger;
			sprintf(uartBuffer,"%c%c",'T',uartTrigger);
			HAL_UART_Transmit(&huart2,(uint8_t*)uartBuffer,2,10);
		}
	}
}



Sequencer::~Sequencer(void){
}





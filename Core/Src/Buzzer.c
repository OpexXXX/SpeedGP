/*
 * Buzzer.cpp
 *
 *  Created on: Mar 20, 2020
 *      Author: 79029
 */

#include "Buzzer.h"




void BuzzerSetFreq(uint16_t freq)
{

	TIM2->PSC = (48000000  / ((BUZZER_VOLUME_MAX*2)*freq)) - 1; //prescaller
}


void BuzzerSetVolume(uint16_t volume)
{
	if(volume > BUZZER_VOLUME_MAX)
		volume = BUZZER_VOLUME_MAX;

	TIM2->CCR1 = volume;
}

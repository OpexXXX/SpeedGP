/*
 * Buzzer.h
 *
 *  Created on: Mar 20, 2020
 *      Author: 79029
 */

#ifndef INC_BUZZER_H_
#define INC_BUZZER_H_

#include "stm32f1xx_hal.h"

void BuzzerSetFreq(uint16_t freq);
void BuzzerSetVolume(uint16_t volume);

typedef struct  {
	uint16_t freq;
	uint16_t duration;
	uint16_t volume;
} buzzerStruct;

static const uint16_t GL_BuzzerAllNotes[] = {
	261, 277, 294, 311, 329, 349, 370, 392, 415, 440, 466, 494,
	523, 554, 587, 622, 659, 698, 740, 784, 831, 880, 932, 988,
	1046, 1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661, 1760, 1865, 1976,
	2093, 2217, 2349, 2489, 2637, 2794, 2960, 3136, 3322, 3520, 3729, 3951,
	4186, 4434, 4699, 4978, 5274, 5588, 5920, 6272, 6645, 7040, 7459, 7902};

#define OCTAVE_ONE_START_INDEX		(0)
#define OCTAVE_TWO_START_INDEX		(OCTAVE_ONE_START_INDEX + 12)
#define OCTAVE_THREE_START_INDEX	(OCTAVE_TWO_START_INDEX + 12)
#define OCTAVE_FOUR_START_INDEX		(OCTAVE_THREE_START_INDEX + 12)
#define OCTAVE_FIVE_START_INDEX		(OCTAVE_FOUR_START_INDEX + 12)

#define BUZZER_DEFAULT_FREQ		(4186) //C8 - 5th octave "Do"
#define BUZZER_DEFAULT_DURATION		(20) //20ms
#define BUZZER_VOLUME_MAX		(125)
#define BUZZER_VOLUME_MUTE		(0)


#endif /* INC_BUZZER_H_ */

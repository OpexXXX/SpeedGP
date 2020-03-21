/*
 * keyboard.h
 *
 *  Created on: Mar 21, 2020
 *      Author: 79029
 */

#ifndef INC_KEYBOARD_H_
#define INC_KEYBOARD_H_

#define BTN1_PIN		GPIOB
#define BTN1_PORT		GPIO_PIN_5
#define BTN2_PIN		GPIOB
#define BTN2_PORT		GPIO_PIN_4
#define BTN3_PIN		GPIOB
#define BTN3_PORT		GPIO_PIN_3



typedef enum
{
	BUTTON_RELEASED = 0,
	BUTTON_SHORT_PRESSED,
	BUTTON_LONG_PRESSED
} ButtonState;

typedef struct  {
	uint8_t buttonNumber;
	ButtonState state;
} buttonStruct;


#endif /* INC_KEYBOARD_H_ */

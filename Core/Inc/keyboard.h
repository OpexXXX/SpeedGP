/*
 * keyboard.h
 *
 *  Created on: Mar 21, 2020
 *      Author: 79029
 */

#ifndef INC_KEYBOARD_H_
#define INC_KEYBOARD_H_

#include "cmsis_os.h"
#include "main.h"
#include "stm32f1xx.h"
#include "Buzzer.h"
#include "helper.h"


#define BTN_COUNT       						3

#define BTN_LONG_PRESS_TIME_DELAY				500
#define BTN_DBL_LONG_PRESS_DELAY 				1500
#define BTN_CHECK_DELAY      					5

#define	BUTTON_LONG_PRESSED_BUZZER_FREQ   		4000
#define	BUTTON_LONG_RELEASED_BUZZER_FREQ 		4500
#define	BUTTON_SHORT_PRESSED_BUZZER_FREQ 		3000
#define BUTTON_SHORT_RELEASED_BUZZER_FREQ 		3500

#define	BUTTON_LONG_PRESSED_BUZZER_TIME         20
#define	BUTTON_LONG_RELEASED_BUZZER_TIME	20
#define	BUTTON_SHORT_PRESSED_BUZZER_TIME	5
#define BUTTON_SHORT_RELEASED_BUZZER_TIME	5

namespace Keyboard{
typedef enum
{
	BUTTON_POLL = 0,
	BUTTON_LONG_PRESSED,
	BUTTON_LONG_RELEASED,
	BUTTON_DBL_LONG_PRESSED,
	BUTTON_DBL_LONG_RELEASED,
	BUTTON_SHORT_PRESSED,
	BUTTON_SHORT_RELEASED
} ButtonState;

typedef struct  {
	uint8_t buttonNumber;
	ButtonState state;
} buttonEventStruct;


const buzzerStruct defShortPressBuzzer = {
		freq:BUTTON_SHORT_PRESSED_BUZZER_FREQ,
		duration:BUTTON_SHORT_PRESSED_BUZZER_TIME,
		volume:BUZZER_VOLUME_MAX
};
const buzzerStruct defLongPressBuzzer = {
		freq:BUTTON_LONG_PRESSED_BUZZER_FREQ,
		duration:BUTTON_LONG_PRESSED_BUZZER_TIME,
		volume:BUZZER_VOLUME_MAX
};
const buzzerStruct defLongReleeseBuzzer = {
		freq:BUTTON_LONG_RELEASED_BUZZER_FREQ,
		duration:BUTTON_LONG_RELEASED_BUZZER_TIME,
		volume:BUZZER_VOLUME_MAX
};
const buzzerStruct defShortReleeseBuzzer = {
		freq:BUTTON_SHORT_RELEASED_BUZZER_FREQ,
		duration:BUTTON_SHORT_RELEASED_BUZZER_TIME,
		volume:BUZZER_VOLUME_MAX
};


class Button {
public:
	Button(GPIO_TypeDef * btnPort,uint16_t btnPin,const char* label);
	ButtonState getState();
	buzzerStruct getBuzzer(ButtonState btnState);
	void setShortPressBuzzer(buzzerStruct buzz){
		shortPressBuzzer = buzz;
	}
	void setLongPressBuzzer(buzzerStruct buzz){
		longPressBuzzer = buzz;
	}
	void setLongReleeseBuzzer(buzzerStruct buzz){
		longReleeseBuzzer = buzz;
	}
	void setShortReleeseBuzzer(buzzerStruct buzz){
		shortReleeseBuzzer = buzz;
	}

private:
	ButtonState btnState=BUTTON_POLL;
	bool readPin();
	bool checkLongpress();
	void btnShortPressed();
	void btnShortReleased();
	void btnLongPressed();
	void btnLongReleased();
	void setDefaultBuzzer();

	uint32_t start_press_timer=0;
	GPIO_TypeDef * buttonPort;
	uint16_t buttonPin;
	const char* Label;
	buzzerStruct shortPressBuzzer;
	buzzerStruct longPressBuzzer;
	buzzerStruct longReleeseBuzzer;
	buzzerStruct shortReleeseBuzzer;
};


class Hadler
{
public:
	Button buttons[BTN_COUNT] = {
			Button(BTN_1_GPIO_Port,BTN_1_Pin,"BTN1"),
			Button(BTN_2_GPIO_Port,BTN_2_Pin,"BTN2"),
			Button(BTN_3_GPIO_Port,BTN_3_Pin,"BTN3")
	};
	Hadler(osQueue<buttonEventStruct> *  keyboardQueue,	osQueue<buzzerStruct> * buzzerQueue);
	void checkKeyboard();
private:
	osQueue<buttonEventStruct> * keyboardQueue;
	osQueue<buzzerStruct>* buzzerQueue;
	//const osMessageQueueId_t* xHandle;
};

}
#endif /* INC_KEYBOARD_H_ */

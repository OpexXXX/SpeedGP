#include  "keyboard.h"
namespace Keyboard{


void Button::setDefaultBuzzer()
{
	this->shortPressBuzzer=defShortPressBuzzer;
	this->longPressBuzzer = defLongPressBuzzer;
	this->longReleeseBuzzer= defLongReleeseBuzzer;
	this->shortReleeseBuzzer = defShortReleeseBuzzer;
}
buzzerStruct  Button::getBuzzer(ButtonState btnState)
{

	switch (btnState) {
	case BUTTON_LONG_PRESSED:
		return this->longPressBuzzer;
		break;
	case BUTTON_LONG_RELEASED:
		return this->longReleeseBuzzer;
		break;
	case BUTTON_SHORT_PRESSED:
		return this->shortPressBuzzer;
		break;
	case BUTTON_SHORT_RELEASED:
		return this->shortReleeseBuzzer;
		break;
	default:
		return this->longPressBuzzer;
		break;
	}
}

Button::Button(GPIO_TypeDef * btnPort,uint16_t btnPin,const char* label):Label(label)
{
	this->btnState==BUTTON_POLL;
	this->buttonPort = btnPort;
	this->buttonPin = btnPin;
	this->setDefaultBuzzer();
}

bool Button::readPin()
{
	bool result = HAL_GPIO_ReadPin(buttonPort, buttonPin)==GPIO_PIN_RESET;
	return result;
}
ButtonState Button::getState()
{
	bool pinState = readPin();

	if(pinState && this->btnState==BUTTON_POLL)
	{
		btnShortPressed();
		return BUTTON_SHORT_PRESSED;
	}

	if(pinState && checkLongpress() && (this->btnState==BUTTON_SHORT_PRESSED))
	{
		btnLongPressed();
		return BUTTON_LONG_PRESSED;
	}

	if(!pinState && (this->btnState==BUTTON_SHORT_PRESSED))
	{
		btnShortReleased();
		return BUTTON_SHORT_RELEASED;
	}

	if(!pinState && (this->btnState==BUTTON_LONG_PRESSED))
	{
		btnLongReleased();
		return BUTTON_LONG_RELEASED;
	}

	if(!pinState)
	{
		return BUTTON_POLL;
	}
	return BUTTON_POLL;
}

void Button::btnShortPressed()
{
	this->btnState = BUTTON_SHORT_PRESSED;
	this->start_press_timer = osKernelGetTickCount();
}
void Button::btnLongPressed()
{
	this->btnState = BUTTON_LONG_PRESSED;
}
void Button::btnShortReleased()
{
	this->btnState = BUTTON_POLL;
}
void Button::btnLongReleased()
{
	this->start_press_timer = 0 ;
	this->btnState = BUTTON_POLL;

}

bool Button::checkLongpress()
{
	if((this->start_press_timer + BTN_LONG_PRESS_TIME_DELAY)<osKernelGetTickCount())
	{
		return true;
	}
	return false;
};


Hadler::Hadler(osMessageQueueId_t *  keyQueue,	osMessageQueueId_t * buzQueue): keyboardQueue(keyQueue), buzzerQueue(buzQueue)
{

}
void Hadler::checkKeyboard()
{
	for (int i = 0; i <BTN_COUNT ; ++i)
	{
		ButtonState stat = buttons[i].getState();
		buzzerStruct buztemp;
		switch (stat) {
		case BUTTON_LONG_PRESSED:
			buztemp = buttons[i].getBuzzer(BUTTON_LONG_PRESSED);
			osMessageQueuePut(*buzzerQueue, &buztemp,0U, 0U);
			break;
		case BUTTON_LONG_RELEASED:
			buztemp = buttons[i].getBuzzer(BUTTON_LONG_RELEASED);
						osMessageQueuePut(*buzzerQueue, &buztemp,0U, 0U);

			break;
		case BUTTON_SHORT_PRESSED:
			buztemp = buttons[i].getBuzzer(BUTTON_SHORT_PRESSED);
								osMessageQueuePut(*buzzerQueue, &buztemp,0U, 0U);
			break;
		case BUTTON_SHORT_RELEASED:
			buztemp = buttons[i].getBuzzer(BUTTON_SHORT_RELEASED);
								osMessageQueuePut(*buzzerQueue, &buztemp,0U, 0U);
			break;
		default:
			break;
		}
	}

}

}

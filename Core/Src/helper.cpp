/*
 * helper.cpp
 *
 *  Created on: 1 апр. 2020 г.
 *      Author: 79029
 */
#include "helper.h"

#ifdef DEBUG_FROM_UART3

void UART_Printf(const char* fmt, ...) {
	char buff[256];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buff, sizeof(buff), fmt, args);
	HAL_UART_Transmit(&huart3, (uint8_t*)buff, strlen(buff),
			HAL_MAX_DELAY);
	va_end(args);
}
#endif



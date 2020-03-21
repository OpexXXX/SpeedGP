/*
 * fonts.h
 *
 *  Created on: 31 окт. 2018 г.
 *      Author: opex
 */

#ifndef FONTS_H_
#define FONTS_H_



#include "stm32f1xx_hal.h"

#ifndef Fonts
#define Fonts

//
//	Structure om font te definieren
//
typedef struct {
	const uint8_t FontWidth;    /*!< Font width in pixels */
	uint8_t FontHeight;   /*!< Font height in pixels */
	const uint16_t *data; /*!< Pointer to data font data array */
} FontDef;


//
//	De 3 fonts
//
extern FontDef Font_7x10;
extern FontDef Font_11x18;
extern FontDef Font_16x26;

#endif



#endif /* FONTS_H_ */

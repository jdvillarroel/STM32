/*
 * e2s_AMFS.h
 *
 *  Created on: Jan 25, 2022
 *      Author: jdvillarroel
 */

#ifndef INC_E2S_AMFS_H_
#define INC_E2S_AMFS_H_

// *********************************************************
//					INCLUDES
// *********************************************************
#include "main.h"

// *********************************************************
//					MACROS
// *********************************************************
#define	DIGIT3		(uint8_t)0x80	// Most significant digit in display
#define	DIGIT2		(uint8_t)0x40
#define	DIGIT1		(uint8_t)0x20
#define	DIGIT0		(uint8_t)0x10	// Least significant digit in display

// Define numbers in the display 7 segments
#define NUMBER0			0x03
#define NUMBER1			0x9F
#define NUMBER2			0x25
#define NUMBER3			0x0D
#define NUMBER4			0x99
#define NUMBER5			0x49
#define NUMBER6			0x41
#define NUMBER7			0x1F
#define NUMBER8			0x01
#define NUMBER9			0x19
#define DECIMAL_POINT	0xFE

#define MIN_SHIFT_DELAY		10

// *********************************************************
//					VARIABLES
// *********************************************************

// Define AMFS structure

typedef struct
{
	TIM_HandleTypeDef		*delayTimer;
	TIM_HandleTypeDef		*update7SDTimer;
	uint8_t					update7SDFlag;
	uint8_t digitIndex;

	uint8_t display[5];
	uint8_t digitsPos[4];
}AMFS_Type;

// *********************************************************
//					FUNCTION PROTOTYPES
// *********************************************************

void AMFSInit(AMFS_Type *amfs);
void writeDigit(uint8_t _value, uint8_t _position);
void update7SD(void);
void write7SDMemory(uint16_t _number);
void write7SD(uint16_t _number);
uint8_t decodeNumber(uint8_t);
static void delayTimerClockBase(uint16_t _delay);
static void shiftByteOut(uint8_t _byte);

#endif /* INC_E2S_AMFS_H_ */

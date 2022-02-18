/*
 * e2s_AMFS.c
 *
 *  Created on: Jan 25, 2022
 *      Author: jdvillarroel
 */

// *********************************************************
//					INCLUDES
// *********************************************************
#include "e2s_AMFS.h"

// *********************************************************
//					VARIABLES
// *********************************************************
const uint8_t NUMBERS[11] = {
		0x03,				//0
		0x9F,				//1
		0x25,
		0x0D,
		0x99,
		0x49,
		0x41,
		0x1F,
		0x01,
		0x19,				//9
		0xFF
};

const uint8_t DIGITS[4] = {
	0x10,					// Least significant digit - most right
	0x20,
	0x40,
	0x80
};

AMFS_Type	*pamfs;

// *********************************************************
//					EXTERN VARIABLES
// *********************************************************



/*
 * @brief	Initialize the Arduino Multi-functional Shield.
 * @param	*delayTimer: Pointer to the timer used to generate the delay
 * 			used to shift data out on the serial output.
 * @retval	None
 */
void AMFSInit(AMFS_Type *amfs)
{
	// New pointer to the AMFS struct to be used inside the library.
	pamfs = amfs;

	// Initialize timer used for delay.
	HAL_TIM_Base_Start(pamfs->delayTimer);

	// Initialize the AMFS Structure.
	pamfs->digitIndex = 0;
//	pamfs->digitPositionIndex = 0;

	pamfs->display[4] = 0;

	for (int i = 0; i < 4; i++)
	{
		pamfs->display[i] = 0xFF;
	}

	// Initialize array that holds decoded values of the digi position
	// on the 7 segment LCD display.
	pamfs->digitsPos[0] = DIGIT0;
	pamfs->digitsPos[1] = DIGIT1;
	pamfs->digitsPos[2] = DIGIT2;
	pamfs->digitsPos[3] = DIGIT3;
}

/*
 * @brief	Function generates a blocking delay based on the timer clock source
 * 			frequency. It should be used to generate very short delays.
 * @param	_delay: number of timer clock cycles to delay.
 */
static void delayTimerClockBase(uint16_t _cycles)
{
	// Reset timer count to zero.
	__HAL_TIM_SET_COUNTER(pamfs->delayTimer, 0);

	// Wait for the specified delay. It will wait the amount of cycles passed in
	// by delay. The time it represents depends on the timer clock source period.
	while(__HAL_TIM_GET_COUNTER(pamfs->delayTimer) < _cycles);
}


void writeDigit(uint8_t _value, uint8_t _position)
{
	// Set all pins at low level
	HAL_GPIO_WritePin(display_LCHCLK_GPIO_Port, display_LCHCLK_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(display_SDI_GPIO_Port, display_SDI_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(display_SFTCLK_GPIO_Port, display_SFTCLK_Pin, GPIO_PIN_RESET);

	// Shift out the digit value to be displayed on the 7 segments display.
	shiftByteOut(_value);

	// Shift out the digit _position in the display 7 segments.
	shiftByteOut(_position);

	// Save _values into latch register by setting latch clock.
	HAL_GPIO_WritePin(display_LCHCLK_GPIO_Port, display_LCHCLK_Pin, GPIO_PIN_SET);

	// Delay to secure _value.
	delayTimerClockBase(MIN_SHIFT_DELAY);

	HAL_GPIO_WritePin(display_LCHCLK_GPIO_Port, display_LCHCLK_Pin, GPIO_PIN_RESET);

}

static void shiftByteOut(uint8_t _byte)
{
	uint8_t lsb;
	uint8_t shiftCount;

	// Extract least significant bit from _value to put in the display SDI
	// Loop is repeated 8 times to shift out the whole byte.
	for (shiftCount = 0; shiftCount < 8; shiftCount++)
	{
		// Extract lsb
		lsb = _byte & 0x1;

		// Write lsb to display SDI
		HAL_GPIO_WritePin(display_SDI_GPIO_Port, display_SDI_Pin, lsb);

		// Delay to shift the _value into shift register
		delayTimerClockBase(MIN_SHIFT_DELAY);

		// Generate shift clock rise.
		HAL_GPIO_WritePin(display_SFTCLK_GPIO_Port, display_SFTCLK_Pin, GPIO_PIN_SET);

		// Shift _value for next iteration.
		_byte = _byte >> 1;

		// Delay for next iteration.
		delayTimerClockBase(MIN_SHIFT_DELAY);

		// Reset clock state.
		HAL_GPIO_WritePin(display_SFTCLK_GPIO_Port, display_SFTCLK_Pin, GPIO_PIN_RESET);
	}
}

void update7SD(void)
{
	// Indicates 7 segments display can be updated with value.
	pamfs->update7SDFlag = 1;

	// Points to next digit in the sequence.
//	pamfs->digitPositionIndex++;
//
//	// Check digit position overflow.
//	if (pamfs->digitPositionIndex >= 4)
//		pamfs->digitPositionIndex = 0;

	pamfs->digitIndex++;

	if (pamfs->digitIndex >= 4)
		pamfs->digitIndex = 0;
}

void write7SD(uint16_t _number)
{
	// Save value to 7 segment display memory.
	write7SDMemory(_number);

	// Send data to 7 segment display stored in the AMFS structure.
	writeDigit(pamfs->display[pamfs->digitIndex], pamfs->digitsPos[pamfs->digitIndex]);

	// Update the index that points to the digit in the 7 segment display memory.
//	pamfs->digitIndex++;
//
//	if (pamfs->digitIndex >= 4)
//		pamfs->digitIndex = 0;
}

void write7SDMemory(uint16_t _number)
{
	uint8_t counter = 0;

	while (_number != 0)
	{
		// Decode and write to the 7 segment memory.
		pamfs->display[counter] = decodeNumber((uint8_t)(_number % 10));

		// Points to next digit on the display.
		counter++;

		// Update number
		_number /= 10;
	}

	// update the leading digits on the 7 segments display.
	for (int i = counter; i < 4; i++)
	{
		pamfs->display[i] = NUMBER0;
	}

	// If decimal point is defines in AMFS structure. Here is added in the
	// desired position. display[4] in the AMFS structure define the position
	// of the decimal point (except for the position 0.
	if (pamfs->display[4] != 0)
	{
		uint8_t decimalPointPosition = pamfs->display[4];

		// Write 0 to the least significant bit of the register of the byte
		// that will display the decimal point.
		pamfs->display[decimalPointPosition] &= DECIMAL_POINT;
	}
}

uint8_t decodeNumber(uint8_t _number)
{
	uint8_t digit;

	switch(_number)
	{
	case 0:
		digit = NUMBER0;
		break;
	case 1:
		digit = NUMBER1;
		break;
	case 2:
		digit = NUMBER2;
		break;
	case 3:
		digit = NUMBER3;
		break;
	case 4:
		digit = NUMBER4;
		break;
	case 5:
		digit = NUMBER5;
		break;
	case 6:
		digit = NUMBER6;
		break;
	case 7:
		digit = NUMBER7;
		break;
	case 8:
		digit = NUMBER8;
		break;
	case 9:
		digit = NUMBER9;
		break;
	default:
		digit = 0xFF;
		break;
	}

	return digit;
}

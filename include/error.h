///////////////////////////////////////////////////////
// error.h
//
// servo

#pragma once // Include this file only once

#include "..\\..\\common_controller\\include\\c99types.h"

#define ERROR_NONE			0		// no error
#define ERROR_ADC			1		// adc out of range
#define ERROR_BUF_OVFL		2		// RSS232 input buffer overflow
#define ERROR_CRC			4		// RS232 CRC error
#define ERROR_COMMAND		8		// unrecognized command from RS232
#define ERROR_CHANNEL		16		// invalid device channel
#define ERROR_DATALOG		32		// datalogging interval out of range (0..255)

#define ERROR_POS			64		// Invalid position commanded

extern volatile uint16_t Error;

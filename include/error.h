///////////////////////////////////////////////////////
// error.h
//
// servo

#pragma once // Include this file only once

#include "..\\..\\common_controller\\include\\c99types.h"

#define ERROR_NONE			0		// No error
#define ERROR_POS			1		// Invalid position commanded
#define ERROR_BUF_OVFL		2		// RS232 input buffer overflow
#define ERROR_CRC			4		// RS232 CRC error
#define ERROR_COMMAND		8		// unrecognized command from RS232
#define ERROR_DATALOG		16		// Datalogging interval out of range (0..255)

extern volatile uint16_t Error;

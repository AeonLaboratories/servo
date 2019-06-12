///////////////////////////////////////////////////////
// config.h
//
// servo

#pragma once // Include this file only once

#include <ez8.h>
#include <uart.h>
#include "..\\..\\common_controller\\include\\z082a.h"

#define SYS_FREQ				5529600		// system clock == __user_frequency
#define TIMER_MAX				65535		// the timer registers have 16 bits

///////////////////////////////////////////////////////
// To avoid rounding/truncation headaches, choose
// timer frequencies that are factors of the system
// clock frequency. Best performance is achieved
// when they are also powers of two.
//
// The factors of 5529600:
///////////////////////////////////////////////////////
// 1, 2, 3, 4, 5, 6, 8, 9, 10, 12, 15, 16, 
// 18, 20, 24, 25, 27, 30, 32, 36, 40, 45, 
// 48, 50, 54, 60, 64, 72, 75, 80, 90, 96, 
// 100, 108, 120, 128, 135, 144, 150, 160, 
// 180, 192, 200, 216, 225, 240, 256, 270, 
// 288, 300, 320, 360, 384, 400, 432, 450, 
// 480, 512, 540, 576, 600, 640, 675, 720, 
// 768, 800, 864, 900, 960, 1024, 1080, 1152, 
// 1200, 1280, 1350, 1440, 1536, 1600, 1728, 
// 1800, 1920, 2048, 2160, 2304, 2400, 2560, 
// 2700, 2880, 3072, 3200, 3456, 3600, 3840, 
// 4096, 4320, 4608, 4800, 5120, 5400, 5760, 
// 6144, 6400, 6912, 7200, 7680, 8192, 8640, 
// 9216, 9600, 10240, 10800, 11520, 12288, 
// 12800, 13824, 14400, 15360, 17280, 18432, 
// 19200, 20480, 21600, 23040, 24576, 25600, 
// 27648, 28800, 30720, 34560, 36864, 38400, 
// 40960, 43200, 46080, 51200, 55296, 57600, 
// 61440, 69120, 73728, 76800, 86400, 92160, 
// 102400, 110592, 115200, 122880, 138240, 
// 153600, 172800, 184320, 204800, 221184, 
// 230400, 276480, 307200, 345600, 368640, 
// 460800, 552960, 614400, 691200, 921600, 
// 1105920, 1382400, 1843200, 2764800, 5529600
///////////////////////////////////////////////////////

#define CLOCKWISE				TRUE
#define ANTICLOCKWISE			FALSE

///////////////////////////////////////////////////////
// Switch debounce code should check the switch state
// eight times (for 1 byte of history bits) in a period just
// slightly longer than the worst-case switch bounce period,
// which is usually ~1..4 ms, but with some devices may be up
// to 7 or 8 ms.
//
// The rotary encoder specs the sliding noise (bounce/chatter)
// at <3 ms at a turn rate of 360 deg / sec. A fast servo turns
// the output shaft 60 degrees in 0.1 seconds, or ~600 deg / sec.
// A slow servo might only manage 300 deg / sec. Most are in the
// 450 deg / sec range. At 600 deg / sec, I'm guessing the 
// rotary encoder transition noise will be shorter than 3 ms.
//
// The motor of a traditional (analog) hobby servo is
// pulsed at the command pulse rate, typically 50 Hz. The
// motor of a digital servo is usually pulsed at 300..400
// Hz. In other applications, motors are driven at ultrasonic
// frequencies (15..40kHz) with values 18..25kHz more common.
///////////////////////////////////////////////////////

// Frequencies of services periodically initiated by isr_timer0()
#define CO_FREQ					512			// control output pulse frequency
#define CU_FREQ					128			// controller update frequency
#define SERVICE_FREQ			2048		// switch debounce frequency			
// SERVICE_FREQ is for things other than CO and 
// CU (like sensors, e.g.). Set it to 1 if it's not needed.

// To optimize overall performance, T0's frequency should
// be as low as possible, just high enough to adequately 
// provide the most frequently required service.
//
// Set T0_FREQ to the least common integer multiple of 
// 		CO_FREQ, CU_FREQ, and SERVICE_FREQ
#define T0_FREQ					2048

// Manually calculate and enter the following values. They 
// must be integers. Later conditional expressions will 
// fail in the preprocessor if any non-integer values are 
// present in the computation, even if the resultant 
// evaluation turns out to be an integer.
//
#define CO_PERIOD				4			// (T0_FREQ / CO_FREQ)
#define CU_PERIOD				16			// (T0_FREQ / CU_FREQ)
#define SERVICE_PERIOD			1			// (T0_FREQ / SERVICE_FREQ)


///////////////////////////////////////////////////////
// Several performance optimizations are possible if certain
// values are integral factors of others and/or are
// powers of 2. However, constructs that depend on these 
// properties can otherwise break the code.
//
// Preprocessor conditional expressions cannot contain 
// non-integral values. Consequently, there is apparently 
// no way to detect whether an expression *is* non-integer;
// the preprocessor simply dies with the error
// "Illegal constant expression."
//
// Therefore, the logic intended by the indicated 
// expressions below must be explicitly programmed 
// case-by-case, based on manually-computed values 
// derived from SYS_FREQ, SERVICE_FREQ, CO_FREQ, and CU_FREQ.
//
///////////////////////////////////////////////////////
// Set these values to 1 (true) if the pseudo-coded 
// conditions are met, othewise, 0 (false):
//
/*
#define T0_DIVIDES_SYS						isNonZeroInteger(SYS_FREQ/T0_FREQ)
#define T0_FREQ_IPOT						isPowerOfTwo(T0_FREQ)
#define CO_PERIOD_IPOT						isPowerOfTwo(CO_PERIOD)
#define CU_PERIOD_IPOT						isPowerOfTwo(CU_PERIOD)
#define SERVICE_PERIOD_IPOT					isPowerOfTwo(SERVICE_PERIOD)
*/
#define T0_DIVIDES_SYS						1
#define T0_FREQ_IPOT						1
#define CO_PERIOD_IPOT						1
#define CU_PERIOD_IPOT						1
#define SERVICE_PERIOD_IPOT					1


///////////////////////////////////////////////////////
// Timer 0
//
//		To optimize counter resolution, clock T0 at the
// highest possible frequency, i.e., use the smallest 
// possible prescale.
//
// TO_PRESCALE must a power of 2 in the range [1..128]
//
// Manually calculate and enter the following value.
// 
// #define T0_PRESCALE = 	(LOWEST POWER OF 2 >= SYS_FREQ / TIMER_MAX / T0_FREQ)
//
#define T0_PRESCALE				1		// >= (5529600 / 65535 / 2048) ~= 0.04


///////////////////////////////////////////////////////
// Timer 1
//
// This timer is used to turn off the control output
// pulse after a duration calculated in the timer 0
// interrupt service routine (which turns the 
// power on).
//
// T1_PRESCALE must a power of 2 in the range [1..128]
//
// To ensure the possibility of a 100% duty
// cycle, set the T1 prescale to the smallest power 
// of 2 that is greater than or equal to the following
// expression.
//
// Manually calculate and enter the following value.
// 
// #define T1_PRESCALE = 	(LOWEST POWER OF 2 >= SYS_FREQ / TIMER_MAX / T0_FREQ * CO_PERIOD)
//
#define T1_PRESCALE				1		// >= (5529600 / 65535 / 2048 * 4) ~= 0.16
//
// For maximum pulse-duration precision, set T1's 
// prescale to 1. The maximum CO pulse width and
// duty cycle will then be limited by the timer
// register size.
//
//#define T1_PRESCALE				1
//
//
///////////////////////////////////////////////////////


///////////////////////////////////////////////////////
// UART/RS232/RX/TX/serial communications speed
#define SYS_BAUD				BAUD_115200

///////////////////////////////////////////////////////
// serial data received and transmission buffers
// NOTE: buffer sizes must be a power of 2
// TXB_SIZE can be set to 0 to disable transmit buffering in order
// to conserve EDATA space at a cost in performance.
#define RXB_SIZE				32		
#define TXB_SIZE				128

///////////////////////////////////////////////////////
// ADC configuration
#define ADC_CTL0_INIT				(ADC_CONT | ADC_CEN)	// continuous mode
//#undef ADC_CTL0_INIT										// one-shot mode



///////////////////////////////////////////////////////
// Implementation-specific IRQ priorities
#define EI_T0()					IRQ0_PRIORITY_HIGH(IRQ_T0);
#define EI_RX()					IRQ0_PRIORITY_LOW(IRQ_U0R)
#define EI_TX()					IRQ0_PRIORITY_LOW(IRQ_U0T)
#define EI_ADC()				IRQ0_PRIORITY_LOW(IRQ_ADC);
#define EI_T1()					IRQ0_PRIORITY_NOMINAL(IRQ_T1);
#define EI_PC3()				IRQ2_PRIORITY_NOMINAL(IRQ_PC3);

#define CMD_IVECT				PC3_IVECT
#define IRQ_CMD					IRQ_PC3
#define DI_CMD()				DI_PC3()
#define EI_CMD()				EI_PC3()
#define IRQ_CLEAR_CMD()			IRQ_CLEAR_PC3()
#define IS_EI_CMD()				IS_EI_PC3()

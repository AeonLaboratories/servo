///////////////////////////////////////////////////////
// irq.c
//
// servo
//

#include <eZ8.h>
#include <defines.h>
#include "..\\..\\common_controller\\include\\c99types.h"
#include "..\\..\\common_controller\\include\\z082a.h"
#include "..\\..\\common_controller\\include\\timer.h"
#include "..\\..\\common_controller\\include\\uart.h"
#include "..\\..\\common_controller\\include\\irq.h"
#include "config.h"
#include "error.h"
#include "gpio.h"

// Store big strings in ROM to conserve RData and EData space.
rom char FIRMWARE[]	= R"Aeon Laboratories Servo ";
rom char VERSION[]	= R"V.20171219-0000";

// configuration options
#define SERVO_DIRECTION			CLOCKWISE 	// wider command pulse => clockwise
#define INCREMENTAL				TRUE		// commands are relative to current position

// which way does {+V to M+} drive the gearmotor output spline? (this is hardware-dependent)
#define COP_DIRECTION			ANTICLOCKWISE
#define ROTATION				(SERVO_DIRECTION == COP_DIRECTION)

#define ENCODER_PPR				24									// rotary encoder pulses per revolution
#define ENCODER_TRANSITIONS		4									// pulse edges per quadrature pulse
#define POS_RESOLUTION			(ENCODER_PPR * ENCODER_TRANSITIONS)	// encoder transitions per revolution
#define POS_MIN					-32767
#define POS_MAX					32767

// The CO_MAX_RESERVE provides time for the "stop pulse"
// interrupt service routine, plus the time at the start
// of the T0 ISR before the interrupts are re-enabled.
// Making CO_MAX_RESERVE long enough prevents contention between
// the start and stop routines, at the cost of limiting the maximum
// duty cycle.
// If the T1 ISR is minimal, and T0 starts the pulse and re-enables
// interrupts as early as possible, CO_MAX_RESERVE might need to be
// as much as 150 system clock cycles, and perhaps longer if 
// another ISR preempts or delays the T1 ISR.
// 
// The following reserves a conservative 256 system clock cycles.
// Using a power of 2 guarantees that the expression always 
// evaluates to an integer. The (T1_CLOCK_FREQ / SYS_FREQ) factor 
// converts system clocks into T1 clocks.
//
//#define CO_MAX_RESERVE			(256 * T1_CLOCK_FREQ / SYS_FREQ)
//
#define CO_MAX_RESERVE			150		// T1 clocks, for minimum reserve,
										// assuming T1_CLOCK_FREQ == SYS_FREQ

#define CO_MAX					(CO_PERIOD * T1_CLOCK_FREQ / T0_FREQ - CO_MAX_RESERVE)
#if CO_MAX > TIMER_MAX
	#undef CO_MAX
	#define CO_MAX				TIMER_MAX
#endif

#define ACTIVE_POS_RANGE		11		// beyond this position error, CO is at max
	// Adjust ACTIVE_POS_RANGE and CO_MIN to govern
	// the servo speed and force near the target position
	// while avoiding overshoot.

// If CO is less than CO_MIN_RESERVE, then the T1 isr can't turn it 
// off on schedule. Empirical timing tests found a minimum pulse 
// width of about 4 microseconds, which worked out to about 22 system
// clocks at a clock frequency of 5529600 Hz.
// ~4 us / 0.18 us / T1_CLOCK ~= 22
// As with CO_MAX_RESERVE, use a power of 2 and convert to T1 clocks.
//
#define CO_MIN_RESERVE			(32 * T1_CLOCK_FREQ / SYS_FREQ)
//

// A lower CO_MIN allows a tighter ACTIVE_POS_RANGE; a higher
// CO_MIN produces greater speed/torque for small errors.
// The test servo started moving (with no load) at around 1000.
#define CO_MIN					2500
	
#if CO_MIN < CO_MIN_RESERVE
	#undef CO_MIN
	#define CO_MIN				CO_MIN_RESERVE
#endif

#define CO_RANGE				(CO_MAX - CO_MIN)
#define CO_GAIN					(CO_RANGE / ACTIVE_POS_RANGE)		// T1 clocks / position units

typedef struct
{
	unsigned int clocks;
	unsigned int ticks;
} T0Count;

///////////////////////////////////////////////////////
//
// global variables
//

volatile unsigned int T0Ticks;			// rolls over when max unsigned int is reached
volatile T0Count SaveT0;
volatile T0Count CpUpT0;
volatile T0Count CpDnT0;

volatile BOOL REncA;
volatile BOOL REncB;
volatile BYTE REncAHistory;
volatile BYTE REncBHistory;

volatile int Pos;						// in rotary encoder transitions; at 96 edges per rev, +/-341 turns fit in an int

volatile BOOL CpUp;
volatile BOOL CpDn;

BOOL CmdReceived = FALSE;

int Cmd;
int Sp;									// setpoint; the commanded position (target Pos value)
int Co;									// scratchpad for the next CO value
int CO;
reentrant void (*do_CO)();				// a pointer to the function that produces the CO signal

BOOL EnableControllerUpdate = TRUE;

BOOL EnableDatalogging;
uint8_t DatalogCount;					// counter for Datalogging
uint8_t DatalogReset = 0xFF;			// report every (this many + 1) seconds
	// Note: 0xFF is used as a disable value (DatalogReset is unsigned)
	// This is convenient because the reset value needs to be one 
	// less than the desired count.


//////////////////////////////////////////////////////
//
// prototypes
//
///////////////////////////////////////////////////////
reentrant void disable_CO();			// prototype needed by init_irq()
void isr_timer0();
void isr_timer1();
void isr_cmd();


void init_switch(BOOL isOn, BYTE *history, BYTE *state)
{
	*state = isOn;
	*history = isOn ? 0xFF : 0x00;
}

///////////////////////////////////////////////////////
// set defaults
void init_irq()
{
	init_switch(RENC_A, &REncAHistory, &REncA);
	init_switch(RENC_B, &REncBHistory, &REncB);
	
	SET_VECTOR(TIMER0, isr_timer0);
	EI_T0();

	SET_VECTOR(TIMER1, isr_timer1);
	EI_T1();

	do_CO = disable_CO;
	CO_disable();
}



///////////////////////////////////////////////////////
void preset() { }


///////////////////////////////////////////////////////
BOOL uint8CounterReset(uint8_t *count, uint8_t reset)
{
	if (*count == reset)
	{
		*count = 0;
		return TRUE;
	}
	++(*count);
	return FALSE;
}


///////////////////////////////////////////////////////
BOOL uint16CounterReset(uint16_t *count, uint16_t reset)
{
	if (*count == reset)
	{
		*count = 0;
		return TRUE;
	}
	++(*count);
	return FALSE;
}


///////////////////////////////////////////////////////
reentrant void disable_CO()
{
	CO_disable();					// this might be dangerous if the motor is spinning
}

reentrant void positive_CO()
{
	CO_off();	
	set_timer1_mark(CO);			// set the stop time
	COP_on();						// start the pulse
	start_timer1();					// timer1 ISR stops the pulse
}

reentrant void negative_CO()
{
	CO_off();
	set_timer1_mark(CO);			// set the stop time
	CON_on();						// start the pulse
	start_timer1();					// timer1 ISR stops the pulse
}

reentrant void zero_CO()
{
	CO_off();
}


///////////////////////////////////////////////////////
// This takes very close to 5 microseconds (5 MHz CPU).
// Interrupts should be disabled before calling this
// function, to avoid a clock/tick inconsistency.
void saveT0(T0Count *save)
{
	save->clocks = T0;
	save->ticks = T0Ticks;
}

void copyT0Count(T0Count *from, T0Count *to)
{
	to->clocks = from->clocks;
	to->ticks = from->ticks;
}

///////////////////////////////////////////////////////
uint16_t clocksBetween(T0Count *before, T0Count *after)
{
	uint16_t eclocks;
	uint16_t eticks = after->ticks - before->ticks;
	
	if (before->clocks > after->clocks)
	{
		--eticks;			// borrow
		eclocks = CLOCKS_PER_TICK - before->clocks + after->clocks;
	}
	else
		eclocks = after->clocks - before->clocks;

	return CLOCKS_PER_TICK * eticks + eclocks;
}


///////////////////////////////////////////////////////
// This function takes ~500-600 us
// (the multiplication is floating point)
// Since T0_CLOCK_PERIOD_US is constant, the
// multiplication could be eliminated by adjusting
// the code that uses this function to use clocks
// instead of microseconds.
int microsecondsBetween(T0Count *before, T0Count *after)
{
	return (int) (T0_CLOCK_PERIOD_US * clocksBetween(before, after));
}


///////////////////////////////////////////////////////
void update_device()
{
	int error;
	BOOL neg;
	
	reentrant void (*f)() = zero_CO;
	Co = 0;

	if (CmdReceived)
	{
		Pos = 0;
		Sp = Cmd;
		CmdReceived = FALSE;
	}
	
	if (error = Sp - Pos)
	{
		if (neg = error < 0) error = -error;
		if (error > ACTIVE_POS_RANGE)
			Co = CO_MAX;
		else
			Co = error * CO_GAIN + CO_MIN;
		if (neg == ROTATION)
			f = negative_CO;
		else
			f = positive_CO;
	}
	
	DI();
	CO = Co;
	do_CO = f;
	EI();
}


///////////////////////////////////////////////////////
void update_controller()
{	
	if (EnableControllerUpdate)
	{
		EnableControllerUpdate = FALSE;	// re-enabled later by isr_timer0
		
		update_device();
	}
}


///////////////////////////////////////////////////////
void report_header()
{
	printromstr(R"--Cmd --Pos ---CO Error");
	endMessage();
}


////////////////////////////////////////////////////////
void report_device()
{
	printi(Sp, 5, ' ');
	printi(Pos, 6, ' ');
	printi(CO, 6, ' ');
	printi(Error, 6, ' ');	
	endMessage();
}


///////////////////////////////////////////////////////
void do_commands()
{
	char c;

	while (!RxbEmpty())				// process a command
	{
		c = getc();		
		mask_clr(Error, ERROR_COMMAND);
		
		// single-byte commands
		if (c == '\0')				// null command
		{
			// (treat as single-byte command that does nothing)
		}
		else if (c == 'z')			// program data
		{
			printromstr(FIRMWARE); printromstr(VERSION); endMessage();
			endMessage();
		}
		else if (c == 'h')			// report header
		{
			report_header();
		}
		else if (c == 's' || c == '0')			// stop
		{
			Cmd = 0;
			CmdReceived = TRUE;
		}
		else						// multi-byte command
		{			
			getArgs();
			if (c == 'r')			// report
			{
				if (argPresent())			// set Datalogging interval
				{
					// DatalogReset rolls under to 0xFF (meaning "disable") if command arg is 0
					DatalogReset = tryArg(0, 255, ERROR_DATALOG, DatalogReset + 1, FALSE) - 1;
					DatalogCount = 0;
				}
				else						// one-time report
					report_device();
			}
			else if (c == 'g')				// move the given number of position units
			{
				if (argPresent())
					Cmd = tryArg(POS_MIN, POS_MAX, ERROR_POS, Cmd, FALSE);
				CmdReceived = TRUE;
			}
			else					// unrecognized command
			{
				mask_set(Error, ERROR_COMMAND);
			}
		}
	}
	
	if (EnableDatalogging)
	{
		EnableDatalogging = FALSE;	// re-enabled later by isr_timer0
		if (DatalogReset != 0xFF && uint8CounterReset(&DatalogCount, DatalogReset))
			report_device();
	}

}


///////////////////////////////////////////////////////
BOOL debounce(BYTE contact, BYTE *history, BYTE *state)
{
	*history = (*history << 1) | (contact & 0x01);
	if (*history == 0x01)
		*state = TRUE;
	else if (*history == 0x80)
		*state = FALSE;
	else
		return FALSE;	// state didn't change
	return TRUE;		// state changed
}


///////////////////////////////////////////////////////
void debounce_switches()
{
	if (debounce(RENC_A, &REncAHistory, &REncA))
	{
		if ((REncA == REncB) == ROTATION)
			--Pos;
		else
			++Pos;
	}
	
	if (debounce(RENC_B, &REncBHistory, &REncB))
	{
		if ((REncA == REncB) == ROTATION)
			++Pos;
		else
			--Pos;
	}	
}


///////////////////////////////////////////////////////
// 
#pragma interrupt
void interrupt isr_timer0()
{
	++T0Ticks;
	
	#if CO_PERIOD > 1
		if (CO_INTERVAL)
	#endif
		 {
			stop_timer1();
			IRQ_CLEAR_T1();
			EI();
			do_CO();
		 }
	#if CO_PERIOD > 1
		 else
			EI(); 
	#endif
	
	#if SERVICE_PERIOD > 1
		if (SERVICE_INTERVAL)
	#endif
			debounce_switches();

	#if CU_PERIOD > 1
		if (CU_INTERVAL)
	#endif
			EnableControllerUpdate = TRUE;
				
//	if (ONE_SECOND)
	if ((T0Ticks & 255) == 0)		// Data logging interval units are 1/8 of second
		EnableDatalogging = TRUE;
	
}


///////////////////////////////////////////////////////
// stop CO pulse
#pragma interrupt
void isr_timer1()
{
	CO_off();
}


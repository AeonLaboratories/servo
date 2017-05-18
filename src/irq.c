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
rom char VERSION[]	= R"V.20170407-0000";


// configuration options
#define SERNO					0
#define SERVO_DIRECTION			CLOCKWISE 	// wider command pulse => clockwise
#define INCREMENTAL				TRUE		// treat current position as zero whenever command pulses are absent
#define REVOLUTIONS				2.0			// the range of motion between CPW_MIN and CPW_MAX positions
											// use 2.0 for 1 full turn in either direction

// which way does {+V to M+} drive the gearmotor output spline? (this is hardware-dependent)
#define COP_DIRECTION			ANTICLOCKWISE
#define ROTATION				(SERVO_DIRECTION == COP_DIRECTION)

// CPW is control pulse width, in microseconds
// The nominal CP period of 20000 microseconds establishes
// a hard limit for CPW_MAX.
#define CPW_UPPER_LIMIT			20000
#define CPW_MIN					300									// the CPW range that maps to the servo
#define CPW_MAX					2700								// range of motion (REVOLUTIONS)
#define CPW_CTR					((CPW_MAX + CPW_MIN)/2)
#define CPW_RANGE				(CPW_MAX - CPW_MIN)

#define ENCODER_PPR				24									// rotary encoder pulses per revolution
#define ENCODER_TRANSITIONS		4									// pulse edges per quadrature pulse
#define POS_RESOLUTION			(ENCODER_PPR * ENCODER_TRANSITIONS)	// encoder transitions per revolution
#define POS_RANGE				(POS_RESOLUTION * REVOLUTIONS)		//

#define POS_PER_CPW				(POS_RANGE / CPW_RANGE)				// encoder transitions per microsecond of control pulse
#define POS_PER_CPW_X2			(POS_PER_CPW * 2)					// used for rounding

#define CMD_OFF					-1									// command pulse is off
#define CMD_COMPLETE			0									// last received command was successful
#define CMD_MIN					(CPW_MIN * POS_PER_CPW)
#define CMD_MAX					(CPW_MAX * POS_PER_CPW)
#define CMD_CTR					(CPW_CTR * POS_PER_CPW)

#define MICROSECONDS_PER_DEGREE	(CPW_RANGE / REVOLUTIONS / 360.0)


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
#define CO_MIN					1500
	
#if CO_MIN < CO_MIN_RESERVE
	#undef CO_MIN
	#define CO_MIN				CO_MIN_RESERVE
#endif

#define CO_RANGE				(CO_MAX - CO_MIN)
#define CO_GAIN					(CO_RANGE / ACTIVE_POS_RANGE)		// T1 clocks / position units

// The nominal command pulse frequency is 50 Hz, so
// whenever the command pulse is active, the servo should
// see a new command pulse about every (CU_FREQ/50)
// cycles. With CU_FREQ == 128, a pulse should arrive 
// every second or third update_controller() cycle.
#define CP_MAX_MISSES			8		// this many absent control pulses ==> CP is off


typedef struct
{
	unsigned int clocks;
	unsigned int ticks;
} T0Count;

// Position can be controlled by CMD CPW (Auto) or serial port (Manual)
typedef char enum { Auto = 0, Manual } Modes;	


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
volatile int Pos;						// in rotary encoder transitions; at 96 edges per rev, +/-341 turns fit in an int

volatile BOOL CpUp;
volatile BOOL CpDn;
int Cpw;								// command pulse width, calculated from CpDnT0 and CpUpT0

int Cmd = CMD_OFF;						// usually Cpw, converted to Pos units

Modes Mode;
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


///////////////////////////////////////////////////////
// set defaults
void init_irq()
{	
	SET_VECTOR(TIMER0, isr_timer0);
	EI_T0();

	SET_VECTOR(TIMER1, isr_timer1);
	EI_T1();

	SET_VECTOR(CMD_IVECT, isr_cmd);
	EI_CMD();
	
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
void start_cpm()
{
	DI_CMD();
	CpUp = CpDn = FALSE;
	IRQ_CLEAR_CMD();
	EI_CMD();	// restart command pulse monitor
}


///////////////////////////////////////////////////////
BOOL check_cmd()
{
	static int prevCmd;
	static uint8_t repeatsNeeded = 1;
	int cmd;
	
	if (Mode == Manual)
		return FALSE;				// ignore command pulses

	if (!CpUp || !CpDn)
		return FALSE;				// no command pulse has been received
	
	if (Cmd != CMD_COMPLETE)
	{
		cmd = microsecondsBetween(&CpUpT0, &CpDnT0);		
		if (cmd < CPW_MIN)		cmd = CPW_MIN;
		else if (cmd > CPW_MAX)	cmd = CPW_MAX;
		Cpw = cmd;
		
		// round to position units: cmd = cmd * POS_PER_CPW + 0.5;
		cmd *= POS_PER_CPW_X2;
		cmd++;
		cmd >>= 1;	// divide by 2
		
		if (cmd != Cmd)			// if this pulse differs from the current command value
		{
			if (cmd == prevCmd)		// but it matches the previous pulse
			{
				--repeatsNeeded;
				if (!repeatsNeeded)
					Cmd = cmd;
			}
			else
				repeatsNeeded = Cmd == CMD_OFF ? 1 : 4;
		}
		prevCmd = cmd;
	}

	start_cpm();

	return TRUE;					// a command pulse was received
}


///////////////////////////////////////////////////////
void update_device()
{
	int error;
	BOOL neg;
	
	reentrant void (*f)() = zero_CO;
	Co = 0;	
	
	#if INCREMENTAL
		if (Cmd == CMD_OFF) Pos = 0;
			else
	#endif		
	
	if (Cmd > CMD_COMPLETE)
	{
		Sp = Cmd - CMD_CTR;
		error = Sp - Pos;
		if (error == 0)		// doesn't account for possible overshoot
		{
			#if INCREMENTAL
				if (Mode == Auto)
					Cmd = CMD_COMPLETE;	// ignore further pulses until they stop
				else
					Cmd = CMD_OFF;
			#endif		
		}
		else			
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
	}
	
	DI();
	CO = Co;
	do_CO = f;
	EI();
}


///////////////////////////////////////////////////////
void update_controller()
{
	static uint8_t nUpdates;	// CO updates since last command pulse monitor restart
	
	if (check_cmd()) nUpdates = 0;
	
	if (EnableControllerUpdate)
	{
		EnableControllerUpdate = FALSE;	// re-enabled later by isr_timer0
		
		if (uint8CounterReset(&nUpdates, CP_MAX_MISSES))
		{
			Cmd = CMD_OFF;
			start_cpm();			// restart cp monitor, in case an IRQ was missed
		}
		update_device();
	}
}


///////////////////////////////////////////////////////
void report_header()
{
	printromstr(R"--Cpw --Cmd ---Sp --Pos ---CO Error");
	endMessage();
}


////////////////////////////////////////////////////////
void report_device()
{
	printi(Cpw, 5, ' '); printSpace();
	printi(Cmd, 5, ' '); printSpace();
	printi(Sp, 5, ' '); printSpace();
	printi(Pos, 5, ' '); printSpace();
	printi(CO, 5, ' '); printSpace();
	printi(Error, 5, ' ');	
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
			printromstr(R"S/N:"); printi(SERNO, 4, ' '); endMessage();
			printromstr(R"CPW_MIN:"); printi(CPW_MIN, 4, ' ');
			printromstr(R"   CPW_MAX:"); printi(CPW_MAX, 6, ' ');
			printromstr(R"   uS/deg:"); printdec(100*MICROSECONDS_PER_DEGREE, 6, ' ', 2); endMessage();

			printromstr(R"CMD_MIN:"); printi(CMD_MIN, 4, ' ');
			printromstr(R"   CMD_MAX:"); printi(CMD_MAX, 6, ' '); endMessage();
			printromstr(R"CO_MIN: "); printi(CO_MIN, 4, ' ');
			printromstr(R"   CO_MAX: "); printi(CO_MAX, 6, ' '); endMessage();
			endMessage();
		}
		else if (c == 'h')			// report header
		{
			report_header();
		}
		else if (c == 'a')			// auto mode
		{
			Mode = Auto;
		}
		else if (c == 's')			// stop
		{
			Mode = Manual;
			Cmd = CMD_OFF;
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
			else if (c == 'g')				// set control output
			{
				Mode = Manual;
				if (argPresent())
					Cmd = tryArg(0, CMD_MAX, ERROR_POS, Cmd, FALSE);
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
	static BYTE rencahistory;
	static BYTE rencbhistory;

	if (debounce(RENC_A, &rencahistory, &REncA))
	{
		if ((REncA == REncB) == ROTATION)
			--Pos;
		else
			++Pos;
	}
	
	if (debounce(RENC_B, &rencbhistory, &REncB))
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


///////////////////////////////////////////////////////
// CMD pin (which receives control pulses) changed state
#pragma interrupt
void isr_cmd()
{
	saveT0(&SaveT0);
	if (CMD_is_high() && !CpUp)
	{
		copyT0Count(&SaveT0, &CpUpT0);
		CpUp = TRUE;
	}
	else if (CpUp && !CpDn)
	{
		copyT0Count(&SaveT0, &CpDnT0);
		CpDn = TRUE;
		DI_CMD();
	}
}

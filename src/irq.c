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
rom char VERSION[]	= R"V.20220823-0000";

// configuration options
#define SERVO_DIRECTION			CLOCKWISE 	// wider command pulse => clockwise
//#define INCREMENTAL				TRUE		// commands are relative to current position (FALSE not implemented)

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

// If CO is less than CO_MIN_RESERVE, then the T1 isr can't turn it 
// off on schedule. Empirical timing tests found a minimum pulse 
// width of about 4 microseconds, which worked out to about 22 system
// clocks at a clock frequency of 5529600 Hz.
// ~4 us / 0.18 us / T1_CLOCK ~= 22
// As with CO_MAX_RESERVE, use a power of 2 and convert to T1 clocks.
//
#define CO_MIN_RESERVE			(32 * T1_CLOCK_FREQ / SYS_FREQ)
//

// A test servo started moving (with no load) at around 1000.
#define CO_MIN					2000
	
#if CO_MIN < CO_MIN_RESERVE
	#undef CO_MIN
	#define CO_MIN				CO_MIN_RESERVE
#endif

#define CO_RANGE				(CO_MAX - CO_MIN)

// When running at MAX_CO, setting Co to 0 stops within this many positions (or fewer).
// The minimum error (Sp - Pos) that should produce maximum CO.
#define ERROR_FOR_MAX_CO		7		// set CO to MAX if error is greater than this
#define CO_GAIN					(CO_RANGE / ERROR_FOR_MAX_CO)		// T1 clocks / position units (must be positive)
#define CO_SAFE					7000	// below this CO, even a stalled servo draws < 2 amps

#define INTERVAL_NORMAL			6		// 80 deg / sec or slower; assume until data available
#define INTERVAL_SLOW			30		// 16 deg / second or slower		
#define INTERVAL_STALLED		192		// 2.5 deg / sec or slower

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
volatile int PriorPos;

// Interval tracks how many updates occurred between the two most recent
// movement detections (position changes). Because control updates occur 
// regularly, Interval is directly related to the servo speed. When Interval 
// is 6, speed is 80 deg/sec or slightly slower; at 30, speed <= 16 deg/sec, 
// and at 192, speed is <= 2.5 deg/sec (effectively stalled).
volatile int IntervalCounter;
volatile int Interval;

volatile BOOL CpUp;
volatile BOOL CpDn;

// for proportional-integral control
int PriorError;
float Integral;

BOOL Manual;							// CO is controlled by command
int Sp;									// setpoint; the commanded position (target Pos value)
float Co;								// scratchpad for the next CO value
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
	
	do_CO = disable_CO;
	CO_disable();

	SET_VECTOR(TIMER0, isr_timer0);
	SET_VECTOR(TIMER1, isr_timer1);

	EI_T0();
	EI_T1();
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
//int microsecondsBetween(T0Count *before, T0Count *after)
//{
//	return (int) (T0_CLOCK_PERIOD_US * clocksBetween(before, after));
//}


///////////////////////////////////////////////////////
// For each motion, set \Integral \to 0, 
// before the first call to this function.
// This method implements a PI-type control (proportional-integral).
void SetControlOutput(int error)
{
	static float Kc = CO_GAIN;				// Kc = controller gain (must be positive)
	static float Ci = 0.2;					// Ci = 1.0 / Ti (note: no Kc)
	int coLimit = CO_MAX;
	int absError = error;

	// Clear the Integral if direction of error changed
	if ((error < 0) == (PriorError >= 0))
		Integral = 0;
	PriorError = error;
	
	if (absError < 0) absError = -absError;
	Co = Kc * absError;					// proportional control
	if (Interval > INTERVAL_SLOW)		// speed is slower than nominal
	{
		Integral += Ci*Co;				// Note: Kc is in co; Ci must not include it
		coLimit = CO_SAFE;				// reduce CO limit to prevent overcurrent at low speeds
		if (Integral > coLimit) Integral = coLimit;		// keep values sane; avoid long-term windup
	}
	Co += Integral;

	if (Co < CO_MIN) Co = CO_MIN;
	if (Co > coLimit) Co = coLimit;
	if (error < 0) Co = -Co;
}


///////////////////////////////////////////////////////
void update_device()
{
	int error = Sp - Pos;
	reentrant void (*f)() = zero_CO;
	int co;
	
	if (Manual)
	{
		// enable for overshoot testing (sets Co to 0 after 1 turn at the commanded speed)
		//if (error == 0) Co = 0.0;

		if (Co == 0.0)
			/* do nothing */;
		else if ((Co < 0.0) == ROTATION)
			f = negative_CO;
		else
			f = positive_CO;
	}
	else if (Sp != 0 && error != 0)
	{
		SetControlOutput(error);
		
		if ((error < 0) == ROTATION)
			f = negative_CO;
		else
			f = positive_CO;
	}
	else	// Sp or error == 0; reset the control output variables
	{
		Co = 0.0;
		PriorError = 0;
		IntervalCounter = 0;
		Interval = INTERVAL_NORMAL;	// assume a normal speed until data is available
	}
	
	co = Co;
	if (co < 0) co = -co;
	
	DI();
	CO = co;
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
	char c, c2;
	int n;

	while (!RxbEmpty())					// process a command
	{
		mask_clr(Error, ERROR_COMMAND);
		GetInput();
		c = Command[0];					// a command
		c2 = Command[1];				// possibly a sub-command
		
		// single-byte commands
		if (c == '\0')					// null command
		{
			// do nothing
		}
		else if (c == 'r')				// report
		{
			if (NargPresent)			// set Datalogging interval
			{
				// rolls under to 0xFF (meaning "disable") if DatalogReset was 0
				DatalogReset = TryInput(0, 255, ERROR_DATALOG, DatalogReset + 1, 0) - 1;
				DatalogCount = 0;
			}
			else						// one-time report
				report_device();
		}
		else if (c == 's')				// stop any movement
		{
			Sp = Pos;
			Co = 0.0;
		}
		else if (c == 'c')				// clear Sp and Pos
		{
			Sp = Pos = 0;
			Manual = FALSE;
		}
		else if (c == 'm')				// manually set CO
		{
			Co = TryInput(-CO_MAX, CO_MAX, ERROR_COMMAND, Co, 0);
			if (!(Error & ERROR_COMMAND))
			{
				Manual = TRUE;
				Pos = 0;
				Sp = 96;
				if (Co < 0) Sp = -Sp;
			}
		}
		else if (c == 'g')				// move the given number of position units
		{
			if (NargPresent)
				Sp = TryInput(POS_MIN, POS_MAX, ERROR_POS, Sp, 0);
			if (!(Error & ERROR_POS))
			{
				// "reset" for new movement
				Pos = 0;
				PriorError = 0;
				Integral = 0.0;
				IntervalCounter = 0;
				Manual = FALSE;
			}
		}
		else if (c == 'h')				// report header
		{
			report_header();
		}
		else if (c == 'z')				// program data
		{
			printromstr(FIRMWARE); printromstr(VERSION); endMessage();
		}
		else							// unrecognized command
		{
			mask_set(Error, ERROR_COMMAND);
		}
	}
	
	if (EnableDatalogging)
	{
		EnableDatalogging = FALSE;		// re-enabled later by isr_timer0
		if (DatalogReset != 0xFF && uint8CounterReset(&DatalogCount, DatalogReset))
			report_device();
	}
}


///////////////////////////////////////////////////////
BOOL debounce(BYTE contact, BYTE *history, BYTE *state)
{
	*history = (*history << 1) | (contact & 0x01);
	if (*history == 0x7F)		// change state after bouncing ends
		*state = TRUE;
	else if (*history == 0x80)	// change state after bouncing ends
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
		{
			// check speed synchronously
			if (CO != 0.0)
			{
				if (Pos == PriorPos)	// no position update occurred during the prior interval
					IntervalCounter++;
				else	// movement detected
				{
					Interval = IntervalCounter;
					IntervalCounter = 0;
				}	
				if (IntervalCounter > Interval)
					Interval = IntervalCounter;
				
				PriorPos = Pos;	
			}
			EnableControllerUpdate = TRUE;
		}
				
//	if (ONE_SECOND)
	if ((T0Ticks & 255) == 0)		// Data logging interval units are 1/8 of second
		EnableDatalogging = TRUE;	
}


///////////////////////////////////////////////////////
// stop CO pulse
void interrupt isr_timer1()
{
	CO_off();
}


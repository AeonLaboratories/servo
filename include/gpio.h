///////////////////////////////////////////////////////
// gpio.h
//
// servo

#pragma once // Include this file only once

#include <ez8.h>
#include "..\\..\\common_controller\\include\\mask.h"

///////////////////////////////////////////////////////
// Conventions for configuring gpio pins
//
// Reserve DBG & -RESET for flashing/debugging
//
// Reserve PA4/RXD & PA5/TXD for UART
//
// Preferred analog inputs:
//		PB0/ANA0 .. PB3/ANA3
//
// Preferred digital inputs
// 		PC0 .. PC3
//
// Preferred digital outputs (open-drain or push-pull possible)
// 		PA0, PA1, PA2, PA3, PA6, PA7
//    Note: PA2 cannot output a strong high; use it active-low, 
//		or open-drain mode; avoid using it as an active-high digital output
//
// Alternate/additional analog inputs:
//		PC0/ANA4 .. PC2/ANA6
//
// Alternate/additional digital outputs (push-pull only)
//		PC0/ANA4 .. PC3/COUT
//		PB0/ANA0 .. PB3/ANA3
//
///////////////////////////////////////////////////////

///////////////////////////////////////////////////////
//
// NOTE: Open drain mode is only available on Port A gpio pins
// (See "Errata for Z8 Encore XP F082A Series UP0069.pdf")
//
// NOTE: PA2 cannot output a strong high. The PMOS output device 
// of the PA2 port is disabled. The PA2 port will not output a high level 
// unless the internal pull-up resistor is enabled. This pull-up resistor 
// is enabled by default but it should not be turned off unless the port 
// is pulled up externally. Because there is no active drive high, 
// the PA2 port will only produce slow rising edges.
// (See "Errata for Z8 Encore XP F082A Series UP0069.pdf")
//
///////////////////////////////////////////////////////

///////////////////////////////////////////////////////
// Port A
// PA7 =  IN:  RENC_B
// PA6 =  IN:  RENC_A
// PA5 = OUT:  TXD0 (Alt. function)
// PA4 =  IN:  RXD0 (Alt. function)
// PA3 = OUT:  CONE				// negative control output enable
// PA2 = OUT:  -COP				// positive control output
// PA1 = OUT:  COPE				// positive control output enable
// PA0 = OUT:  -CON				// negative control output
//
// DD == data direction
// OC == output control
// AF == alternate function
//
// PADD		= 11010000			// 1 = IN; 0 = OUT (set unused pins to OUT)
// PAOC		= 00001111			// 1 = open drain; 0 = push-pull (the default)
// PAAF		= 00110000			// alternate functions
// PAOUT	= 00000101			// defaults
//
#define PA_DD					0xD0
#define PA_OC					0x0F
#define PA_AF					0x30
#define PA_OUT					0x05

#define PIN_RENC_B				0x80
#define PIN_RENC_A				0x40

#define PIN_CONE				0x08
#define PIN_COP					0x04	// active low
#define PIN_COPE				0x02
#define PIN_CON					0x01	// active low

// COPE and CONE turn on the appropriate low-side transistor of the H-bridge
// COP and CON turn on the appropriate high-side transistor

#define RENC_A					((PAIN & PIN_RENC_A) == 0)
#define RENC_B					((PAIN & PIN_RENC_B) == 0)

#define CO_PORT					PAOUT

#define COP_disable()			mask_clr(CO_PORT, PIN_COPE)
#define COP_enable()			mask_set(CO_PORT, PIN_COPE)
#define COP_is_enabled()		((CO_PORT & PIN_COPE) != 0)

#define CON_disable()			mask_clr(CO_PORT, PIN_CONE)
#define CON_enable()			mask_set(CO_PORT, PIN_CONE)
#define CON_is_enabled()		((CO_PORT & PIN_CONE) != 0)

#define COP_on()				(CON_disable(), mask_clr(CO_PORT, PIN_COP))
#define COP_off()				mask_set(CO_PORT, PIN_COP)
#define COP_is_on()				((CO_PORT & PIN_COP) == 0)

#define CON_on()				(COP_disable(), mask_clr(CO_PORT, PIN_CON))
#define CON_off()				mask_set(CO_PORT, PIN_CON)
#define CON_is_on()				((CO_PORT & PIN_CON) == 0)

#define CO_disable()			(mask_set(CO_PORT, PIN_COP | PIN_CON), mask_clr(CO_PORT, PIN_COPE | PIN_CONE))
#define CO_off()				(mask_set(CO_PORT, PIN_COP | PIN_CON), mask_set(CO_PORT, PIN_COPE | PIN_CONE))
#define CO_is_on()				((CO_PORT & (PIN_COP | PIN_CON)) != (PIN_COP | PIN_CON))


///////////////////////////////////////////////////////
// Port B
// PB7 = N/A
// PB6 = N/A
// PB5 = N/A
// PB4 = N/A
// PB3 = OUT: no connect
// PB2 = OUT: no connect
// PB1 = OUT: no connect
// PB0 = OUT: no connect
//
// PBDD		= 00000000		// 1 = IN; 0 = OUT (set unused pins to OUT)
// PBAF		= 00000000		// alternate functions
// PBOUT	= 00000000		// defaults
//
// NOTE: open drain mode does not work for Port B gpio pins
// (See "Errata for Z8 Encore XP F082A Series UP0069.pdf")
//
#define PB_DD					0x00
#define PB_AF					0x00
#define PB_OUT					0x00

///////////////////////////////////////////////////////
// Port C
// PC7 = N/A
// PC6 = N/A
// PC5 = N/A
// PC4 = N/A
// PC3 = IN: CMD
// PC2 = OUT: no connect
// PC1 = OUT: no connect
// PC0 = OUT: no connect
//
// PCDD		= 00001000		// 1 = IN; 0 = OUT (set unused pins to OUT)
// PCAF		= 00000000		// alternate functions
// PCOUT	= 00000000		// default values
//
// NOTE: open drain mode does not work for Port C gpio pins
// (See "Errata for Z8 Encore XP F082A Series UP0069.pdf")
//
// NOTE: External Vref is not available on 20-pin parts.
// For 20-pin devices, external Vref is not an option on pin PC2.
// There is no workaround.
// (See "Errata for Z8 Encore XP F082A Series UP0069.pdf")
//

//
#define PC_DD					0x08
#define PC_AF					0x00
#define PC_OUT					0x00

#define PIN_CMD					0x08
#define CMD_is_low()			((PCIN & PIN_CMD) == 0)
#define CMD_is_high()			((PCIN & PIN_CMD) != 0)



///////////////////////////////////////////////////////
// prototypes
//
void init_gpio(void);

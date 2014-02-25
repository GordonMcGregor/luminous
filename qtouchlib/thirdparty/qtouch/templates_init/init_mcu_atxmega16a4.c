/*******************************************************************************
*   $FILE:  main.c
*   Atmel Corporation:  http://www.atmel.com \n
*   Support email:  touch@atmel.com
******************************************************************************/

/*  License
*   Copyright (c) 2010, Atmel Corporation All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions are met:
*
*   1. Redistributions of source code must retain the above copyright notice,
*   this list of conditions and the following disclaimer.
*
*   2. Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
*   3. The name of ATMEL may not be used to endorse or promote products derived
*   from this software without specific prior written permission.
*
*   THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
*   WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
*   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
*   SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
*   INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
*   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
*   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
*   THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*----------------------------------------------------------------------------
                            compiler information
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
                                include files
----------------------------------------------------------------------------*/
#include "asf.h"
#include "touch.h"
#include "touch_api.h"
/*----------------------------------------------------------------------------
                            manifest constants
----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------
                            type definitions
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
                                prototypes
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
                            Structure Declarations
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
                                    macros
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
                                global variables
----------------------------------------------------------------------------*/
/* Timer period in msec. */
extern uint16_t qt_measurement_period_msec;

/*----------------------------------------------------------------------------
                                extern variables
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
                                static variables
----------------------------------------------------------------------------*/

/* flag set by timer ISR when it's time to measure touch */
extern volatile uint8_t time_to_measure_touch;

/* current time, set by timer ISR */
extern volatile uint16_t current_time_ms_touch;

#if defined(__ATxmega16A4__)

/*============================================================================
Name    :   timer_isr
------------------------------------------------------------------------------
Purpose :   timer ISR to update time info
============================================================================*/
static void
example_cca_interrupt_callback (void)
{
  /*  set flag: it's time to measure touch    */
  time_to_measure_touch = 1u;
  /*  update the current time  */
  current_time_ms_touch += qt_measurement_period_msec;
}

/*============================================================================
Name    :   init_timer_isr
------------------------------------------------------------------------------
Purpose :   configure timer ISR to fire regularly
============================================================================*/

void init_timer_isr( void )
{
  tc_enable (&TCC0);
  tc_write_period (&TCC0, (TICKS_PER_MS * qt_measurement_period_msec));
  tc_write_clock_source (&TCC0, TC_CLKSEL_DIV8_gc);
  tc_set_cca_interrupt_level (&TCC0, PMIC_LVL_LOW);
  tc_set_cca_interrupt_callback(&TCC0, example_cca_interrupt_callback);
}

/*============================================================================
Name    :   init_system
------------------------------------------------------------------------------
Purpose :   initialise host app, pins, watchdog, etc
============================================================================*/
void init_system( void )
{
	// Initialize ASF services
	sysclk_init();
	board_init();

}


#endif






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
#include <avr/io.h>
#include <avr/interrupt.h>
#include "touch.h"
/*----------------------------------------------------------------------------
                            manifest constants
----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------
                            type definitions
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
                                prototypes
----------------------------------------------------------------------------*/
/* configure timer ISR to fire regularly */
void init_timer_isr( void );

/* initialise host app, pins, watchdog, etc */
void init_system( void );

/*----------------------------------------------------------------------------
                            Structure Declarations
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
                                    macros
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
                                global variables
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
                                extern variables
----------------------------------------------------------------------------*/
/* Timer period in msec. */
extern uint16_t qt_measurement_period_msec;
extern uint16_t time_ms_inc;
/*----------------------------------------------------------------------------
                                static variables
----------------------------------------------------------------------------*/

/* flag set by timer ISR when it's time to measure touch */
extern volatile uint8_t time_to_measure_touch;

/* current time, set by timer ISR */
extern volatile uint16_t current_time_ms_touch;

#if defined(__ATtiny861A__)
#if defined(_QTOUCH_) || defined(_QMATRIX_)

/*==============================================================================
Name    : init_timer_isr
--------------------------------------------------------------------------------
Purpose : configure timer ISR to interrupt regularly
Input   :
Output  :
Notes   :
==============================================================================*/
void init_timer_isr( void )
{
   /*  set timer compare value (how often timer ISR will fire) */
   OCR1A  = (TICKS_PER_MS * QT_TIMER_PERIOD_MSEC);
   /*  enable timer ISR */
   TIMSK  = (1u << OCIE1A);
   /*  timer prescaler = system clock / 64  */
   TCCR1B = (1u << CS12) | (1u << CS11) | (1u << CS10);
   /*  timer mode = CTC (count up to compare value, then reset)    */
   TCCR1D = (1u << WGM11);
}

/*==============================================================================
Name    : timer_isr
--------------------------------------------------------------------------------
Purpose : timer 0 compare ISR
Input   :
Output  :
Notes   :
==============================================================================*/
ISR(TIMER1_COMPA_vect)
{
  time_ms_inc += QT_TIMER_PERIOD_MSEC;

  if(time_ms_inc >= qt_measurement_period_msec)
  {
    time_ms_inc =0;
    /*  set flag: it's time to measure touch    */
    time_to_measure_touch = 1u;
  }
  else
  {

  }
  /*  update the current time */
   current_time_ms_touch += QT_TIMER_PERIOD_MSEC;
}
#endif
/*============================================================================
Name    :   init_system
------------------------------------------------------------------------------
Purpose :   initialise host app, pins, watchdog, etc
============================================================================*/
void init_system( void )
{
    /*  run at 4MHz (assuming internal clock is set to 8MHz)*/
   asm("ldi r16,0x80");
   asm("sts 0x48,r16");
   asm("ldi r16,0x01");
   asm("sts 0x48,r16");

    /*  disable pull-ups    */
    MCUCR |= (1u << PUD);

}
#endif







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
Example File History
----------------------------------------------------------------------------*/
/* QT600 Example projet version 1.2
* ---------------------------------
* To work with QT library version 4.0
*/

/* QT600 Example projet version 1.1
* ---------------------------------
*/

/*----------------------------------------------------------------------------
Device Fuse Settings
----------------------------------------------------------------------------*/
/*	The default fuse settings for the device will be correct for the
QT600 Example Project. However it is adviceable to verify
the fuse settings before trying to run the application.

WDWP:       8 cycles (8ms @ 3.3V)
WDP:        8 cycles (8ms @ 3.3V)
BOOTRST:	Application Reset
BODACT:     BOD Disabled
BODPD:      BOD Disabled
JTAGEN:     Programmed

Fusebyte0: 0xFF
Fusebyte1: 0x00
Fusebyte2: 0xFF
Fusebyte4: 0xFE
Fusebyte5: 0xFF
*/

/*----------------------------------------------------------------------------
compiler information
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
include files
----------------------------------------------------------------------------*/
#include <avr/io.h>
#include <avr/interrupt.h>
#define __delay_cycles(n)     __builtin_avr_delay_cycles(n)
#define __enable_interrupt()  sei()

/*  now include touch api.h with the localization defined above */
#include "touch_api.h"

#include "QDebug.h"
#include "QDebugTransport.h"

/*----------------------------------------------------------------------------
manifest constants
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
type definitions
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
prototypes
----------------------------------------------------------------------------*/
/*  initialise host app, pins, watchdog, etc    */
static void init_system( void );
/*  configure timer ISR to fire regularly   */
static void init_timer_isr( void );
/*  Assign the parameters values to global configuration parameter structure    */
static void qt_set_parameters( void );

/*----------------------------------------------------------------------------
Structure Declarations
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
macros
----------------------------------------------------------------------------*/
/*  The timing information for timer to fire periodically to measure touch  */
#ifdef TICKS_PER_MS
#undef TICKS_PER_MS
#define TICKS_PER_MS                1000u //500u. 1.1 change as clock is running at 8MHz
#endif

#define GET_SENSOR_STATE(SENSOR_NUMBER) qt_measure_data.qt_touch_status.sensor_states[(SENSOR_NUMBER/8)] & (1 << (SENSOR_NUMBER % 8))
#define GET_ROTOR_SLIDER_POSITION(ROTOR_SLIDER_NUMBER) qt_measure_data.qt_touch_status.rotor_slider_values[ROTOR_SLIDER_NUMBER]

#define TIMER_COUNTER_L TCC0.CNTL
#define TIMER_COUNTER_H TCC0.CNTH

/*----------------------------------------------------------------------------
global variables
----------------------------------------------------------------------------*/

// only access with NOT_TIMEDOUT and SET_TIMEOUT
/* Timer period in msec. */
uint16_t qt_measurement_period_msec = 25u;

#ifdef _DEBUG_INTERFACE_
extern uint16_t timestamp1_hword;
extern uint16_t timestamp1_lword;
extern uint16_t timestamp2_hword;
extern uint16_t timestamp2_lword;
extern uint16_t timestamp3_hword;
extern uint16_t timestamp3_lword;
#endif
/*----------------------------------------------------------------------------
extern variables
----------------------------------------------------------------------------*/
/* This configuration data structure parameters if needs to be changed will be
changed in the qt_set_parameters function */
extern qt_touch_lib_config_data_t qt_config_data;
/* touch output - measurement data */
extern qt_touch_lib_measure_data_t qt_measure_data;
/* Get sensor delta values */
extern int16_t qt_get_sensor_delta( uint8_t sensor);
/* Output can be observed in the watch window using this pointer */
qt_touch_lib_measure_data_t *pqt_measure_data = &qt_measure_data;

#ifdef QDEBUG_TWI
extern unsigned int gMsTimeout;
#endif
/*----------------------------------------------------------------------------
static variables
----------------------------------------------------------------------------*/
/* flag set by timer ISR when it's time to measure touch */
static volatile uint8_t time_to_measure_touch = 0u;
/* current time, set by timer ISR */
volatile uint16_t current_time_ms_touch = 0u;

/*============================================================================
Name    :   main
------------------------------------------------------------------------------
Purpose :   main code entry point
Input   :   n/a
Output  :   n/a
Notes   :
============================================================================*/
int main( void )
{
    /*status flags to indicate the re-burst for library*/
    uint16_t status_flag = 0u;
    uint16_t burst_flag = 0u;

    init_timer_isr();
#ifdef _DEBUG_INTERFACE_
    timestamp1_hword = current_time_ms_touch;
    timestamp1_lword = (uint16_t)TIMER_COUNTER_L;
    timestamp1_lword |= (uint16_t)(TIMER_COUNTER_H << 8);
#endif
    /* initialise host app, pins, watchdog, etc */
    init_system();

    /* Config sensors  */
    qt_enable_key( CHANNEL_0, NO_AKS_GROUP, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_1, NO_AKS_GROUP, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_2, NO_AKS_GROUP, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_3, NO_AKS_GROUP, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_4, NO_AKS_GROUP, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_5, NO_AKS_GROUP, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_6, NO_AKS_GROUP, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_7, NO_AKS_GROUP, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_8, NO_AKS_GROUP, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_9, NO_AKS_GROUP, 10u, HYST_6_25 );
    qt_enable_slider( CHANNEL_10, CHANNEL_12, NO_AKS_GROUP, 10u, HYST_6_25, RES_8_BIT, 0u  );
    qt_enable_rotor( CHANNEL_13, CHANNEL_15, NO_AKS_GROUP, 6u, HYST_50, RES_8_BIT, 0u );

    /* Initialise and set touch params */
    qt_init_sensing();
    qt_set_parameters();

    /*  Address to pass address of user functions   */
    /*  This function is called after the library has made capacitive measurements,
    *   but before it has processed them. The user can use this hook to apply filter
    *   functions to the measured signal values.(Possibly to fix sensor layout faults)    */
    qt_filter_callback = 0;
    #ifdef _DEBUG_INTERFACE_
	/* Initialize debug protocol */
    QDebug_Init();
	#endif
    __enable_interrupt();
	#ifdef _DEBUG_INTERFACE_
    /* Process commands from PC */
    QDebug_ProcessCommands();
	#endif
    /* loop forever */
    for( ; ; )
    {
        if( time_to_measure_touch )
        {
            time_to_measure_touch = 0u;
            do
            {
                #ifdef _DEBUG_INTERFACE_
				timestamp2_hword = current_time_ms_touch;
                timestamp2_lword = (uint16_t)TIMER_COUNTER_L;
                timestamp2_lword |= (uint16_t)(TIMER_COUNTER_H << 8);
				#endif
                /* Measure touch once */
                status_flag = qt_measure_sensors( current_time_ms_touch );
				#ifdef _DEBUG_INTERFACE_
                timestamp3_hword = current_time_ms_touch;
                timestamp3_lword = (uint16_t)TIMER_COUNTER_L;
                timestamp3_lword |= (uint16_t)(TIMER_COUNTER_H << 8);
				#endif
                burst_flag = status_flag & QTLIB_BURST_AGAIN;
				#ifdef _DEBUG_INTERFACE_
                /* send debug data */
                QDebug_SendData(status_flag);
				#endif
                /*Time critical host application code goes here*/

            }while (burst_flag) ;
            #ifdef _DEBUG_INTERFACE_
            /* Process commands from PC */
            QDebug_ProcessCommands();
        	#endif
		}

        /* Non time-critical application code goes here */
    }
}

/*============================================================================
Name    :   qt_set_parameters
------------------------------------------------------------------------------
Purpose :   This will fill the default threshold values in the configuration
data structure.But User can change the values of these parameters .
Input   :   n/a
Output  :   n/a
Notes   :   initialize configuration data for processing
============================================================================*/
static void qt_set_parameters( void )
{
   qt_config_data.qt_di              = DEF_QT_DI;
   qt_config_data.qt_neg_drift_rate  = DEF_QT_NEG_DRIFT_RATE;
   qt_config_data.qt_pos_drift_rate  = DEF_QT_POS_DRIFT_RATE;
   qt_config_data.qt_max_on_duration = DEF_QT_MAX_ON_DURATION;
   qt_config_data.qt_drift_hold_time = DEF_QT_DRIFT_HOLD_TIME;
   qt_config_data.qt_recal_threshold = DEF_QT_RECAL_THRESHOLD;
   qt_config_data.qt_pos_recal_delay = DEF_QT_POS_RECAL_DELAY;
}

/*============================================================================
Name    :   init_timer_isr
------------------------------------------------------------------------------
Purpose :   configure timer ISR to fire regularly
Input   :   n/a
Output  :   n/a
Notes   :
============================================================================*/
static void init_timer_isr( void )
{
   /*  Set timer period    */
   TCC0.PER = TICKS_PER_MS * qt_measurement_period_msec;
   /*  select clock source */
   TCC0.CTRLA = (TOUCH_DATA_T)4;
   /*  Set Comparre A interrupt to low level   */
   TCC0.INTCTRLB = 1u;
   /*  enable low lever interrupts in power manager interrupt control  */
   PMIC.CTRL |= 1u;
}

/*============================================================================
Name    :   CCP write helper function written in assembly.
------------------------------------------------------------------------------
Purpose :   This function is written in assembly because of the timecritial
operation of writing to the registers for xmega.
Input   :   address - A pointer to the address to write to.
value   - The value to put in to the register.
Notes   :
============================================================================*/
void CCPWrite( volatile uint8_t * address, uint8_t value )
{
   volatile uint8_t * tmpAddr = address;
#ifdef RAMPZ
   RAMPZ = 0;
#endif
   asm volatile(
                "movw r30,  %0"	"\n\t"
                "ldi  r16,  %2"	"\n\t"
                "out   %3, r16"	"\n\t"
                "st     Z,  %1"
                :
                : "r" (tmpAddr), "r" (value), "M" (CCP_IOREG_gc), "m" (CCP)
                : "r16", "r30", "r31"
               );
}

/*============================================================================
Name    :   init_system
------------------------------------------------------------------------------
Purpose :   initialise host app, pins, watchdog, etc
Input   :   n/a
Output  :   n/a
Notes   :
============================================================================*/
static void init_system( void )
{
   uint8_t PSconfig;
   uint8_t clkCtrl;

   /*  Configure Oscillator and Clock source   */

   /*  Select Prescaler A divider as 4 and Prescaler B & C divider as (1,1) respectively.  */
   /*  Overall divide by 4 i.e. A*B*C  */
   PSconfig = (uint8_t) CLK_PSADIV_4_gc | CLK_PSBCDIV_1_1_gc;
   /*  Enable internal 32 MHz ring oscillator. */
   OSC.CTRL |= OSC_RC32MEN_bm;
   CCPWrite( &CLK.PSCTRL, PSconfig );
   /*  Wait until oscillator is ready. */
   while ( ( OSC.STATUS & OSC_RC32MRDY_bm ) == 0 );
   /*  Set the 32 MHz ring oscillator as the main clock source */
   clkCtrl = ( CLK.CTRL & ~CLK_SCLKSEL_gm ) | CLK_SCLKSEL_RC32M_gc;
   CCPWrite( &CLK.CTRL, clkCtrl );

   /*  Route clk signal to port pin    */
   /*  PORTCFG_CLKEVOUT = 0x03;    */
   /*  PORTE_DIRSET = 0x80;    */
}

/*============================================================================
Name    :   timer_isr
------------------------------------------------------------------------------
Purpose :   timer 1 compare ISR
Input   :   n/a
Output  :   n/a
Notes   :
============================================================================*/
ISR(TCC0_CCA_vect)
{
#ifdef QDEBUG_TWI
   if (gMsTimeout)
   {
      gMsTimeout--;
   }
#endif
   /*  set flag: it's time to measure touch    */
   time_to_measure_touch = 1u;
   /*  update the current time  */
   current_time_ms_touch += qt_measurement_period_msec;
}

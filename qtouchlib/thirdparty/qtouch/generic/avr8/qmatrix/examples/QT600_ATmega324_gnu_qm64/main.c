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

BODLEVEL:	BOD Disabled
JTAGEN:     Programmed
SPIEN:      Programmed
CKDIV8:		Programmed
CKOUT:		UnProgrammed
SUT_CKSEL:	Int RC 8MHz, Any Start Up Time

Extended Fuse: 0xFF
High Fuse:     0x9F
Low Fuse:      0x62
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

#include "touch_api.h"
#ifdef _DEBUG_INTERFACE_
#include "QDebug.h"
#include "QDebugTransport.h"
#endif/* _DEBUG_INTERFACE_*/

/*----------------------------------------------------------------------------
manifest constants
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
type definitions
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
prototypes
----------------------------------------------------------------------------*/
/* initialise host app, pins, watchdog, etc */
static void init_system( void );
/* configure timer ISR to fire regularly */
static void init_timer_isr( void );
/* Assign the parameters values to global configuration parameter structure */
static void qt_set_parameters( void );
/* Configure the Burst Length*/
static void burst_len_config( void );

/*----------------------------------------------------------------------------
macros
----------------------------------------------------------------------------*/

#define GET_SENSOR_STATE(SENSOR_NUMBER) qt_measure_data.qt_touch_status.sensor_states[(SENSOR_NUMBER/8)] & (1 << (SENSOR_NUMBER % 8))
#define GET_ROTOR_SLIDER_POSITION(ROTOR_SLIDER_NUMBER) qt_measure_data.qt_touch_status.rotor_slider_values[ROTOR_SLIDER_NUMBER]
#ifdef _DEBUG_INTERFACE_
#define TIMER_COUNTER_L TCNT0
#define TIMER_COUNTER_H 0
#endif/* _DEBUG_INTERFACE_*/

/*----------------------------------------------------------------------------
global variables
----------------------------------------------------------------------------*/
/* Timer period in msec. */
uint16_t qt_measurement_period_msec = 25u;

#ifdef TICKS_PER_MS
#undef TICKS_PER_MS
#define TICKS_PER_MS                8u
#endif

#ifdef _DEBUG_INTERFACE_
extern uint16_t timestamp1_hword;
extern uint16_t timestamp1_lword;    
extern uint16_t timestamp2_hword;
extern uint16_t timestamp2_lword;
extern uint16_t timestamp3_hword;
extern uint16_t timestamp3_lword;
#endif/* _DEBUG_INTERFACE_*/
/*----------------------------------------------------------------------------
extern variables
----------------------------------------------------------------------------*/
/* This configuration data structure parameters if needs to be changed will be
changed in the qt_set_parameters function */
extern qt_touch_lib_config_data_t qt_config_data;

#ifdef _QMATRIX_
extern y_line_info_t ya_line_info[NUM_Y_LINES];
extern y_line_info_t yb_line_info[NUM_Y_LINES];
extern x_line_info_t x_line_info[NUM_X_LINES];

/* Fill out the X-Line masks on the X- Port selected.
* The order of X - Line numbering follows from the way the
* X-Lines are filled as below.
* Here, X0,X1,X2,X3,X4,X5,X6,X7 are on port-pin specified.
* 1 - if to specify if X line is on PORT_X_1,pin on the same selected port.
* 2 - if to specify if X line is on PORT_X_2,pin on the same selected port.
* 3 - if to specify if X line is on PORT_X_3,pin on the same selected port.
*
* Note: 1. The Number entries should be based on NUM_X_LINES
*          4 entries when NUM_X_LINES =4 and
*          8 entries when NUM_X_LINES=8
*/

x_line_info_t x_line_info[NUM_X_LINES]= {
   FILL_OUT_X_LINE_INFO(  1,0u ),
   FILL_OUT_X_LINE_INFO(  1,1u ),
   FILL_OUT_X_LINE_INFO(  1,2u ),
   FILL_OUT_X_LINE_INFO(  1,3u ),
   FILL_OUT_X_LINE_INFO(  1,4u ),
   FILL_OUT_X_LINE_INFO(  1,5u ),
   FILL_OUT_X_LINE_INFO(  1,6u ),
   FILL_OUT_X_LINE_INFO(  1,7u ),
};

/* Fill out the Y-Line masks on the Y- Line port selected
* The order of Y - Line numbering follows from the way the
* Y-Lines are filled as below
* Here, Y0,Y1,Y2,Y3 on 0,1,2,3
* Note: 1. The Number entries should be based on NUM_X_LINES
*          2 entries when NUM_Y_LINES=2
*          4 entries when NUM_Y_LINES=4
*          8 entries when NUM_Y_LINES=8
*/
y_line_info_t ya_line_info[NUM_Y_LINES]= {
   FILL_OUT_YA_LINE_INFO(  0u ),
   FILL_OUT_YA_LINE_INFO(  1u ),
   FILL_OUT_YA_LINE_INFO(  2u ),
   FILL_OUT_YA_LINE_INFO(  3u ),
   FILL_OUT_YA_LINE_INFO(  4u ),
   FILL_OUT_YA_LINE_INFO(  5u ),
   FILL_OUT_YA_LINE_INFO(  6u ),
   FILL_OUT_YA_LINE_INFO(  7u ),
};
y_line_info_t yb_line_info[NUM_Y_LINES]= {
   FILL_OUT_YB_LINE_INFO(  0u ),
   FILL_OUT_YB_LINE_INFO(  1u ),
   FILL_OUT_YB_LINE_INFO(  2u ),
   FILL_OUT_YB_LINE_INFO(  3u ),
   FILL_OUT_YB_LINE_INFO(  4u ),
   FILL_OUT_YB_LINE_INFO(  5u ),
   FILL_OUT_YB_LINE_INFO(  6u ),
   FILL_OUT_YB_LINE_INFO(  7u ),
};
#endif/*_QMATRIX_*/

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
#endif/* _DEBUG_INTERFACE_*/

    /* initialise host app, pins, watchdog, etc */
    init_system();
    
    /*Configure Burst Length*/
    burst_len_config();

    /* Configure the Sensors as keys or Keys With Rotor/Sliders in this function */
    qt_enable_rotor( CHANNEL_0, CHANNEL_3, NO_AKS_GROUP, 32u, HYST_50, RES_8_BIT, 0u  );
    qt_enable_slider( CHANNEL_4, CHANNEL_7, NO_AKS_GROUP, 30u, HYST_50, RES_8_BIT, 0u  );

    qt_enable_key( CHANNEL_8, NO_AKS_GROUP, 16u, HYST_50 );
    qt_enable_key( CHANNEL_9, NO_AKS_GROUP, 16u, HYST_50 );
    qt_enable_key( CHANNEL_10, NO_AKS_GROUP, 20u, HYST_50 );
    qt_enable_key( CHANNEL_11, NO_AKS_GROUP, 20u, HYST_50 );
    qt_enable_key( CHANNEL_12, NO_AKS_GROUP, 30u, HYST_50 );
    qt_enable_key( CHANNEL_13, NO_AKS_GROUP, 20u, HYST_50 );
    qt_enable_key( CHANNEL_14, NO_AKS_GROUP, 30u, HYST_50 );
    qt_enable_key( CHANNEL_15, NO_AKS_GROUP, 30u, HYST_50 );

    qt_enable_rotor( CHANNEL_16, CHANNEL_19, NO_AKS_GROUP, 32u, HYST_50, RES_8_BIT, 0u  );
    qt_enable_slider( CHANNEL_20, CHANNEL_23, NO_AKS_GROUP, 30u, HYST_50, RES_8_BIT, 0u  );

    qt_enable_key( CHANNEL_24, NO_AKS_GROUP, 16u, HYST_50 );
    qt_enable_key( CHANNEL_25, NO_AKS_GROUP, 16u, HYST_50 );
    qt_enable_key( CHANNEL_26, NO_AKS_GROUP, 20u, HYST_50 );
    qt_enable_key( CHANNEL_27, NO_AKS_GROUP, 20u, HYST_50 );
    qt_enable_key( CHANNEL_28, NO_AKS_GROUP, 20u, HYST_50 );
    qt_enable_key( CHANNEL_29, NO_AKS_GROUP, 20u, HYST_50 );
    qt_enable_key( CHANNEL_30, NO_AKS_GROUP, 20u, HYST_50 );
    qt_enable_key( CHANNEL_31, NO_AKS_GROUP, 20u, HYST_50 );

    qt_enable_rotor( CHANNEL_32, CHANNEL_35, NO_AKS_GROUP, 32u, HYST_50, RES_8_BIT, 0u  );
    qt_enable_slider( CHANNEL_36, CHANNEL_39, NO_AKS_GROUP, 28u, HYST_50, RES_8_BIT, 0u  );

    qt_enable_key( CHANNEL_40, NO_AKS_GROUP, 16u, HYST_50 );
    qt_enable_key( CHANNEL_41, NO_AKS_GROUP, 16u, HYST_50 );
    qt_enable_key( CHANNEL_42, NO_AKS_GROUP, 20u, HYST_50 );
    qt_enable_key( CHANNEL_43, NO_AKS_GROUP, 20u, HYST_50 );
    qt_enable_key( CHANNEL_44, NO_AKS_GROUP, 20u, HYST_50 );
    qt_enable_key( CHANNEL_45, NO_AKS_GROUP, 20u, HYST_50 );
    qt_enable_key( CHANNEL_46, NO_AKS_GROUP, 20u, HYST_50 );
    qt_enable_key( CHANNEL_47, NO_AKS_GROUP, 20u, HYST_50 );

    qt_enable_rotor( CHANNEL_48, CHANNEL_51, NO_AKS_GROUP, 32u, HYST_50, RES_8_BIT, 0u  );
    qt_enable_slider( CHANNEL_52, CHANNEL_55, NO_AKS_GROUP, 28u, HYST_50, RES_8_BIT, 0u  );

    qt_enable_key( CHANNEL_56, NO_AKS_GROUP, 16u, HYST_50 );
    qt_enable_key( CHANNEL_57, NO_AKS_GROUP, 16u, HYST_50 );
    qt_enable_key( CHANNEL_58, NO_AKS_GROUP, 20u, HYST_50 );
    qt_enable_key( CHANNEL_59, NO_AKS_GROUP, 20u, HYST_50 );
    qt_enable_key( CHANNEL_60, NO_AKS_GROUP, 20u, HYST_50 );
    qt_enable_key( CHANNEL_61, NO_AKS_GROUP, 20u, HYST_50 );
    qt_enable_key( CHANNEL_62, NO_AKS_GROUP, 20u, HYST_50 );
    qt_enable_key( CHANNEL_63, NO_AKS_GROUP, 20u, HYST_50 );

    /* Initialise and set touch params */
    qt_init_sensing();
    qt_set_parameters();

    /*  Address to pass address of user functions   */
    /*  This function is called after the library has made capacitive measurements,
    *   but before it has processed them. The user can use this hook to apply filter
    *   functions to the measured signal values.(Possibly to fix sensor layout faults)    */
    qt_filter_callback = 0;
    /* Initialize debug protocol */
#ifdef _DEBUG_INTERFACE_
    QDebug_Init();	
#endif/* _DEBUG_INTERFACE_*/
    __enable_interrupt();

    /* Process commands from PC */
#ifdef _DEBUG_INTERFACE_
    QDebug_ProcessCommands();
#endif/* _DEBUG_INTERFACE_*/

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
#endif/* _DEBUG_INTERFACE_*/
                /* Measure touch once */
                status_flag = qt_measure_sensors( current_time_ms_touch );
#ifdef _DEBUG_INTERFACE_
                timestamp3_hword = current_time_ms_touch;
                timestamp3_lword = (uint16_t)TIMER_COUNTER_L;
                timestamp3_lword |= (uint16_t)(TIMER_COUNTER_H << 8);
#endif/* _DEBUG_INTERFACE_*/
                burst_flag = status_flag & QTLIB_BURST_AGAIN;

                /* send debug data */ 
#ifdef _DEBUG_INTERFACE_
                QDebug_SendData(status_flag);
#endif/* _DEBUG_INTERFACE_*/

                /* Time-critical host application code goes here */

            }while (burst_flag) ;

            /* Process commands from PC */
#ifdef _DEBUG_INTERFACE_
            QDebug_ProcessCommands();
#endif/* _DEBUG_INTERFACE_*/
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
   qt_config_data.qt_recal_threshold = RECAL_100;  //DEF_QT_RECAL_THRESHOLD;
   qt_config_data.qt_pos_recal_delay = 10u;        //DEF_QT_POS_RECAL_DELAY;    
}
/*============================================================================
Name    :   burst_len_config
------------------------------------------------------------------------------
Purpose :   configure the individual channel burst length
Input   :   n/a
Output  :   n/a
Notes   :   n/a
============================================================================*/
static void burst_len_config( void )
{
   qt_burst_lengths[0] = 32u;
   qt_burst_lengths[1] = 32u;
   qt_burst_lengths[2] = 32u;
   qt_burst_lengths[3] = 32u;
   qt_burst_lengths[4] = 40u;
   qt_burst_lengths[5] = 32u;
   qt_burst_lengths[6] = 32u;
   qt_burst_lengths[7] = 34u;
   qt_burst_lengths[8] = 25u;
   qt_burst_lengths[9] = 25u;
   qt_burst_lengths[10] = 32u;
   qt_burst_lengths[11] = 32u;
   qt_burst_lengths[12] = 32u;
   qt_burst_lengths[13] = 32u;
   qt_burst_lengths[14] = 32u;
   qt_burst_lengths[15] = 32u;
   qt_burst_lengths[16] = 32u;
   qt_burst_lengths[17] = 32u;
   qt_burst_lengths[18] = 32u;
   qt_burst_lengths[19] = 32u;
   qt_burst_lengths[20] = 34u;
   qt_burst_lengths[21] = 32u;
   qt_burst_lengths[22] = 32u;
   qt_burst_lengths[23] = 34u;
   qt_burst_lengths[24] = 25u;
   qt_burst_lengths[25] = 25u;
   qt_burst_lengths[26] = 32u;
   qt_burst_lengths[27] = 32u;
   qt_burst_lengths[28] = 32u;
   qt_burst_lengths[29] = 32u;
   qt_burst_lengths[30] = 32u;
   qt_burst_lengths[31] = 32u;
   qt_burst_lengths[32] = 32u;
   qt_burst_lengths[33] = 32u;
   qt_burst_lengths[34] = 32u;
   qt_burst_lengths[35] = 32u;
   qt_burst_lengths[36] = 34u;
   qt_burst_lengths[37] = 32u;
   qt_burst_lengths[38] = 32u;
   qt_burst_lengths[39] = 34u;
   qt_burst_lengths[40] = 25u;
   qt_burst_lengths[41] = 25u;
   qt_burst_lengths[42] = 32u;
   qt_burst_lengths[43] = 32u;
   qt_burst_lengths[44] = 32u;
   qt_burst_lengths[45] = 32u;
   qt_burst_lengths[46] = 32u;
   qt_burst_lengths[47] = 32u;
   qt_burst_lengths[48] = 32u;
   qt_burst_lengths[49] = 32u;
   qt_burst_lengths[50] = 32u;
   qt_burst_lengths[51] = 32u;
   qt_burst_lengths[52] = 34u;
   qt_burst_lengths[53] = 32u;
   qt_burst_lengths[54] = 32u;
   qt_burst_lengths[55] = 40u;
   qt_burst_lengths[56] = 25u;
   qt_burst_lengths[57] = 25u;
   qt_burst_lengths[58] = 32u;
   qt_burst_lengths[59] = 32u;
   qt_burst_lengths[60] = 32u;
   qt_burst_lengths[61] = 32u;
   qt_burst_lengths[62] = 32u;
   qt_burst_lengths[63] = 32u;
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
   /*  set timer compare value (how often timer ISR will fire) */
   OCR0A = ( TICKS_PER_MS * qt_measurement_period_msec);
   /*  enable timer ISR on compare A */
   TIMSK0 = 0x02u; // Caution
   /*  timer prescaler = system clock / 1024  */
   TCCR0B = 0x05u; // Caution
   /*  timer mode = CTC (count up to compare value, then reset)    */
   TCCR0A = 0x02u; // Caution
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
   /* run at 8MHz */
   //CLKPR = 0x80u;
   //CLKPR = 0x00u;
   asm("ldi r16,0x80");
   asm("sts 0x61,r16");
   asm("ldi r16,0x00");
   asm("sts 0x61,r16");

   /* Disable the JTAG Pins */
   /*MCUCR |= (1u << JTD);*/
   /*MCUCR |= (1u << JTD);*/
   asm("ldi r16,0x80");
   asm("sts 0x55,r16");
   asm("ldi r16,0x80");
   asm("sts 0x55,r16");

   asm("nop");
   asm("nop");

   /* disable pull-ups */
   MCUCR |= (1u << PUD);
}

/*============================================================================
Name    :   timer_isr
------------------------------------------------------------------------------
Purpose: timer 0 compare ISR
Input  : n/a
Output : n/a
Notes  :
============================================================================*/
ISR(TIMER0_COMPA_vect)
{
   /* set flag: it's time to measure touch */
   time_to_measure_touch = 1u;
   /* update the current time */
   current_time_ms_touch += qt_measurement_period_msec;
}

/*=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-EOF-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/

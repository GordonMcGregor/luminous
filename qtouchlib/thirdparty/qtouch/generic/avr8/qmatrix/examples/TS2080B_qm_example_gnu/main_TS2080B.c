/* This file has been prepared for Doxygen automatic documentation generation.*/

/* $FILE:  main.c

* $BRIEF:
    This source file is the main.c for touch sensing. the library has to be

    included along with this file to create a complete touch sensing application

    added to the host application program.
*

*   Atmel Corporation: http://www.atmel.com \n

*   Support email: avr@atmel.com

*	$Revision:  0002 $

* $Date:   03 Sep 2009 $  \n

******************************************************************************/

/* License

 * Copyright (c) 2008-2009, Atmel Corporation All rights reserved.

 *

 * Redistribution and use in source and binary forms, with or without

 * modification, are permitted provided that the following conditions are met:

 *

 * 1. Redistributions of source code must retain the above copyright notice,

 * this list of conditions and the following disclaimer.

 *

 * 2. Redistributions in binary form must reproduce the above copyright notice,

 * this list of conditions and the following disclaimer in the documentation

 * and/or other materials provided with the distribution.

 *

 * 3. The name of ATMEL may not be used to endorse or promote products derived

 * from this software without specific prior written permission.

 *

 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED

 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF

 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND

 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,

 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES

 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;

 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND

 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT

 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF

 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

/******************************************************************************

*

* Revision History

*-----------------*

* Author   Date        Description of changes

* rsk                  Changes done to QMatrix Application Code.

*/
/******************************************************************************/

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

/*----------------------------------------------------------------------------
  manifest constants
----------------------------------------------------------------------------*/
#define EVK2080B 1u

/* The Debug Interface ( _DEBUG_INTERFACE_ ) Needs to be enabled in the Project
   Options to get the debugging information . DBG_DATA_PORT , DBG_CLK_PORT ,
   DBG_DATA_BIT and DBG_CLK_BIT needs to be changed by the user according to
   their own Debug Related Ports and Pins Usage */
#ifdef _DEBUG_INTERFACE_
    /* Debug information */
    /* Debug related ports */
    #define DBG_DATA_PORT  C
    #define DBG_CLK_PORT   A
    /* Debug related pins */
    #define DBG_DATA_BIT   7u
    #define DBG_CLK_BIT    1u
#endif /*_DEBUG_INTERFACE_*/

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
/* Configure the sensors */
static void config_sensors(void);

#if defined(_ROTOR_SLIDER_)
/* Configure the sensors with rotors/sliders with keys */
static void config_rotor_sliders(void);
/* Configure the sensors for 8 channel Key Rotor/sliders */
static void config_8ch_krs(void);

#else

/* Configure the sensors for  Keys configuration */
static void config_keys(void);
#endif /* _ROTOR_SLIDER_ */

#ifdef _DEBUG_INTERFACE_
/* debug functions */
static void report_debug_data( void );
static void output_to_debugger( uint8_t *p, uint8_t count );
static void send_debug_byte( uint8_t data );
static void init_debug_if( void );
#endif /*_DEBUG_INTERFACE_*/


/*----------------------------------------------------------------------------
  Structure Declarations
----------------------------------------------------------------------------*/
/* board info returned in debug data */

#ifdef _DEBUG_INTERFACE_
typedef struct tag_board_info_t
{

    uint8_t qt_max_num_rotors_sliders_board_id;	/*board ID plus max num rotors/sliders
                                                * bits 0..3: board ID
                                                * bits 4..7: the max number of rotors or sliders supported by the library
                                                */
    uint8_t qt_num_channels;	/* the number of touch channels supported by the library */

} board_info_t;
#endif
/*----------------------------------------------------------------------------
  macros
----------------------------------------------------------------------------*/

#ifdef _DEBUG_INTERFACE_
	/* fill out sensor config info for reporting in debug data */
	#define SENSOR_CONFIG( from, to, type ) ( ( to << 5 ) | ( from << 2 ) | type )
#endif/* _DEBUG_INTERFACE_ */

/*----------------------------------------------------------------------------
  global variables
----------------------------------------------------------------------------*/
/* Timer period in msec. */
uint16_t qt_measurement_period_msec = 25u;
#ifdef _DEBUG_INTERFACE_
	uint8_t sensor_config[QT_NUM_CHANNELS];
	board_info_t board_info;
#endif

/*----------------------------------------------------------------------------
  extern variables
----------------------------------------------------------------------------*/
/* This configuration data structure parameters if needs to be changed will be
   changed in the qt_set_parameters function */
extern qt_touch_lib_config_data_t qt_config_data;
/* measurement data */
extern qt_touch_lib_measure_data_t qt_measure_data;
qt_touch_lib_measure_data_t *pqt_measure_data = &qt_measure_data;
/* Get sensor delta values */
extern int16_t qt_get_sensor_delta( uint8_t sensor);

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
    FILL_OUT_X_LINE_INFO(  1,3u )
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
    FILL_OUT_YA_LINE_INFO(  1u )
};
y_line_info_t yb_line_info[NUM_Y_LINES]= {
    FILL_OUT_YB_LINE_INFO(  0u ),
    FILL_OUT_YB_LINE_INFO(  1u )
};
#endif/*_QMATRIX_*/

/*----------------------------------------------------------------------------
  static variables
----------------------------------------------------------------------------*/

/* flag set by timer ISR when it's time to measure touch */
static volatile uint8_t time_to_measure_touch = 0u;

/* current time, set by timer ISR */
static volatile uint16_t current_time_ms_touch = 0u;

/*============================================================================
Name : main
------------------------------------------------------------------------------
Purpose: main code entry point
Input  : n/a
Output : n/a
Notes  :
============================================================================*/

int main( void )
{
    /*status flags to indicate the re-burst for library*/
    uint16_t status_flag = 0u;
    uint16_t burst_flag = 0u;
    /* initialise host app, pins, watchdog, etc */
    init_system();

	/* Configure the Sensors as keys or Keys With Rotor/Sliders in this function */
	config_sensors();

    /* initialise touch sensing */
    qt_init_sensing();

    /*  Set the parameters like recalibration threshold, Max_On_Duration etc in this function by the user */
    qt_set_parameters( );

    /* configure timer ISR to fire regularly */
    init_timer_isr();

#ifdef _DEBUG_INTERFACE_
    /* configure the debug data reported to the PC */
	board_info.qt_max_num_rotors_sliders_board_id = ( ( QT_MAX_NUM_ROTORS_SLIDERS << 4u ) | EVK2080B );
	board_info.qt_num_channels = QT_NUM_CHANNELS;
#endif


    /* Address to pass address of user functions */
    /* This function is called after the library has made capacitive measurements,
    * but before it has processed them. The user can use this hook to apply filter
    * functions to the measured signal values.(Possibly to fix sensor layout faults) */
    qt_filter_callback = 0;

    /* enable interrupts */
    __enable_interrupt();

/* loop forever */
   for( ; ; )
   {
      if( time_to_measure_touch )
      {
         /*  clear flag: it's time to measure touch  */
         time_to_measure_touch = 0u;

         do {
            /*  one time measure touch sensors    */
            status_flag = qt_measure_sensors( current_time_ms_touch );
            burst_flag = status_flag & QTLIB_BURST_AGAIN;

            /*Time critical host application code goes here*/

         }while (  burst_flag) ;
         #ifdef _DEBUG_INTERFACE_
            /*  report debug data to host   */
            report_debug_data();
         #endif /*_DEBUG_INTERFACE_*/
      }

      /*  Time Non-critical host application code goes here  */

   }
}

/*============================================================================
  Name : qt_set_parameters
------------------------------------------------------------------------------
Purpose: This will fill the default threshold values in the configuration data
         structure.But User can change the values of these parameters .
Input  : n/a
Output : n/a
Notes  : initialize configuration data for processing
============================================================================*/

static void qt_set_parameters( void )
{
    /* This will be modified by the user to different values */
    qt_config_data.qt_di              = DEF_QT_DI;
    qt_config_data.qt_neg_drift_rate  = DEF_QT_NEG_DRIFT_RATE;
    qt_config_data.qt_pos_drift_rate  = DEF_QT_POS_DRIFT_RATE;
    qt_config_data.qt_max_on_duration = DEF_QT_MAX_ON_DURATION;
    qt_config_data.qt_drift_hold_time = DEF_QT_DRIFT_HOLD_TIME;
    qt_config_data.qt_recal_threshold = DEF_QT_RECAL_THRESHOLD;
    qt_config_data.qt_pos_recal_delay = DEF_QT_POS_RECAL_DELAY;
}

/*============================================================================
  Name : config_sensors
------------------------------------------------------------------------------
Purpose: Configure the sensors
Input  : n/a
Output : n/a
Notes  : n/a
============================================================================*/
static void config_sensors(void)
{
#if defined(_ROTOR_SLIDER_)
	config_rotor_sliders();
#else /* !_ROTOR_SLIDER_ OR ONLY KEYS*/
	config_keys();
#endif /* _ROTOR_SLIDER_ */
}
/*============================================================================
  Name : config_keys
------------------------------------------------------------------------------
Purpose: Configure the sensors as keys only
Input  : n/a
Output : n/a
Notes  : n/a
============================================================================*/
#ifndef _ROTOR_SLIDER_
static void config_keys(void)
{
	/* enable sensors 0..3: keys on channels 0..3 */
    qt_enable_key( CHANNEL_0, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_1, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_2, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_3, AKS_GROUP_1, 10u, HYST_6_25 );
	/* enable sensors 4..7: keys on channels 4..7 */
	qt_enable_key( CHANNEL_4, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_5, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_6, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_7, AKS_GROUP_1, 10u, HYST_6_25 );

#ifdef _DEBUG_INTERFACE_
    /* configure the debug data reported to the PC */
    sensor_config[0] = SENSOR_CONFIG( CHANNEL_0, CHANNEL_0, SENSOR_TYPE_KEY );
    sensor_config[1] = SENSOR_CONFIG( CHANNEL_1, CHANNEL_1, SENSOR_TYPE_KEY );
    sensor_config[2] = SENSOR_CONFIG( CHANNEL_2, CHANNEL_2, SENSOR_TYPE_KEY );
    sensor_config[3] = SENSOR_CONFIG( CHANNEL_3, CHANNEL_3, SENSOR_TYPE_KEY );
    sensor_config[4] = SENSOR_CONFIG( CHANNEL_4, CHANNEL_4, SENSOR_TYPE_KEY );
    sensor_config[5] = SENSOR_CONFIG( CHANNEL_5, CHANNEL_5, SENSOR_TYPE_KEY );
    sensor_config[6] = SENSOR_CONFIG( CHANNEL_6, CHANNEL_6, SENSOR_TYPE_KEY );
    sensor_config[7] = SENSOR_CONFIG( CHANNEL_7, CHANNEL_7, SENSOR_TYPE_KEY );

#endif/* _DEBUG_INTERFACE_ */
}
#endif/* _ROTOR_SLIDER_ */
/*============================================================================
  Name : config_rotor_sliders
------------------------------------------------------------------------------
Purpose: Configure the Sensors as keys and also as Rotors/Sliders
Input  : n/a
Output : n/a
Notes  : n/a
============================================================================*/
#if defined(_ROTOR_SLIDER_)
static void config_rotor_sliders(void)
{
	/* Call this function if library used is 8 channel library with KRS Configuration */
    config_8ch_krs();
}

/*============================================================================
  Name : config_8ch_krs
------------------------------------------------------------------------------
Purpose: Configure the Sensors as keys and Rotor/Sliders for 8 channels only
Input  : n/a
Output : n/a
Notes  : n/a
============================================================================*/
static void config_8ch_krs(void)
{
	/* enable sensor 0: a rotor on channels 0..3 */
	qt_enable_rotor( CHANNEL_0, CHANNEL_3, AKS_GROUP_1, 16u, HYST_6_25, RES_8_BIT, 0u  );

	/* enable sensors 1: wlider on channels 4..7 respectively */
	qt_enable_slider( CHANNEL_4, CHANNEL_7, AKS_GROUP_1, 16u, HYST_6_25, RES_8_BIT, 0u  );

#ifdef _DEBUG_INTERFACE_
	/* configure the debug data reported to the PC */
	sensor_config[0] = SENSOR_CONFIG( CHANNEL_0, CHANNEL_3, SENSOR_TYPE_ROTOR );
	sensor_config[1] = SENSOR_CONFIG( CHANNEL_4, CHANNEL_7, SENSOR_TYPE_SLIDER );
#endif/* _DEBUG_INTERFACE_ */

}
#endif /* _ROTOR_SLIDER_ */

#ifdef _DEBUG_INTERFACE_
/*============================================================================
  Name : report_debug_data
------------------------------------------------------------------------------
Purpose: report debug data to host
Input  : n/a
Output : n/a
Notes  : n/a
============================================================================*/

static void report_debug_data( void )
{
    uint8_t zero = 0;
    int8_t i;
    int16_t sensor_delta;

    output_to_debugger( (uint8_t *) &board_info, (uint8_t) sizeof( board_info ) );
    output_to_debugger( (uint8_t *) &qt_measure_data.channel_signals[0], (uint8_t) sizeof( qt_measure_data.channel_signals ) );
    output_to_debugger( (uint8_t *) &qt_measure_data.channel_references[0], (uint8_t) sizeof( qt_measure_data.channel_references ) );
    for( i = 0u; i < QT_NUM_CHANNELS; i++ )
    {
        sensor_delta = qt_get_sensor_delta( i );
        output_to_debugger( (uint8_t *) &sensor_delta, sizeof( int16_t ) );
    }
    output_to_debugger( (uint8_t *) &qt_measure_data.qt_touch_status, (uint8_t) sizeof( qt_measure_data.qt_touch_status.sensor_states ) );
    for( i = 0u; i < QT_MAX_NUM_ROTORS_SLIDERS; i++ )
    {
        output_to_debugger( (uint8_t *) &qt_measure_data.qt_touch_status.rotor_slider_values[i], sizeof( uint8_t ) );
        output_to_debugger( (uint8_t *) &zero, sizeof( uint8_t ) );
    }
    output_to_debugger( (uint8_t *) &sensor_config[0], (uint8_t) sizeof( sensor_config ) );
}


/*============================================================================
  Name : output_to_debugger
------------------------------------------------------------------------------
Purpose: transmit multiple bytes over the debug interface
Input  : p = ptr to bytes to transmit
         count = number of bytes to transmit
Output : n/a
Notes  :
============================================================================*/

static void output_to_debugger( uint8_t *p, uint8_t count )
{
    uint8_t i;
    uint8_t data;

    for( i = 0u; i < count; i++ )
    {
        /* get next byte to transmit */
        data = *p;

        /* transmit a byte over the debug interface */
        send_debug_byte( data );

        /* point to next byte to transmit */
        p++;
    }
}
#endif /*_DEBUG_INTERFACE_*/


/*============================================================================
  Name : init_timer_isr
------------------------------------------------------------------------------
Purpose: configure timer ISR to fire regularly
Input  : n/a
Output : n/a
Notes  :
============================================================================*/

static void init_timer_isr( void )
{
    /* set timer compare value (how often timer ISR will fire) */
    OCR1A = ( TICKS_PER_MS * qt_measurement_period_msec);

    /* enable timer ISR */
    TIMSK1 |= (1u << OCIE1A);

    /* timer prescaler = system clock / 8 */
    TCCR1B |= (1u << CS11);

    /* timer mode = CTC (count up to compare value, then reset) */
    TCCR1B |= (1u << WGM12);
}


/*============================================================================
  Name : init_system
------------------------------------------------------------------------------
Purpose: initialise host app, pins, watchdog, etc
Input  : n/a
Output : n/a
Notes  :
============================================================================*/

static void init_system( void )
{
    /* run at 4MHz */
    CLKPR = 0x80u;
    CLKPR = 0x01u;

    /* disable pull-ups */
    MCUCR |= (1u << PUD);

#ifdef _DEBUG_INTERFACE_
    /* inilize debug ports */
    init_debug_if( );
#endif /*_DEBUG_INTERFACE_*/
}

/*============================================================================
  Name : timer_isr
------------------------------------------------------------------------------
Purpose: timer 1 compare ISR
Input  : n/a
Output : n/a
Notes  :
============================================================================*/
ISR(TIMER1_COMPA_vect)
{
    /* set flag: it's time to measure touch */
    time_to_measure_touch = 1u;

    /* update the current time */
    current_time_ms_touch += qt_measurement_period_msec;
}

#ifdef _DEBUG_INTERFACE_
/*============================================================================
  Name : init_debug_if
------------------------------------------------------------------------------
Purpose: report debug data to host
Input  : n/a
Output : n/a
Notes  : n/a
============================================================================*/

static void init_debug_if( void )
{
    /* init port pins */
    REG( DDR, DBG_DATA_PORT ) |= (1u << DBG_DATA_BIT );
    REG( DDR, DBG_CLK_PORT ) |= (1u << DBG_CLK_BIT );
}

/*============================================================================
  Name : send_debug_byte
------------------------------------------------------------------------------
Purpose: transmit a byte over the debug interface
Input  : data = byte to be transmitted
Output : n/a
Notes  :
============================================================================*/

static void send_debug_byte( uint8_t data )
{
    uint8_t i;

    for( i = 0u; i < 8u; i++ )
    {
        /* set data */
        if( data & 0x80u )
        {
            REG( PORT, DBG_DATA_PORT ) |= (1u << DBG_DATA_BIT);
        }
        else
        {
            REG( PORT, DBG_DATA_PORT ) &= ~(1u << DBG_DATA_BIT);
        }

        /* data set up time before clock pulse */
        __delay_cycles( 10UL );

        /* shift next bit up, ready for output */
        data = (uint8_t)( data << 1u );

        /* clock pulse */
        REG( PORT, DBG_CLK_PORT ) |= (1u << DBG_CLK_BIT );

        __delay_cycles( 10UL );

        REG( PORT, DBG_CLK_PORT ) &= ~(1u << DBG_CLK_BIT );

      /* delay before next bit */
        __delay_cycles( 10UL );
    }

    /* hold data low between bytes */
    REG( PORT, DBG_DATA_PORT ) &= ~(1u << DBG_DATA_BIT );

    /* inter-byte delay */
    __delay_cycles( 50UL );
}

#endif /*_DEBUG_INTERFACE_*/


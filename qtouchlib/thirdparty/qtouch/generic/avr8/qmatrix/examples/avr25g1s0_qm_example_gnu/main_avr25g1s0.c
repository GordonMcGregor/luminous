/*******************************************************************************
*   Atmel Corporation:  http://www.atmel.com
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
#define __delay_cycles(n)     __builtin_avr_delay_cycles(n)
#define __enable_interrupt()  sei()
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
/*  initialise host app, pins, watchdog, etc    */
static void init_system( void );
/*  configure timer ISR to fire regularly   */
static void init_timer_isr( void );
/*  Assign the parameters values to global configuration parameter structure    */
static void qt_set_parameters( void );
/*  Configure the sensors */
static void config_sensors(void);

#if defined(_ROTOR_SLIDER_)
   /*  Configure the sensors with rotors/sliders with keys */
   static void config_rotor_sliders(void);
   #if (QT_NUM_CHANNELS == 4u)
      /*  Configure the sensors for 4 channel Key Rotor/sliders   */
      static void config_4ch_krs(void);
   #endif
   #if (QT_NUM_CHANNELS == 8u)
      /*  Configure the sensors for 8 channel Key Rotor/sliders   */
      static void config_8ch_krs(void);
   #endif

#else
   /*  Configure the sensors for  Keys configuration */
   static void config_keys(void);
#endif /* _ROTOR_SLIDER_ */

/*----------------------------------------------------------------------------
Structure Declarations
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
macros
----------------------------------------------------------------------------*/
#define GET_SENSOR_STATE(SENSOR_NUMBER) qt_measure_data.qt_touch_status.sensor_states[(SENSOR_NUMBER/8)] & (1 << (SENSOR_NUMBER % 8))
#define GET_ROTOR_SLIDER_POSITION(ROTOR_SLIDER_NUMBER) qt_measure_data.qt_touch_status.rotor_slider_values[ROTOR_SLIDER_NUMBER]

#ifdef TICKS_PER_MS
#undef TICKS_PER_MS
#define TICKS_PER_MS                8u
#endif
/*----------------------------------------------------------------------------
global variables
----------------------------------------------------------------------------*/
/* Timer period in msec. */
uint16_t qt_measurement_period_msec = 25u;

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

   #if (NUM_X_LINES==4u)
      x_line_info_t x_line_info[NUM_X_LINES]= {
         FILL_OUT_X_LINE_INFO(  1,0u ),
         FILL_OUT_X_LINE_INFO(  1,3u ),
         FILL_OUT_X_LINE_INFO(  1,4u ),
         FILL_OUT_X_LINE_INFO(  1,5u )
      };
   #endif


   /* Fill out the Y-Line masks on the Y- Line port selected
   * The order of Y - Line numbering follows from the way the
   * Y-Lines are filled as below
   * Here, Y0,Y1,Y2,Y3 on 0,1,2,3
   * Note: 1. The Number entries should be based on NUM_X_LINES
   *          2 entries when NUM_Y_LINES=2
   *          4 entries when NUM_Y_LINES=4
   *          8 entries when NUM_Y_LINES=8
   */
   #if (NUM_Y_LINES==1u)
      y_line_info_t ya_line_info[NUM_Y_LINES]= {
         FILL_OUT_YA_LINE_INFO(  2u )
      };
	  y_line_info_t yb_line_info[NUM_Y_LINES]= {
         FILL_OUT_YB_LINE_INFO(  2u )
      };
   #endif
   #if (NUM_Y_LINES==2u)
      y_line_info_t ya_line_info[NUM_Y_LINES]= {
         FILL_OUT_YA_LINE_INFO(  2u ),
         FILL_OUT_YA_LINE_INFO(  0u )
      };
	  y_line_info_t yb_line_info[NUM_Y_LINES]= {
         FILL_OUT_YB_LINE_INFO(  2u ),
         FILL_OUT_YB_LINE_INFO(  0u )
      };
   #endif

#endif/*_QMATRIX_*/

/*----------------------------------------------------------------------------
static variables
----------------------------------------------------------------------------*/
/* flag set by timer ISR when it's time to measure touch */
static volatile uint8_t time_to_measure_touch = 0u;
/* current time, set by timer ISR */
static volatile uint16_t current_time_ms_touch = 0u;

/*============================================================================
Name    :   main
------------------------------------------------------------------------------
Purpose :   main code entry point
Input   :
Output  :
Notes   :
============================================================================*/
int main( void )
{

  /*
    STK 600 Pin Connections
  
    4x line 1y line combination

    Port A Pin 0    X0   
    Port A Pin 1    AIN0 connected to GND
    Port A Pin 2    YB0
    Port A Pin 3    X1
    Port A Pin 4    X2
    Port A Pin 5    X3
    Port A Pin 6    DBG_DATA_PORT
    Port A Pin 7    DBG_CLK_PORT
    
    Port B Pin 1    SMP_PIN
    Port B Pin 2    YA0
  */
  
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

   /*  Address to pass address of user functions   */
   /*  This function is called after the library has made capacitive measurements,
   *   but before it has processed them. The user can use this hook to apply filter
   *   functions to the measured signal values.(Possibly to fix sensor layout faults)    */
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

         }while(burst_flag);
      }

      /*  Time Non-critical host application code goes here  */

   }
}

/*============================================================================
Name    :   qt_set_parameters
------------------------------------------------------------------------------
Purpose :   This will fill the default threshold values in the configuration
data structure.But User can change the values of these parameters .
Input   :
Output  :
Notes   :   initialize configuration data for processing
============================================================================*/
static void qt_set_parameters( void )
{
   qt_config_data.qt_di                     = DEF_QT_DI;
   qt_config_data.qt_neg_drift_rate         = DEF_QT_NEG_DRIFT_RATE;
   qt_config_data.qt_pos_drift_rate         = DEF_QT_POS_DRIFT_RATE;
   qt_config_data.qt_max_on_duration        = DEF_QT_MAX_ON_DURATION;
   qt_config_data.qt_drift_hold_time        = DEF_QT_DRIFT_HOLD_TIME;
   qt_config_data.qt_recal_threshold        = DEF_QT_RECAL_THRESHOLD;
   qt_config_data.qt_pos_recal_delay        = DEF_QT_POS_RECAL_DELAY;
}

/*============================================================================
Name    :   config_sensors
------------------------------------------------------------------------------
Purpose :   Configure the sensors
Input   :
Output  :
Notes   :
============================================================================*/
static void config_sensors(void)
{
   #if defined(_ROTOR_SLIDER_)
      config_rotor_sliders();
   #else /*    !_ROTOR_SLIDER_ OR ONLY KEYS    */
      config_keys();
   #endif /*   _ROTOR_SLIDER_  */
}
/*============================================================================
Name    :   config_keys
------------------------------------------------------------------------------
Purpose :   Configure the sensors as keys only
Input   :
Output  :
Notes   :
============================================================================*/
#ifndef _ROTOR_SLIDER_
static void config_keys(void)
{
   qt_enable_key( CHANNEL_0, AKS_GROUP_1, 10u, HYST_6_25 );
   qt_enable_key( CHANNEL_1, AKS_GROUP_1, 10u, HYST_6_25 );
   qt_enable_key( CHANNEL_2, AKS_GROUP_1, 10u, HYST_6_25 );
   qt_enable_key( CHANNEL_3, AKS_GROUP_1, 10u, HYST_6_25 );

   #if(QT_NUM_CHANNELS >= 8u)
      qt_enable_key( CHANNEL_4, AKS_GROUP_1, 10u, HYST_6_25 );
      qt_enable_key( CHANNEL_5, AKS_GROUP_1, 10u, HYST_6_25 );
      qt_enable_key( CHANNEL_6, AKS_GROUP_1, 10u, HYST_6_25 );
      qt_enable_key( CHANNEL_7, AKS_GROUP_1, 10u, HYST_6_25 );
   #endif
}
#endif/*    _ROTOR_SLIDER_  */
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
   #if (QT_NUM_CHANNELS == 4u)
      {
         config_4ch_krs();
      }
   #endif
   #if (QT_NUM_CHANNELS == 8u)
      {
         config_8ch_krs();
      }
   #endif
}

/*============================================================================
Name    :   config_4ch_krs
------------------------------------------------------------------------------
Purpose :   Configure the Sensors as keys and Rotor/Sliders for 4 channels only
Input   :   n/a
Output  :   n/a
Notes   :   n/a
============================================================================*/
#if (QT_NUM_CHANNELS == 4u)
static void config_4ch_krs(void)
{
   qt_enable_key( CHANNEL_0, AKS_GROUP_2, 10u, HYST_6_25 );
   qt_enable_slider( CHANNEL_1, CHANNEL_3, NO_AKS_GROUP, 16u, HYST_6_25, RES_8_BIT, 0u  );
}
#endif /*QT_NUM_CHANNELS == 4u config_4ch_krs */

/*============================================================================
Name    :   config_8ch_krs
------------------------------------------------------------------------------
Purpose :   Configure the Sensors as keys and Rotor/Sliders for 8 channels only
Input   :   n/a
Output  :   n/a
Notes   :   n/a
============================================================================*/
#if (QT_NUM_CHANNELS == 8u)
static void config_8ch_krs(void)
{
   #if defined(_QTOUCH_)
      qt_enable_key( CHANNEL_0, AKS_GROUP_2, 10u, HYST_6_25 );
      qt_enable_slider( CHANNEL_1, CHANNEL_3, NO_AKS_GROUP, 16u, HYST_6_25, RES_8_BIT, 0u  );
      qt_enable_key( CHANNEL_4, AKS_GROUP_2, 10u, HYST_6_25 );
      qt_enable_rotor( CHANNEL_5, CHANNEL_7, NO_AKS_GROUP, 16u, HYST_6_25, RES_8_BIT, 0u );
   #elif defined(_QMATRIX_)
      qt_enable_rotor( CHANNEL_0, CHANNEL_3, NO_AKS_GROUP, 16u, HYST_6_25, RES_8_BIT, 0u  );
      qt_enable_slider( CHANNEL_4, CHANNEL_7, NO_AKS_GROUP, 16u, HYST_6_25, RES_8_BIT, 0u  );
   #endif/*    _QTOUCH_ / _QMATRIX_    */
}
#endif /*   QT_NUM_CHANNELS == 8u config_8ch_krs    */
#endif /* _ROTOR_SLIDER_ */

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
   OCR1A = ( TICKS_PER_MS * qt_measurement_period_msec);
   /*  enable timer ISR */
   TIMSK1 |= (1u << OCIE1A);
   /*  timer prescaler = system clock / 8  */
   TCCR1B |= (1u << CS11);
   /*  timer mode = CTC (count up to compare value, then reset)    */
   TCCR1B |= (1u << WGM12);
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
   /*  run at 4MHz */
   CLKPR = 0x80u;
   CLKPR = 0x01u;
   /*  disable pull-ups    */
   MCUCR |= (1u << PUD);
}

/*============================================================================
Name    :   timer_isr
------------------------------------------------------------------------------
Purpose :   timer 1 compare ISR
Input   :   n/a
Output  :   n/a
Notes   :
============================================================================*/
ISR(TIM1_COMPA_vect)
{
   /*  set flag: it's time to measure touch    */
   time_to_measure_touch = 1u;
   /*  update the current time */
   current_time_ms_touch += qt_measurement_period_msec;
}

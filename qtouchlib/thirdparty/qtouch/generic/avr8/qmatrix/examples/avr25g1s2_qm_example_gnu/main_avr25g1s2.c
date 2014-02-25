/*******************************************************************************
*   Atmel Corporation:  http://www.atmel.com
*   Support email:  touch@atmel.com
*******************************************************************************/
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

/*------------------------------------------------------------------------------
compiler information
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
include files
------------------------------------------------------------------------------*/
#include <avr/io.h>
#include <avr/interrupt.h>
#define __delay_cycles(n)     __builtin_avr_delay_cycles(n)
#define __enable_interrupt()  sei()

#include "touch_api.h"
#ifdef _DEBUG_INTERFACE_
#include "QDebug.h"
#include "QDebugTransport.h"
#endif/* _DEBUG_INTERFACE_*/
/*------------------------------------------------------------------------------
manifest constants
------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------
type definitions
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
prototypes
------------------------------------------------------------------------------*/
/* initialise host app, pins, watchdog, etc */
static void init_system( void );
/* configure timer ISR to fire regularly */
static void init_timer_isr( void );
/* Assign the parameters values to global configuration parameter structure */
static void qt_set_parameters( void );
/*Configure sensors */
static void config_sensors(void);

#if defined(_ROTOR_SLIDER_)
   static void config_rotor_sliders(void);
   #if (QT_NUM_CHANNELS == 4u)
      static void config_4ch_krs(void);
   #endif
   #if (QT_NUM_CHANNELS == 8u)
      static void config_8ch_krs(void);
   #endif
   #if (QT_NUM_CHANNELS == 12u)
      static void config_12ch_krs(void);
   #endif
   #if (QT_NUM_CHANNELS == 16u)
      static void config_16ch_krs(void);
   #endif
#else
   static void config_keys(void);
#endif /* _ROTOR_SLIDER_ */

/*------------------------------------------------------------------------------
Interrupt prototypes
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
macros
------------------------------------------------------------------------------*/

#define GET_SENSOR_STATE(SENSOR_NUMBER) qt_measure_data.qt_touch_status.sensor_states[(SENSOR_NUMBER/8)] & (1 << (SENSOR_NUMBER % 8))
#define GET_ROTOR_SLIDER_POSITION(ROTOR_SLIDER_NUMBER) qt_measure_data.qt_touch_status.rotor_slider_values[ROTOR_SLIDER_NUMBER]
#ifdef _DEBUG_INTERFACE_
#define TIMER_COUNTER_L TCNT0L
#define TIMER_COUNTER_H 0
#endif/* _DEBUG_INTERFACE_*/

/*------------------------------------------------------------------------------
global variables
------------------------------------------------------------------------------*/
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
/*------------------------------------------------------------------------------
extern variables
------------------------------------------------------------------------------*/
/* This configuration data structure parameters if needs to be changed will be
changed in the qt_set_parameters function */
extern qt_touch_lib_config_data_t qt_config_data;
extern qt_touch_lib_measure_data_t qt_measure_data;
qt_touch_lib_measure_data_t *pqt_measure_data = &qt_measure_data;
extern int16_t qt_get_sensor_delta( uint8_t sensor);

#ifdef _QMATRIX_
   extern y_line_info_t ya_line_info[NUM_Y_LINES];
   extern y_line_info_t yb_line_info[NUM_Y_LINES];
   extern x_line_info_t x_line_info[NUM_X_LINES];

   #if (NUM_X_LINES==4u)
      x_line_info_t x_line_info[NUM_X_LINES]= {
         FILL_OUT_X_LINE_INFO(1,7u),
         FILL_OUT_X_LINE_INFO(1,1u),
         FILL_OUT_X_LINE_INFO(1,4u),
         FILL_OUT_X_LINE_INFO(1,5u)
      };
   #endif

   #if (NUM_X_LINES==8u)
      x_line_info_t x_line_info[NUM_X_LINES]= {
         FILL_OUT_X_LINE_INFO(1,0u),
         FILL_OUT_X_LINE_INFO(1,1u),
         FILL_OUT_X_LINE_INFO(1,2u),
         FILL_OUT_X_LINE_INFO(1,3u),
         FILL_OUT_X_LINE_INFO(1,4u),
         FILL_OUT_X_LINE_INFO(1,5u),
         FILL_OUT_X_LINE_INFO(1,6u),
         FILL_OUT_X_LINE_INFO(1,7u)
      };
   #endif

   #if (NUM_Y_LINES==1u)
      y_line_info_t ya_line_info[NUM_Y_LINES]= {
         FILL_OUT_YA_LINE_INFO(0u)
      };
	  y_line_info_t yb_line_info[NUM_Y_LINES]= {
         FILL_OUT_YB_LINE_INFO(0u)
      };
   #endif

   #if (NUM_Y_LINES==2u)
      y_line_info_t ya_line_info[NUM_Y_LINES]= {
         FILL_OUT_YA_LINE_INFO(0u),
         FILL_OUT_YA_LINE_INFO(2u)
      };
	  y_line_info_t yb_line_info[NUM_Y_LINES]= {
         FILL_OUT_YB_LINE_INFO(0u),
         FILL_OUT_YB_LINE_INFO(2u)
      };
   #endif

   #if (NUM_Y_LINES==4u)
      y_line_info_t ya_line_info[NUM_Y_LINES]= {
         FILL_OUT_YA_LINE_INFO(0u),
         FILL_OUT_YA_LINE_INFO(1u),
         FILL_OUT_YA_LINE_INFO(2u),
         FILL_OUT_YA_LINE_INFO(3u)
      };
	  y_line_info_t yb_line_info[NUM_Y_LINES]= {
         FILL_OUT_YB_LINE_INFO(0u),
         FILL_OUT_YB_LINE_INFO(1u),
         FILL_OUT_YB_LINE_INFO(2u),
         FILL_OUT_YB_LINE_INFO(3u)
      };
   #endif


#endif /*_QMATRIX_*/

/*------------------------------------------------------------------------------
static variables
------------------------------------------------------------------------------*/
static volatile uint8_t time_to_measure_touch = 0u;
static volatile uint16_t current_time_ms_touch = 0u;


/*==============================================================================
Name    : main
--------------------------------------------------------------------------------
Purpose : main code entry point
Input   :
Output  :
Notes   :
==============================================================================*/
int main( void )
{
  /*
    STK600 pin connection details
  
    4x 2y configuration
  
    Port A Pin 0    YB0
    Port A Pin 1    X1
    Port A Pin 2    YB1
    Port A Pin 3
    Port A Pin 4    X2
    Port A Pin 5    X3
    Port A Pin 6    AIN0 - connected to GND
    Port A Pin 7    X0
  
    Port B Pin 0    YA0
    Port B Pin 1    SMP_PIN
    Port B Pin 2    YA1
    Port B Pin 3    SPI_BB_SS
    Port B Pin 4    SPI_BB_MOSI
    Port B Pin 5    SPI_BB_MISO
    Port B Pin 6    SPI_BB_SCK
    Port B Pin 7
  */
  
  
    /*status flags to indicate the re-burst for library*/
   uint16_t status_flag = 0u;
   uint16_t burst_flag = 0u;

   /* initialise host app, pins, watchdog, etc */
   init_system();
   /* Configure the Sensors as keys or Keys With Rotor/Sliders in this function */
   config_sensors();
   /* Initialise and set touch params */
   qt_init_sensing();
   qt_set_parameters();


   init_timer_isr();
#ifdef _DEBUG_INTERFACE_
    timestamp1_hword = current_time_ms_touch;
    timestamp1_lword = (uint16_t)TIMER_COUNTER_L;
    timestamp1_lword |= (uint16_t)(TIMER_COUNTER_H << 8);
#endif/* _DEBUG_INTERFACE_*/
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

/*==============================================================================
Name    : qt_set_parameters
--------------------------------------------------------------------------------
Purpose : Default touch processing variables
Input   :
Output  :
Notes   :
==============================================================================*/
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

/*==============================================================================
Name    : config_sensors
--------------------------------------------------------------------------------
Purpose : Configure the sensors
Input   :
Output  :
Notes   :
==============================================================================*/
static void config_sensors(void)
{
   #if defined(_ROTOR_SLIDER_)
      config_rotor_sliders();
   #else
      config_keys();
   #endif
}

/*==============================================================================
Name    : config_keys
--------------------------------------------------------------------------------
Purpose : Configure sensors as keys only
Input   :
Output  :
Notes   :
==============================================================================*/
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

   #if(QT_NUM_CHANNELS >= 12u)
      qt_enable_key( CHANNEL_8, AKS_GROUP_1, 10u, HYST_6_25 );
      qt_enable_key( CHANNEL_9, AKS_GROUP_1, 10u, HYST_6_25 );
      qt_enable_key( CHANNEL_10, AKS_GROUP_1, 10u, HYST_6_25 );
      qt_enable_key( CHANNEL_11, AKS_GROUP_1, 10u, HYST_6_25 );
   #endif

   #if (QT_NUM_CHANNELS >= 16u)
      qt_enable_key( CHANNEL_12, AKS_GROUP_1, 10u, HYST_6_25 );
      qt_enable_key( CHANNEL_13, AKS_GROUP_1, 10u, HYST_6_25 );
      qt_enable_key( CHANNEL_14, AKS_GROUP_1, 10u, HYST_6_25 );
      qt_enable_key( CHANNEL_15, AKS_GROUP_1, 10u, HYST_6_25 );
   #endif

}
#endif /*_ROTOR_SLIDER_  */

/*==============================================================================
Name    : config_rotor_sliders
--------------------------------------------------------------------------------
Purpose : Configure Sensors as keys, Rotors and Sliders
Input   :
Output  :
Notes   :
==============================================================================*/
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
   #if (QT_NUM_CHANNELS == 12u)
      {
         config_12ch_krs();
      }
   #endif
   #if (QT_NUM_CHANNELS == 16u)
      {
         config_16ch_krs();
      }
   #endif
}

/*==============================================================================
Name    : config_4ch_krs
--------------------------------------------------------------------------------
Purpose : 4 channel sensor configuration with keys, rotors and sliders
Input   :
Output  :
Notes   :
==============================================================================*/
#if (QT_NUM_CHANNELS == 4u)
static void config_4ch_krs(void)
{
   qt_enable_key( CHANNEL_0, AKS_GROUP_2, 10u, HYST_6_25 );
   qt_enable_slider( CHANNEL_1, CHANNEL_3, NO_AKS_GROUP, 16u, HYST_6_25, RES_8_BIT, 0u  );
}
#endif /*QT_NUM_CHANNELS == 4u*/

/*==============================================================================
Name    : config_8ch_krs
--------------------------------------------------------------------------------
Purpose : 8 channel sensor configuration with keys, rotors and sliders
Input   :
Output  :
Notes   :
==============================================================================*/
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

   #endif /*_QTOUCH_*/
}
#endif /* QT_NUM_CHANNELS == 8u */

/*==============================================================================
Name    : config_12ch_krs
--------------------------------------------------------------------------------
Purpose : 12 channel sensor configuration with keys, rotors and sliders
Input   :
Output  :
Notes   :
==============================================================================*/
#if (QT_NUM_CHANNELS == 12u)
static void config_12ch_krs(void)
{
   #if defined(_QTOUCH_)
      qt_enable_key( CHANNEL_0, AKS_GROUP_2, 10u, HYST_6_25 );
      qt_enable_slider( CHANNEL_1, CHANNEL_3, NO_AKS_GROUP, 16u, HYST_6_25, RES_8_BIT, 0u  );
      qt_enable_key( CHANNEL_4, AKS_GROUP_2, 10u, HYST_6_25 );
      qt_enable_rotor( CHANNEL_5, CHANNEL_7, NO_AKS_GROUP, 16u, HYST_6_25, RES_8_BIT, 0u );
      qt_enable_key( CHANNEL_8, AKS_GROUP_2, 10u, HYST_6_25 );
      qt_enable_slider( CHANNEL_9, CHANNEL_11, NO_AKS_GROUP, 16u, HYST_6_25, RES_8_BIT, 0u  );

   #elif defined(_QMATRIX_)
      qt_enable_rotor( CHANNEL_0, CHANNEL_3, NO_AKS_GROUP, 16u, HYST_6_25, RES_8_BIT, 0u  );
      qt_enable_slider( CHANNEL_4, CHANNEL_7, NO_AKS_GROUP, 16u, HYST_6_25, RES_8_BIT, 0u  );
      qt_enable_key( CHANNEL_8, AKS_GROUP_1, 10u, HYST_6_25 );
      qt_enable_key( CHANNEL_9, AKS_GROUP_1, 10u, HYST_6_25 );
      qt_enable_key( CHANNEL_10, AKS_GROUP_1, 10u, HYST_6_25 );
      qt_enable_key( CHANNEL_11, AKS_GROUP_1, 10u, HYST_6_25 );

   #endif /*_QTOUCH_*/
}
#endif /*   QT_NUM_CHANNELS == 12u */

/*==============================================================================
Name    : config_16ch_krs
--------------------------------------------------------------------------------
Purpose : 16 channel sensor configuration with keys, rotors and sliders
Input   :
Output  :
Notes   :
==============================================================================*/
#if (QT_NUM_CHANNELS == 16u)
static void config_16ch_krs(void)
{
   #if defined(_QTOUCH_)
      qt_enable_key( CHANNEL_0, AKS_GROUP_1, 10u, HYST_6_25 );
      qt_enable_key( CHANNEL_1, AKS_GROUP_1, 10u, HYST_6_25 );
      qt_enable_key( CHANNEL_2, AKS_GROUP_1, 10u, HYST_6_25 );
      qt_enable_key( CHANNEL_3, AKS_GROUP_1, 10u, HYST_6_25 );
      qt_enable_key( CHANNEL_4, AKS_GROUP_1, 10u, HYST_6_25 );
      qt_enable_key( CHANNEL_5, AKS_GROUP_1, 10u, HYST_6_25 );
      qt_enable_key( CHANNEL_6, AKS_GROUP_1, 10u, HYST_6_25 );
      qt_enable_key( CHANNEL_7, AKS_GROUP_1, 10u, HYST_6_25 );
      qt_enable_rotor( CHANNEL_8, CHANNEL_10, NO_AKS_GROUP, 16u, HYST_6_25, RES_8_BIT, 0u  );
      qt_enable_slider( CHANNEL_11, CHANNEL_13, NO_AKS_GROUP, 16u, HYST_6_25, RES_8_BIT, 0u  );
      qt_enable_key( CHANNEL_14, AKS_GROUP_1, 10u, HYST_6_25 );
      qt_enable_key( CHANNEL_15, AKS_GROUP_1, 10u, HYST_6_25 );

   #elif defined(_QMATRIX_)
      qt_enable_key( CHANNEL_0, AKS_GROUP_1, 10u, HYST_6_25 );
      qt_enable_key( CHANNEL_1, AKS_GROUP_1, 10u, HYST_6_25 );
      qt_enable_key( CHANNEL_2, AKS_GROUP_1, 10u, HYST_6_25 );
      qt_enable_key( CHANNEL_3, AKS_GROUP_1, 10u, HYST_6_25 );
      qt_enable_key( CHANNEL_4, AKS_GROUP_1, 10u, HYST_6_25 );
      qt_enable_key( CHANNEL_5, AKS_GROUP_1, 10u, HYST_6_25 );
      qt_enable_key( CHANNEL_6, AKS_GROUP_1, 10u, HYST_6_25 );
      qt_enable_key( CHANNEL_7, AKS_GROUP_1, 10u, HYST_6_25 );
      qt_enable_rotor( CHANNEL_8, CHANNEL_11, NO_AKS_GROUP, 16u, HYST_6_25, RES_8_BIT, 0u  );
      qt_enable_slider( CHANNEL_12, CHANNEL_15, NO_AKS_GROUP, 16u, HYST_6_25, RES_8_BIT, 0u  );

   #endif /*_QTOUCH_*/
}
#endif  /*  QT_NUM_CHANNELS == 16u */


#endif /* _ROTOR_SLIDER_ */

/*==============================================================================
Name    : init_timer_isr
--------------------------------------------------------------------------------
Purpose : configure timer ISR to interrupt regularly
Input   :
Output  :
Notes   :
==============================================================================*/
static void init_timer_isr( void )
{
   /*  set timer compare value (how often timer ISR will fire) */
   OCR1A  = (TICKS_PER_MS * qt_measurement_period_msec);
   /*  enable timer ISR */
   TIMSK  = (1u << OCIE1A);
   /*  timer prescaler = system clock / 8  */
   TCCR1B = (1u << CS11);
   /*  timer mode = CTC (count up to compare value, then reset)    */
   TCCR1D = (1u << WGM11);

}

/*==============================================================================
Name    : init_system
--------------------------------------------------------------------------------
Purpose : initialise host app, pins, watchdog, etc
Input   :
Output  :
Notes   :
==============================================================================*/
static void init_system( void )
{
   CLKPR = 0x80u;
   CLKPR = 0x01u; /* Run at 4 MHz */
   MCUCR |= (1u << PUD);

}

/*==============================================================================
Name    : timer_isr
--------------------------------------------------------------------------------
Purpose : timer 1 compare ISR
Input   :
Output  :
Notes   :
==============================================================================*/
ISR(TIMER1_COMPA_vect)
{
   time_to_measure_touch = 1u;
   current_time_ms_touch += qt_measurement_period_msec;
}
/*=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-EOF-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/
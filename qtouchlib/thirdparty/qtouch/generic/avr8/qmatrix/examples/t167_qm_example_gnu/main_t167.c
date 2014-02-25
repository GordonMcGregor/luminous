/*******************************************************************************
*   $FILE:  main.c
*   Atmel Corporation:  http://www.atmel.com \n
*   Support email:  avr@atmel.com
******************************************************************************/

/*  License
*   Copyright (c) 2008-2009, Atmel Corporation All rights reserved.
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
/*  Configure the sensors */
static void config_sensors(void);

#if defined(_ROTOR_SLIDER_)

/*  Configure the sensors with rotors/sliders with keys */
static void config_rotor_sliders(void);

    #if (QT_NUM_CHANNELS == 8u)
        /*  Configure the sensors for 8 channel Key Rotor/sliders   */
        static void config_8ch_krs(void);
    #endif
    #if (QT_NUM_CHANNELS == 16u)
        /*  Configure the sensors for 16 channel Key Rotor/sliders  */
        static void config_16ch_krs(void);
    #endif
    #if (QT_NUM_CHANNELS == 32u)
        /*  Configure the sensors for 32 channel Key Rotor/sliders  */
        static void config_32ch_krs(void);
    #endif
    #if (QT_NUM_CHANNELS == 64u)
        /*  Configure the sensors for 64 channel Key Rotor/sliders  */
        static void config_64ch_krs(void);
    #endif

#else
    /*  Configure the sensors for  Keys configuration */
    static void config_keys(void);
#endif /* _ROTOR_SLIDER_ */


/*----------------------------------------------------------------------------
                                    macros
----------------------------------------------------------------------------*/

#define GET_SENSOR_STATE(SENSOR_NUMBER) qt_measure_data.qt_touch_status.sensor_states[(SENSOR_NUMBER/8)] & (1 << (SENSOR_NUMBER % 8))
#define GET_ROTOR_SLIDER_POSITION(ROTOR_SLIDER_NUMBER) qt_measure_data.qt_touch_status.rotor_slider_values[ROTOR_SLIDER_NUMBER]
#ifdef _DEBUG_INTERFACE_
#define TIMER_COUNTER_L TCNT0
#define TIMER_COUNTER_H 0
#endif

/*----------------------------------------------------------------------------
                                global variables
----------------------------------------------------------------------------*/
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
    FILL_OUT_X_LINE_INFO(  1,2u ),
    FILL_OUT_X_LINE_INFO(  1,3u ),
    FILL_OUT_X_LINE_INFO(  1,4u ),
    FILL_OUT_X_LINE_INFO(  1,5u ),
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
};
y_line_info_t yb_line_info[NUM_Y_LINES]= {
    FILL_OUT_YB_LINE_INFO(  0u ),
    FILL_OUT_YB_LINE_INFO(  1u ),
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
Name    :   main
------------------------------------------------------------------------------
Purpose :   main code entry point
Input   :   n/a
Output  :   n/a
Notes   :
============================================================================*/

int main( void )
{

/*
    STK600 Connections for testing:

	4 X lines and 2 Y lines combination.

    Port A Pin 0	YB0
    Port A Pin 1	YB1
    Port A Pin 2	MOSI_BB
    Port A Pin 3	MISO_BB
    Port A Pin 4	SCK_BB
    Port A Pin 5	SS_BB
    Port A Pin 6    AIN0 -  Connect to GND.

    Port B Pin 0	YA0
    Port B Pin 1	YA1
    Port B Pin 2	X0
    Port B Pin 3	X1
    Port B Pin 4	X2
    Port B Pin 5	X3
    Port B Pin 6	SMP_PIN
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
#ifdef _DEBUG_INTERFACE_
		QDebug_Init();
#endif
    /* enable interrupts */
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
         /*  clear flag: it's time to measure touch  */
         time_to_measure_touch = 0u;

         do {
            /*  one time measure touch sensors    */
            status_flag = qt_measure_sensors( current_time_ms_touch );
            burst_flag = status_flag & QTLIB_BURST_AGAIN;

#ifdef _DEBUG_INTERFACE_
			/* send debug data */
			 QDebug_SendData(status_flag);
#endif

            /*Time critical host application code goes here*/

         }while (  burst_flag) ;
      }

      /*  Time Non-critical host application code goes here  */
#ifdef _DEBUG_INTERFACE_
    /* Process commands from PC */
            QDebug_ProcessCommands();
#endif
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
    /*  This will be modified by the user to different values   */
    qt_config_data.qt_di              = DEF_QT_DI;
    qt_config_data.qt_neg_drift_rate  = DEF_QT_NEG_DRIFT_RATE;
    qt_config_data.qt_pos_drift_rate  = DEF_QT_POS_DRIFT_RATE;
    qt_config_data.qt_max_on_duration = DEF_QT_MAX_ON_DURATION;
    qt_config_data.qt_drift_hold_time = DEF_QT_DRIFT_HOLD_TIME;
    qt_config_data.qt_recal_threshold = DEF_QT_RECAL_THRESHOLD;
    qt_config_data.qt_pos_recal_delay = DEF_QT_POS_RECAL_DELAY;
}

/*============================================================================
Name    :   config_sensors
------------------------------------------------------------------------------
Purpose :   Configure the sensors
Input   :   n/a
Output  :   n/a
Notes   :   n/a
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
Input   :   n/a
Output  :   n/a
Notes   :   n/a
============================================================================*/
#ifndef _ROTOR_SLIDER_
static void config_keys(void)
{
    /*  enable sensors 0..3: keys on channels 0..3  */
    qt_enable_key( CHANNEL_0, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_1, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_2, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_3, AKS_GROUP_1, 10u, HYST_6_25 );

#if(QT_NUM_CHANNELS >= 8u)
    /*  enable sensors 4..7: keys on channels 4..7   */
    qt_enable_key( CHANNEL_4, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_5, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_6, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_7, AKS_GROUP_1, 10u, HYST_6_25 );
#endif
#if (QT_NUM_CHANNELS >= 16u)
    /*  enable sensors 8..15: keys on channels 8..15    */
    qt_enable_key( CHANNEL_8, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_9, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_10, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_11, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_12, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_13, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_14, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_15, AKS_GROUP_1, 10u, HYST_6_25 );
#endif
#if (QT_NUM_CHANNELS >= 32u)
    /*  enable sensors 16..31: keys on channels 16..31  */
    qt_enable_key( CHANNEL_16, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_17, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_18, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_19, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_20, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_21, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_22, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_23, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_24, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_25, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_26, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_27, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_28, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_29, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_30, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_31, AKS_GROUP_1, 10u, HYST_6_25 );
#endif
#if (QT_NUM_CHANNELS >= 64u)
    /*  enable sensors 32..63: keys on channels 32..63  */
    qt_enable_key( CHANNEL_32, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_33, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_34, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_35, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_36, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_37, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_38, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_39, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_40, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_41, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_42, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_43, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_44, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_45, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_46, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_47, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_48, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_49, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_50, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_51, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_52, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_53, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_54, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_55, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_56, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_57, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_58, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_59, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_60, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_61, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_62, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_63, AKS_GROUP_1, 10u, HYST_6_25 );
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
#if (QT_NUM_CHANNELS == 8u)
    {
        /*  Call this function if library used is 8 channel library with KRS Configuration  */
        config_8ch_krs();
    }
#endif
#if (QT_NUM_CHANNELS == 16u)
    {
        /*  Call this function if library used is 16 channel library with KRS Configuration */
        config_16ch_krs();
    }
#endif
#if (QT_NUM_CHANNELS == 32u)
    {
        /*  Call this function if library used is 32 channel library with KRS Configuration */
        config_32ch_krs();
    }
#endif
#if (QT_NUM_CHANNELS == 64u)
    {
        /*  Call this function if library used is 64 channel library with KRS Configuration */
        config_64ch_krs();
    }
#endif
}


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
    /*  enable sensor 0: a rotor on channels 0..3   */
    qt_enable_rotor( CHANNEL_0, CHANNEL_3, NO_AKS_GROUP, 16u, HYST_6_25, RES_8_BIT, 0u  );

    /*  enable sensors 1: wlider on channels 4..7 respectively  */
    qt_enable_slider( CHANNEL_4, CHANNEL_7, NO_AKS_GROUP, 16u, HYST_6_25, RES_8_BIT, 0u  );

}
#endif /*   QT_NUM_CHANNELS == 8u config_8ch_krs    */

/*============================================================================
Name    :   config_16ch_krs
------------------------------------------------------------------------------
Purpose :   Configure the Sensors as keys and Rotor/Sliders for 16 channels only
Input   :   n/a
Output  :   n/a
Notes   :   n/a
============================================================================*/
#if (QT_NUM_CHANNELS == 16u)
static void config_16ch_krs(void)
{
    /*  enable sensors 0..7: keys on channels 0..7  */
    qt_enable_key( CHANNEL_0, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_1, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_2, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_3, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_4, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_5, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_6, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_7, AKS_GROUP_1, 10u, HYST_6_25 );
    /*  enable sensor 8: a rotor on channels 8..11  */
    qt_enable_rotor( CHANNEL_8, CHANNEL_11, NO_AKS_GROUP, 16u, HYST_6_25, RES_8_BIT, 0u  );
    /*  enable sensors 9: sliders on channels 12..15 respectively   */
    qt_enable_slider( CHANNEL_12, CHANNEL_15, NO_AKS_GROUP, 16u, HYST_6_25, RES_8_BIT, 0u  );

}
#endif  /*  QT_NUM_CHANNELS == 16u config_16ch_krs  */

/*============================================================================
Name    :   config_32ch_krs
------------------------------------------------------------------------------
Purpose :   Configure the Sensors as keys and Rotor/Sliders for 32 channels only
Input   :   n/a
Output  :   n/a
Notes   :   n/a
============================================================================*/
#if (QT_NUM_CHANNELS == 32u)
static void config_32ch_krs(void)
{
        /*  enable sensors 0..7: keys on channels 0..7  */
    qt_enable_key( CHANNEL_0, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_1, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_2, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_3, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_4, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_5, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_6, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_7, AKS_GROUP_1, 10u, HYST_6_25 );
    /*  enable sensors 8..15: keys on channels 8..15    */
    qt_enable_key( CHANNEL_8, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_9, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_10, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_11, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_12, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_13, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_14, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_15, AKS_GROUP_1, 10u, HYST_6_25 );
    /*  enable sensors : rotor on channels  respectively   */
    qt_enable_rotor( CHANNEL_16, CHANNEL_19, NO_AKS_GROUP, 16u, HYST_6_25, RES_8_BIT, 0u  );
    /*  enable sensors : rotor on channels  respectively   */
    qt_enable_rotor( CHANNEL_20, CHANNEL_23, NO_AKS_GROUP, 16u, HYST_6_25, RES_8_BIT, 0u  );
     /*  enable sensors : slider on channels  respectively   */
    qt_enable_slider( CHANNEL_24, CHANNEL_27, NO_AKS_GROUP, 16u, HYST_6_25, RES_8_BIT, 0u  );
     /*  enable sensors : slider on channels  respectively   */
    qt_enable_slider( CHANNEL_28, CHANNEL_31, NO_AKS_GROUP, 16u, HYST_6_25, RES_8_BIT, 0u  );

}
#endif/*    QT_NUM_CHANNELS == 32u config_32ch_krs  */


/*============================================================================
Name    :   config_64ch_krs
------------------------------------------------------------------------------
Purpose :   Configure the Sensors as keys and Rotor/Sliders for 64 channels only
Input   :   n/a
Output  :   n/a
Notes   :   n/a
============================================================================*/
#if (QT_NUM_CHANNELS == 64u)
static void config_64ch_krs(void)
{
    /*  enable sensors 0..7: keys on channels 0..7  */
    qt_enable_key( CHANNEL_0, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_1, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_2, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_3, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_4, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_5, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_6, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_7, AKS_GROUP_1, 10u, HYST_6_25 );
    /*  enable sensor 8: a rotor on channels 8..11  */
    qt_enable_rotor( CHANNEL_8, CHANNEL_11, NO_AKS_GROUP, 16u, HYST_6_25, RES_8_BIT, 0u  );
    /*  enable sensors 9: sliders on channels 12..15 respectively    */
    qt_enable_slider( CHANNEL_12, CHANNEL_15, NO_AKS_GROUP, 16u, HYST_6_25, RES_8_BIT, 0u  );
    /*  enable sensors 10..11: keys on channels 16..17  */
    qt_enable_key( CHANNEL_16, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_17, AKS_GROUP_1, 10u, HYST_6_25 );
    /*  enable sensors 12..13: keys on channels 33..34   */
    qt_enable_key( CHANNEL_33, AKS_GROUP_1, 10u, HYST_6_25 );
    qt_enable_key( CHANNEL_34, AKS_GROUP_1, 10u, HYST_6_25 );

#ifdef _DEBUG_INTERFACE_
    /*  configure the debug data reported to the PC */
    sensor_config[0] = SENSOR_CONFIG( CHANNEL_0, CHANNEL_0, SENSOR_TYPE_KEY );
    sensor_config[1] = SENSOR_CONFIG( CHANNEL_1, CHANNEL_1, SENSOR_TYPE_KEY );
    sensor_config[2] = SENSOR_CONFIG( CHANNEL_2, CHANNEL_2, SENSOR_TYPE_KEY );
    sensor_config[3] = SENSOR_CONFIG( CHANNEL_3, CHANNEL_3, SENSOR_TYPE_KEY );
    sensor_config[4] = SENSOR_CONFIG( CHANNEL_4, CHANNEL_4, SENSOR_TYPE_KEY );
    sensor_config[5] = SENSOR_CONFIG( CHANNEL_5, CHANNEL_5, SENSOR_TYPE_KEY );
    sensor_config[6] = SENSOR_CONFIG( CHANNEL_6, CHANNEL_6, SENSOR_TYPE_KEY );
    sensor_config[7] = SENSOR_CONFIG( CHANNEL_7, CHANNEL_7, SENSOR_TYPE_KEY );
    sensor_config[8] = SENSOR_CONFIG( CHANNEL_8, CHANNEL_11, SENSOR_TYPE_ROTOR );
    sensor_config[9] = SENSOR_CONFIG( CHANNEL_12, CHANNEL_15, SENSOR_TYPE_SLIDER );
    sensor_config[10] = SENSOR_CONFIG( CHANNEL_16, CHANNEL_16, SENSOR_TYPE_KEY );
    sensor_config[11] = SENSOR_CONFIG( CHANNEL_17, CHANNEL_17, SENSOR_TYPE_KEY );
    sensor_config[12] = SENSOR_CONFIG( CHANNEL_33, CHANNEL_33, SENSOR_TYPE_KEY );
    sensor_config[13] = SENSOR_CONFIG( CHANNEL_34, CHANNEL_34, SENSOR_TYPE_KEY );
#endif
}
#endif /*QT_NUM_CHANNELS == 64u config_64ch_krs */
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
 	asm("ldi r16,0x80");
	asm("sts 0x61,r16");

	asm("ldi r16,0x01");
	asm("sts 0x61,r16");

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
ISR(TIMER1_COMPA_vect)
{
    /*  set flag: it's time to measure touch    */
    time_to_measure_touch = 1u;

    /*  update the current time */
    current_time_ms_touch += qt_measurement_period_msec;
}



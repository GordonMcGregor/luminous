/*******************************************************************************
*   touch.c
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
/*  This is an automatically generated file. Do not modify the contents manually.
    Instead, use the Project Builder Wizard to make changes to the design.
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

/*  now include touch api.h with the localization defined above */
#include "touch_api.h"
#include "touch.h"

#ifdef _DEBUG_INTERFACE_
/* Include files for QTouch Studio integration */
#include "QDebug.h"
#include "QDebugTransport.h"
#endif

/*----------------------------------------------------------------------------
                            manifest constants
----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------
                            type definitions
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
                                prototypes
----------------------------------------------------------------------------*/

/*  Assign the parameters values to global configuration parameter structure    */
static void qt_set_parameters( void );
/*  Configure the sensors */
static void config_sensors(void);
/*  Configure burst length  */
static void burst_len_config( void );
#ifdef _DEBUG_INTERFACE_
void set_timestamp1(void); // used for timestamping
#endif


/*----------------------------------------------------------------------------
                            Structure Declarations
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
                                    macros
----------------------------------------------------------------------------*/



#ifdef _DEBUG_INTERFACE_
  #ifdef _QDEBUG_TIME_STAMPS_
    /* This below code is used for timestamping related information */
  
    #define TIMER_COUNTER_L  TCNT0
    #define TIMER_COUNTER_H  0

	#define TIMESTAMP0  asm("cli"); timestamp0_lword = (uint16_t)TIMER_COUNTER_L;timestamp0_lword |= (uint16_t)(TIMER_COUNTER_H << 8); timestamp0_hword = current_time_ms_touch; asm("sei");
	#define TIMESTAMP1  asm("cli"); timestamp1_lword = (uint16_t)TIMER_COUNTER_L;timestamp1_lword |= (uint16_t)(TIMER_COUNTER_H << 8); timestamp1_hword = current_time_ms_touch; asm("sei");
	#define TIMESTAMP2  asm("cli"); timestamp2_lword = (uint16_t)TIMER_COUNTER_L;timestamp2_lword |= (uint16_t)(TIMER_COUNTER_H << 8); timestamp2_hword = current_time_ms_touch; asm("sei");
	#define TIMESTAMP3  asm("cli"); timestamp3_lword = (uint16_t)TIMER_COUNTER_L;timestamp3_lword |= (uint16_t)(TIMER_COUNTER_H << 8); timestamp3_hword = current_time_ms_touch; asm("sei");
	#define TIMESTAMP4  asm("cli"); timestamp4_lword = (uint16_t)TIMER_COUNTER_L;timestamp4_lword |= (uint16_t)(TIMER_COUNTER_H << 8); timestamp4_hword = current_time_ms_touch; asm("sei");
	#define TIMESTAMP5  asm("cli"); timestamp5_lword = (uint16_t)TIMER_COUNTER_L;timestamp5_lword |= (uint16_t)(TIMER_COUNTER_H << 8); timestamp5_hword = current_time_ms_touch; asm("sei");

  #else

    #define TIMESTAMP0   {}
    #define TIMESTAMP1   {}
    #define TIMESTAMP2   {}
    #define TIMESTAMP3   {}
    #define TIMESTAMP4   {}
    #define TIMESTAMP5   {}
  #endif
#endif
/*----------------------------------------------------------------------------
                                global variables
----------------------------------------------------------------------------*/

#ifdef _DEBUG_INTERFACE_
 #ifdef _QDEBUG_TIME_STAMPS_
  uint16_t timestamp0_hword=0;
  uint16_t timestamp0_lword=0;
  uint16_t timestamp1_hword=0;
  uint16_t timestamp1_lword=0;
  uint16_t timestamp2_hword=0;
  uint16_t timestamp2_lword=0;
  uint16_t timestamp3_hword=0;
  uint16_t timestamp3_lword=0;
  uint16_t timestamp4_hword=0;
  uint16_t timestamp4_lword=0;
  uint16_t timestamp5_hword=0;
  uint16_t timestamp5_lword=0;
 #endif
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
   extern y_line_info_t y_line_info[NUM_Y_LINES];
   extern x_line_info_t x_line_info[NUM_X_LINES];

/* Fill out the X-Line masks and  Y-Line masks on the X- Port and Y-Line Port selected.
* The order of X - Line numbering follows from the way the
*
*/


x_line_info_t x_line_info[NUM_X_LINES]= {
FILL_OUT_X_LINE_INFO(1,0),
FILL_OUT_X_LINE_INFO(1,1),
FILL_OUT_X_LINE_INFO(1,2),
FILL_OUT_X_LINE_INFO(1,3),
FILL_OUT_X_LINE_INFO(1,4),
FILL_OUT_X_LINE_INFO(1,5),
FILL_OUT_X_LINE_INFO(1,6),
FILL_OUT_X_LINE_INFO(1,7)};

y_line_info_t ya_line_info[NUM_Y_LINES]={
FILL_OUT_YA_LINE_INFO(0),
FILL_OUT_YA_LINE_INFO(1),
FILL_OUT_YA_LINE_INFO(2),
FILL_OUT_YA_LINE_INFO(3),
FILL_OUT_YA_LINE_INFO(4),
FILL_OUT_YA_LINE_INFO(5),
FILL_OUT_YA_LINE_INFO(6),
FILL_OUT_YA_LINE_INFO(7),
};

y_line_info_t yb_line_info[NUM_Y_LINES]={
FILL_OUT_YB_LINE_INFO(0),
FILL_OUT_YB_LINE_INFO(1),
FILL_OUT_YB_LINE_INFO(2),
FILL_OUT_YB_LINE_INFO(3),
FILL_OUT_YB_LINE_INFO(4),
FILL_OUT_YB_LINE_INFO(5),
FILL_OUT_YB_LINE_INFO(6),
FILL_OUT_YB_LINE_INFO(7),
};


#endif/*_QMATRIX_*/

extern uint8_t time_to_measure_touch;
extern uint16_t current_time_ms_touch;


/*----------------------------------------------------------------------------
                                static variables
----------------------------------------------------------------------------*/

/*============================================================================
Name    :   touch_init
------------------------------------------------------------------------------
Purpose :   This will initialize touch related code.
Input   :   n/a
Output  :   n/a
Notes   :
============================================================================*/

void touch_init( void )
{

    /* Configure the Sensors as keys or Keys With Rotor/Sliders in this function */
    config_sensors();
    
    /*  Configure burst length  */
    burst_len_config();

    /* initialise touch sensing */
    qt_init_sensing();

    /*  Set the parameters like recalibration threshold, Max_On_Duration etc in this function by the user */
    qt_set_parameters( );

    /*  Address to pass address of user functions   */
    /**  This function is called after the library has made capacitive measurements,
      *   but before it has processed them. The user can use this hook to apply filter
      *   functions to the measured signal values.(Possibly to fix sensor layout faults)    
      */
    #ifdef _DEBUG_INTERFACE_
        qt_filter_callback = &set_timestamp1;
    #else
        qt_filter_callback = 0;
    #endif
    
    #ifdef _DEBUG_INTERFACE_
        /* Initialize debug protocol */
        QDebug_Init();

        /* Process commands from PC */
        QDebug_ProcessCommands();
    #endif

}
#ifdef _DEBUG_INTERFACE_
void set_timestamp1(void)
{
	TIMESTAMP1;
}
#endif

/*============================================================================
Name    :   touch_measure
------------------------------------------------------------------------------
Purpose :   This will call all the functions for touch related measurement.
Input   :   n/a
Output  :   n/a
Notes   :
============================================================================*/
void touch_measure()
{
   /*status flags to indicate the re-burst for library*/
   static uint16_t status_flag = 0u;
   static uint16_t burst_flag = 0u;

	  if( time_to_measure_touch )
        {

            /*  clear flag: it's time to measure touch  */
            time_to_measure_touch = 0u;

            do {
				#ifdef _DEBUG_INTERFACE_
				    TIMESTAMP0;
				#endif
                /*  one time measure touch sensors    */
                status_flag = qt_measure_sensors( current_time_ms_touch );

				#ifdef _DEBUG_INTERFACE_
				    TIMESTAMP2;
				#endif
                burst_flag = status_flag & QTLIB_BURST_AGAIN;

				#ifdef _DEBUG_INTERFACE_
                    /* send debug data */
                    QDebug_SendData(status_flag);
				    /* Process commands from PC */
            	    QDebug_ProcessCommands();
				#endif
				
				#ifdef _DEBUG_INTERFACE_
				    TIMESTAMP3;
				#endif

                /* Time-critical host application code goes here */

            }while (burst_flag) ;

        }
        
        #ifdef _DEBUG_INTERFACE_
            TIMESTAMP4;
        #endif
        
        /* Non-Time critical host application code goes here */
        
        
        #ifdef _DEBUG_INTERFACE_
            TIMESTAMP5;
        #endif
        /* Host sleep code goes here */

}


/*============================================================================
Name    :   qt_set_parameters
------------------------------------------------------------------------------
Purpose :   This will fill the default threshold values in the configuration
            data structure.But User can change the values of these parameters .
Input   :   n/a
Output  :   n/a
Notes   :   Generated Code from QTouch Studio. Do not change
============================================================================*/

static void qt_set_parameters( void )
{

/*  This will be modified by the user to different values   */
qt_config_data.qt_di              = 2;
qt_config_data.qt_neg_drift_rate  = 20;
qt_config_data.qt_pos_drift_rate  = 5;
qt_config_data.qt_max_on_duration = 0;
qt_config_data.qt_drift_hold_time = 20;
qt_config_data.qt_recal_threshold = 1;
qt_config_data.qt_pos_recal_delay = 3;

}

/*============================================================================
Name    :   config_sensors -
------------------------------------------------------------------------------
Purpose :   Configure the sensors
Input   :   n/a
Output  :   n/a
Notes   :   Generated code from QTouch Studio. Do not change
============================================================================*/
static void config_sensors(void)
{
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



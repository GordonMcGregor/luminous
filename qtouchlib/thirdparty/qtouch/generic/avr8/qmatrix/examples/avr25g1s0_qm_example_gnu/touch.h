/******************************************************************************* 
*   $FILE:  QTouchDesing1.h
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
#ifndef TOUCH_H
#define TOUCH_H

/******************************************************************************* 
*   Acquisition method definition
******************************************************************************/
//#define _QTOUCH_

#define _QMATRIX_

/****************************************************************************** 
*   Library configuration for QTouch acquisition
******************************************************************************/

#ifdef _QTOUCH_

/**
  * The following rules need to be followed while configuring 
  * the QTouch library for QTouch acquisition method.
  * 1. Always setup the following to have valid values
  *			#define QT_NUM_CHANNELS	<value>
  *			#define QT_DELAY_CYCLES	<value>
  *			#define SNS1			<value>
  * 			#define SNSK1			<value>
  *	The value for QT_NUM_CHANNELS should match the library chosen. For example, when using
  *    libavr51g1-12qt-k-0rs.a library, set QT_NUM_CHANNELS to 12 only. 
  * 2. If using a library with rotors and sliders, define _ROTOR_SLIDER as follows:
  *			#define _ROTOR_SLIDER_
  * 3. If SNS1 and SNSK1 pins are using the same port in intraport configuration, define
  * 		#define _SNS1_SNSK1_SAME_PORT_
  * 4. If using two port pairs, define
  *			#define SNS2			<value>
  *			#define SNSK2			<value>
  * 5. If SNS2 and SNSK2 pins are using the same port in intraport configuration, define
  *			#define _SNS2_SNSK2_SAME_PORT_
  * 6. If using a debug interface, define
  * 		#define _DEBUG_INTERFACE_
  * 7. If using power optimized code on ATTiny or ATMega devices, define
  *			#define _POWER_OPTIMIZATION 1
  *    Otherwise, for ATTiny or ATMega devices, add
  *			#define _POWER_OPTIMIZATION 0
  * 8. If using custom pin configuration, and would like to define pins used, 
  * 		#define QTOUCH_STUDIO_MASKS
  *    Otherwise, the library will automatically calculate pins used.
  *
  */



/**
  * Number of Channels(dependent on the library used).Please refer to the user guide
  * more information on selecting the number of channels.
  *
  * Possible values: 4, 8, 12, 16.
  */
#define QT_NUM_CHANNELS 8


/**
  * Delay cycles that determine the capacitance charge pulse width.
  *
  * Possible values: 1 to 255
  */
#define QT_DELAY_CYCLES 1

/**
  * Enabling the _ROTOR_SLIDER_ constant will link the library need for using rotors
  * and sliders. 
  *
  * Possible values: comment/uncomment the define
  */

//#define _ROTOR_SLIDER_
  

/**
 * If using a debug interface in your application, enable macro below
 */

#define _DEBUG_INTERFACE_



/**
  * Enabling _POWER_OPTIMIZATION_ will lead to a 40% reduction in power consumed
  * by the library, but by disabling spread spectrum feature. When power optimization
  * is enabled the unused pins, within a port used for QTouch, may not be usable for
  * interrupt driven applications. This option is available only for ATtiny and ATmega
  * devices.
  *
  * Possible values: 0 or 1 (For ATtiny and ATmega devices)
  *                  0 (For ATxmega devices)
  */

#ifndef _POWER_OPTIMIZATION_
#define _POWER_OPTIMIZATION_ 0
#endif



/**
  * Define the ports to be used for SNS1,SNS2 and SNSK1,SNSK2 pins. SNS1,SNS2 and SNSK1,SNSK2 port pins
  * can be available on the same port or on different ports
  *
  * Possible values: refer to the device data sheet and QTouch libraries user guide.
  */

#define SNS1	B
#define SNSK1	A

/**
 * 
 */
#define QTOUCH_STUDIO_MASKS
#endif // end of _QTOUCH_
/****************************************************************************** 
*   Library configuration for QMatrix acquisition
******************************************************************************/
#ifdef _QMATRIX_
/**
  * The following rules need to be followed while configuring 
  * the QTouch library for QTouch acquisition method.
  * 1. Always setup the following to have valid values
  *			#define QT_NUM_CHANNELS	<value>
  *			#define QT_DELAY_CYCLES	<value>
  *      The value for QT_NUM_CHANNELS should match the library chosen. For example, when using
  *     libavr5g4-64qm-8x-8y-krs-4rs.a library, set QT_NUM_CHANNELS to 64 only. 
  * 2. Define the following for X lines used
  *		  #define NUM_X_LINES	<value>
  *  		  #define NUM_X_PORTS <value>
  *  		  #define PORT_NUM_1	<value>
  *  		  #define PORT_NUM_2	<value, if using 2 X ports>
  *  		  #define PORT_NUM_3	<value, if using 3 X ports>
  * 		  #define PORT_X_1			<value>
  *		  #define PORT_X_2			<value, if using 2 X ports>
  *		  #define PORT_X_3			<value, if using 3 X ports>
  * 3. Define the following macros for Y lines used
  * 		 #define NUM_Y_LINES	<value>
  *		 #define PORT_YA				<value>
  *		 #define PORT_YB				<value>
  * 4. If ports for YA and YB are the same, define macro
  *		 #define SHARED_YAYB		1
  * 5 Define these macros for SMP pins used
  *		#define	PORT_SMP		<value>
  * 		#define 	SMP_PIN			<value>
  * 6. If using a library with rotors and sliders, define the macros below:
  *			#define _ROTOR_SLIDER_
  *			#define QT_MAX_NUM_ROTOR_SLIDER  <value>
  *    The value for QT_MAX_NUM_ROTOR_SLIDER should match the library chosen. For example, when using
  *    libavr5g4-64qm-8x-8y-krs-4rs.a library, set QT_MAX_NUM_ROTOR_SLIDER to 4 only. 
  * 7. If using a debug interface, define
  * 		#define _DEBUG_INTERFACE_
  *
  **/



/**
  * Delay cycles that determine the capacitance charge pulse width. Value of 0
  * will enable charge pulse width of 1 clock cycle, 1 will enable a width of 2
  * clock cycles and so on...
  *
  * Possible values: 1,2,3,4,5,10,25
  */
#ifndef QT_DELAY_CYCLES
#define QT_DELAY_CYCLES 4
#endif

/**
  * Define the Number of ROTORS/SLIDERS used.
  * Possible values: 0             ( if _ROTOR_SLIDER_ is not defined)
  *                  1, 2, 4 and 8 ( if _ROTOR_SLIDER_ is defined)
  * Depending on the library used.Please refer to the QTouch library user guide.pdf 
  * and library selection guide.xls more information on selecting the number of channels.
  */
#define QT_MAX_NUM_ROTORS_SLIDERS 0



/**
  * Number of Channels(dependent on the library used and application requirement).
  * The least possible number more that the application needs.
  * Please refer to the QTouch library user guide.pdf and library selection 
  * guide.xls more information on selecting the number of channels.
  *
  * Possible values: 4, 8, 12, 16, 32. in case of QTouch
  * Possible values: 4, 8, 16, 32, 56, 64. in case of QMatrix. 
  */
  #define QT_NUM_CHANNELS   8
/** 
  * Define the Number X lines to be used.
  * Possible values: 4 and 8
  * Depending on the library used.Please refer to the QTouch library user guide.pdf 
  * and library selection guide.xls more information on selecting the number of channels.
  *
  */
#define NUM_X_LINES	4
/**
  * Specify the number of ports on which X-lines that are distributed.
  * Note: Support is provided only for a maximum of 3 ports for X.
  *		 ( Maximum possible value for NUM_X_PORTS is 3)
  * Possible values: 1,2,3
  * Also, note that code memory increases with the number of ports
  * used for X lines.
  */
#define NUM_X_PORTS	1

#define PORT_NUM_1	1

/**
  * Specify Which ports have X lines on them. These macros are used
  * to conditionally compile in support for ports driving X lines.
  */
#define PORT_X_1	B

/** 
  * Define the Number Y lines to be used.
  *  Possible values: 1, 2, 4 and 8
  * Depending on the library used.Please refer to the QTouch library user guide.pdf 
  * and library selection guide.xls more information on selecting the number of channels.
  */
#define NUM_Y_LINES	2

/**
  * Specify the port for YA, YB, 
  * For rules to specify the port for YA ,YB please go through QTouch library 
  * user guide .pdf
  */
#define PORT_YA 	D
#define PORT_YB 	C

/**
  * Specify the port for SMP. 
  * And Specify the pin for SMP on selected SMP port. 
  * Any GPIO pin not conflicting with the other touch pins used for the application
  */
#define PORT_SMP 	D
#define SMP_PIN 	7

#define _DEBUG_INTERFACE_

#endif //end of _QMATRIX_

/****************************************************************************** 
*  Post Processing Configuration for Touch Library
******************************************************************************/

/**
  * Total ticks per msec.
  * TICKS_PER_MS = (CLK_FREQ/TIMER_PRESCALER)*(1/1000)
  * Current Clock frequency is 4Mhz, and Timer Prescaler is 8
  */
#define TICKS_PER_MS                500u

/* Initialization values for the Qtouch library parameters. */
/*
* Sensor detect integration (DI) limit.
* Default value: 4.
*/
#define DEF_QT_DI                      4u

/*
* Sensor negative drift rate.
*
* Units: 200ms
* Default value: 20 (4 seconds per LSB).
*/
#define DEF_QT_NEG_DRIFT_RATE          20      /* 4s per LSB */

/*
* Sensor positive drift rate.
*
* Units: 200ms
* Default value: 5 (1 second per LSB).
*/
#define DEF_QT_POS_DRIFT_RATE          5       /* 1s per LSB */

/*
* Sensor drift hold time.
*
* Units: 200ms
* Default value: 20 (hold off drifting for 4 seconds after leaving detect).
*/
#define DEF_QT_DRIFT_HOLD_TIME         20      /* 4s */

/*
* Sensor maximum on duration.
*
* Units: 200ms (e.g., 150 = recalibrate after 30s). 0 = no recalibration.
* Default value: 0 (recalibration disabled).
*/
#define DEF_QT_MAX_ON_DURATION         0       /* disabled */

/*
* Sensor recalibration threshold.
*
* Default: RECAL_50 (recalibration threshold = 50% of detection threshold).
*/
#define DEF_QT_RECAL_THRESHOLD         RECAL_50 /* recal threshold = 50% of detect */

/*
* Positive recalibration delay.
*
* Default: 3
*/
#define DEF_QT_POS_RECAL_DELAY         3u
/****************************************************************************** 
*   Debug Inteface settings
******************************************************************************/
 
 #ifdef _DEBUG_INTERFACE_
 // Ensure that the port pins mentioned below are not used for SNS/SNSK
#define SS_BB       2
#define SCK_BB      3
#define MOSI_BB     4
#define MISO_BB     5

#define SPI_BB_SS      		D
#define SPI_BB_SCK      	D
#define SPI_BB_MOSI      	D
#define SPI_BB_MISO      	D

// Select the type of interface to use for the debug protocol.
// Comment out the interface not used.
// Only one interface should be active.
//#define	QDEBUG_SPI	/* If using on-chip SPI peripheral*/
//#define QDEBUG_TWI		/* If using on-chip TWI peripheral */
#define QDEBUG_SPI_BB		/* If using Bit Bang SPI */

#endif

// Set up project info
#define		PROJECT_ID			QM08

#endif //TOUCH_H

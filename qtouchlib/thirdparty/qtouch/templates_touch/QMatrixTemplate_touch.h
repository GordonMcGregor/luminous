/*******************************************************************************
*   touch.h
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
/**
  * Acquisition Technology Name.
  *
  * Possible values: _QMATRIX_, _QTOUCH_ .
  */
#define _QMATRIX_

/**
  * Device Name.
  *
  */
$$$DEVICE_NAME$$$

/**
  * Delay cycles that determine the capacitance charge pulse width. Value of 0
  * will enable charge pulse width of 1 clock cycle, 1 will enable a width of 2
  * clock cycles and so on...
  *
  * Possible values: 1,2,3,4,5,10,25,50
  */
#ifndef QT_DELAY_CYCLES
$$$QT_DELAY_CYCLES$$$
#endif


/**
  * Define the Number of ROTORS/SLIDERS used.
  * Possible values: 0             ( if _ROTOR_SLIDER_ is not defined)
  *                  1, 2, 4 and 8 ( if _ROTOR_SLIDER_ is defined)
  * Depending on the library used..
  */
$$$QT_MAX_NUM_ROT_SLIDERS$$$



$$$QMATRIX_CONSTANTS$$$



/**
  * Provide the number of timer clock ticks (cycles) required to provide a 1 millisecond time interval.
  * TICKS_PER_MS = (CLK_FREQ/TIMER_PRESCALER)*(1/1000)
  * Example, TICKS_PER_MS = (8MHz/64)*(1/1000) = 125
  */
$$$QT_TICKS_PER_MS$$$

/**
  * Provide the periodic interrupt interval for which the timer is configured.
  * Example, QT_TIMER_PERIOD_MSEC  2u
  * Timer ISR will fire at every 2 milliseconds.
  */
#define QT_TIMER_PERIOD_MSEC  1u

/**
  * Provide the periodic interval for touch measurement.
  * Example, QT_MEASUREMENT_PERIOD_MS 50u
  * Perform a single touch measurement every 50msec.
  */
$$$QT_MEASUREMENT_PERIOD_MS$$$





/******************************************************************************
*   Debug Interface Settings
******************************************************************************/

$$$QT_DEBUG_INTERFACE$$$


$$$QT_QDEBUG_TIME_STAMPS$$$


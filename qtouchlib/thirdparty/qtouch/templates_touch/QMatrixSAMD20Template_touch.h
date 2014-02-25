/* This source file is part of the ATMEL QTouch Library Release 5.0 */

/*****************************************************************************
 *
 * \file
 *
 * \brief  This file contains the SAMD20 QTouch Library pin, register and
 *sensors
 * configuration options for Capacitive Touch acquisition using the PTC module.
 *
 *
 * - Userguide:          QTouch Library Peripheral Touch Controller User Guide.
 * - Support email:      touch@atmel.com
 *
 *
 * Copyright (c) 2013 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 ******************************************************************************/

#ifndef TOUCH_CONFIG_SAMD20_H
#define TOUCH_CONFIG_SAMD20_H

/*----------------------------------------------------------------------------
*                   PTC module clock and interrupt level configuration.
*  ----------------------------------------------------------------------------*/

/**
 * PTC Module clock configuration.
 * Before using the QTouch library API, the PTC module clock generator source
 * should be configured appropriately.  The PTC module clock can be generated
 * using any of the eight generic clock generators (GCLK0-GCLK7).  The associated
 * generic clock multiplexer should be configured such that the PTC module clock
 * is set to 4MHz.  Refer touch_configure_ptc_clock API in touch.c for more
 * information.
 *
 * PTC Module interrupt level.
 * The Nested Vectored Interrupt Controller (NVIC) in the SAM D20 supports
 * four different priority levels.  The priority level of the PTC end of
 * conversion ISR can be chosen based on application requirements in order
 * to accommodate time critical operations.
 */
#define DEF_TOUCH_PTC_ISR_LVL   (1u)

/*----------------------------------------------------------------------------
*                   Mutual Cap method enable/disable.
*  ----------------------------------------------------------------------------*/

/**
 * Enable/Disable Self/Mutual Capacitance method.
 */
#define DEF_TOUCH_MUTLCAP               (1u)
#define DEF_TOUCH_SELFCAP               (0u)
/*----------------------------------------------------------------------------
*                   Mutual Cap method pin configuration.
*  ----------------------------------------------------------------------------*/

/**
 * Mutual Cap method touch channel nodes (GPIO pins) selected.
 * Mutual capacitance method use a pair of sensing electrodes for each touch
 * channel.  These are denoted as X and Y lines.  Touch channel numbering
 * follows the order in which X-Y nodes are specified.  Capacitance measurement
 * is done sequentially in the order in which touch channel nodes are specified.
 */
$$$DEF_MUTLCAP_NODES$$$

/*----------------------------------------------------------------------------
*                   Mutual Cap method sensor configuration.
*  ----------------------------------------------------------------------------*/

/**
 * Mutual Cap number of channels.
 * Specify the number of Mutual Cap touch channels to be used by the Touch
 * Library.
 * A key is formed used one touch channel.  A rotor or slider can be formed
 * using 3 to 8 touch channels.
 * Range: 1u to 256u.
 */
$$$DEF_MUTLCAP_NUM_CHANNELS$$$

/**
 * Mutual Cap number of Sensors.
 * Specify the number of Mutual Cap touch sensors to be used by the Touch
 *Library.
 * A sensor is either a key, rotor or slider.
 * Example configuration: If the configuration has 6 keys (a key is formed
 * using one X-Y node), one rotor (a Mutual Cap rotor is formed using 3 X-Y
 * nodes) and one slider (a Mutual Cap slider is formed using 3 X-Y nodes),
 *then
 * the number of sensors is 6 key + 1 rotor + 1 slider = 8 sensors.
 * Range: 1u to 256u.
 */
$$$DEF_MUTLCAP_NUM_SENSORS$$$

/**
 * Mutual Cap number of Rotors and Sliders.
 * Specify the total number of Mutual Cap Rotors and Sliders to be used by
 * the Touch Library.  The number of Rotors and Sliders mentioned here is part
 *of
 * the Total number of sensors specified in the QT_NUM_SENSORS macro.  When
 * no rotors or slider are required, specify a value of 0u.
 */
$$$DEF_MUTLCAP_NUM_ROTORS_SLIDERS$$$

/*----------------------------------------------------------------------------
*                   Mutual Cap method aquisition parameters.
*  ----------------------------------------------------------------------------*/

/**
 * Mutual Cap noise counter measure enable/disable.
 * Noise counter measure setting allows users to trade-off between power
 * consumption or noise immunity.  When disabled the PTC is optimized for
 * fastest operation or lowest power operation, when enabled it is optimized
 * for best noise immunity.  During initialization, the QTouch library
 * carries out charge transfer auto-tuning to ensure full charge transfer
 * for each sensor, by adjusting either the internal series resistor or
 * the PTC clock pre-scalar.  When noise countermeasure is enabled the internal
 * series resistor is set to maximum and the PTC pre-scalar is adjusted to
 * slow down the PTC operation to ensure full charge transfer. Enabling noise
 * countermeasure also implements a frequency hopping cycle and a median filter.
 * When disabled, the PTC runs at maximum speed and the series resistor is set
 * to the optimum value which still allows full charge transfer.  Frequency
 * hopping and median filter are not applied.
 * 0u: Disable noise counter measures.
 * 1u: Enable noise counter measures.
 */
$$$DEF_MUTLCAP_NOISE_COUNTER_MEASURE$$$

/**
 * Mutual Cap filter level setting.
 * The filter level setting controls the number of samples taken
 * to resolve each acquisition. A higher filter level setting provides
 * improved signal to noise ratio under noisy conditions, while
 * increasing the total time for measurement resulting in increased
 * power consumption.  Refer filter_level_t in touch_api_SAMD20.h
 * Range: FILTER_LEVEL_1 (one sample) to FILTER_LEVEL_64 ( 64 samples).
 */
$$$DEF_MUTLCAP_FILTER_LEVEL$$$

/**
 * Mutual Cap auto oversample setting.
 * Auto oversample controls the automatic oversampling of sensor channels when
 * unstable signals are detected with the default setting of ‘Filter level’.
 * Each increment of ‘Auto Oversample’ doubles the number of samples taken on
 * the corresponding sensor channel when an unstable signal is observed.
 * In a case where ‘Filter level’ is set to FILTER_LEVEL_4 and ‘Auto Oversample’
 * is set to AUTO_OS_4, 4 oversamples are taken with stable signal values and 16
 * oversamples are taken when unstable signal is detected.
 * Refer auto_os_t in touch_api_SAMD20.h
 * Range: AUTO_OS_DISABLE (oversample disabled) to AUTO_OS_128 (128
 *oversamples).
 */
$$$DEF_MUTLCAP_AUTO_OS$$$

/**
 * Mutual Cap gain per touch channel.
 * Gain is applied on a per-channel basis to allow a scaling-up of the touch
 *delta
 * Note: delta on touch contact, not the resting signal which is measured on
 *each sensor.
 * Refer gain_t in touch_api_SAMD20.h
 * Range:GAIN_1 (no scaling) to GAIN_32 (scale-up by 32)
 */
$$$DEF_MUTLCAP_GAIN_PER_NODE$$$

/*----------------------------------------------------------------------------
*                   Mutual Cap method sensor global parameters.
*  ----------------------------------------------------------------------------*/

/*! \name Mutual Cap Global acquisition parameters.
 * Refer the Touch Library User guide for more information on these parameters.
 */
/* ! @{ */

/**
 * Mutual Cap Sensor measurement interval.
 * Speicify period in milliseconds.  Example, DEF_TOUCH_MEASUREMENT_PERIOD_MS 50u
 * will perform measurement on touch sensors every 50msec.
 */
$$$DEF_TOUCH_MEASUREMENT_PERIOD_MS$$$

/**
 * Mutual Cap Sensor detect integration (DI) limit.
 * Range: 0u to 255u.
 */
$$$DEF_MUTLCAP_DI$$$

/**
 * Mutual Cap Sensor towards touch drift rate.
 * Units: 200ms
 * Default value: 20 = 4 seconds.
 * Range: 1u to 127u.
 */
$$$DEF_MUTLCAP_TCH_DRIFT_RATE$$$

/**
 * Mutual Cap Sensor away from touch drift rate.
 * Units: 200ms
 * Default value: 5u = 1 second.
 * Range: 1u to 127u.
 */
$$$DEF_MUTLCAP_ATCH_DRIFT_RATE$$$

/**
 * Mutual Cap Sensor maximum ON time duration.
 * Units: 200ms (Example: a value 5u indicated Max ON duration of 1 second.)
 * Default value: 0 (No maximum ON time limit).
 * Range: 0u to 255u.
 */
$$$DEF_MUTLCAP_MAX_ON_DURATION$$$

/**
 * Mutual Cap Sensor drift hold time.
 * Units: 200ms
 * Default value: 20 (hold off drifting for 4 seconds after leaving detect).
 * Range: 1u to 255u.
 */
$$$DEF_MUTLCAP_DRIFT_HOLD_TIME$$$

/**
 * Mutual Cap Sensor away from touch recalibration delay.
 * Default value: 10.
 * Range: 1u to 255u.
 */
$$$DEF_MUTLCAP_ATCH_RECAL_DELAY$$$

/** Mutual Cap Sensor away from touch recalibration threshold.
 * Default: RECAL_50 (recalibration threshold = 50% of detection threshold).
 * Range: refer recal_threshold_t enum in touch_api_SAMD20.h.
 */
$$$DEF_MUTLCAP_ATCH_RECAL_THRESHOLD$$$

/* ! @} */

/*----------------------------------------------------------------------------
*                   Mutual Cap method callback functions.
*  ----------------------------------------------------------------------------*/

/*! \name Mutual Cap Callback functions.
 */
/* ! @{ */

/**
 * Mutual Cap Filter callback function.
 * A filter callback (when not NULL) is called by the Touch Library each time
 * a new set of Signal values are available.
 * An Example filter callback function prototype.
 * void qm_filter_callback( void );
 */
#define DEF_MUTLCAP_FILTER_CALLBACK              (NULL)

/* ! @} */

/*----------------------------------------------------------------------------
*                   QDebug debug communication parameters.
*  ----------------------------------------------------------------------------*/

/*! \name QDebug debug communication parameters.
 */
/* ! @{ */

#define DEF_TOUCH_QDEBUG_ENABLE_MUTLCAP

$$$DEF_TOUCH_QDEBUG_ENABLE$$$

/* ! @} */

#endif /* TOUCH_CONFIG_SAMD20_H */

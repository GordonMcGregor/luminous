/* This source file is part of the ATMEL QTouch Library Release 5.0 */

/*****************************************************************************
 *
 * \file
 *
 * \brief  This file contains the SAMD20 QTouch library initialization,
 * sensor configuration and measurement routines.
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

/*----------------------------------------------------------------------------
*                               include files
*  ----------------------------------------------------------------------------*/

/**
 * Includes for Touch Library.
 */
#include <asf.h>
#include "touch_api_SAMD20.h"

/*----------------------------------------------------------------------------
*                           manifest constants
*  ----------------------------------------------------------------------------*/
#define PTC_GCLK_ID 27u
#define PTC_APBC_BITMASK (1u << 19u)

/*----------------------------------------------------------------------------
*                                   macros
*  ----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
*                           type definitions
*  ----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
*                           Structure Declarations
*  ----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
*                               global variables
*  ----------------------------------------------------------------------------*/
/* ! QTouch Library Timing info. */
touch_time_t touch_time;

/**
 * Mutual Cap sensors measured data pointer.
 * Note: This pointer is initialized by the QTouch library once the
 * touch_mutlcap_sensors_init API is called. */
touch_measure_data_t *p_mutlcap_measure_data = NULL;

/**
 * Self Cap sensors measured data pointer.
 * Note: This pointer is initialized by the QTouch library once the
 * touch_selfcap_sensors_init API is called. */
touch_measure_data_t *p_selfcap_measure_data = NULL;

/*----------------------------------------------------------------------------
*                               static variables
*  ----------------------------------------------------------------------------*/

/**
 * Self Cap sensors data block provided as input to Touch library.
 */
static uint8_t selfcap_data_blk[PRIV_SELFCAP_DATA_BLK_SIZE];

/**
 * Self Cap sensors Pins Info.
 */

uint16_t selfcap_y_nodes[DEF_SELFCAP_NUM_CHANNELS] = {DEF_SELFCAP_LINES};

gain_t selfcap_gain_per_node[DEF_SELFCAP_NUM_CHANNELS]
	= {DEF_SELFCAP_GAIN_PER_NODE};

/**
 * Self Cap Configuration structure provided as input to Touch Library.
 *
 * Note: Use the touch_config_SAMD20.h configuration header file to fill in
 * the elements of this structure.  DO NOT modify any of the input values
 * directly in this structure.
 */
static touch_selfcap_config_t selfcap_config = {
	DEF_SELFCAP_NUM_CHANNELS,       /* Self Cap number of channels. */
	DEF_SELFCAP_NUM_SENSORS,        /* Self Cap number of sensors. */
	DEF_SELFCAP_NUM_ROTORS_SLIDERS, /* Self Cap number of rotors and
	                                 *sliders. */

	/* Self Cap GLOBAL SENSOR CONFIGURATION INFO. */
	{
		DEF_SELFCAP_DI,         /* uint8_t  di; Sensor detect
		                         *integration (DI) limit. */
		/* Interchanging Negative and Positive Drift rate, since Signal
		 *increases on Touch. */
		DEF_SELFCAP_ATCH_DRIFT_RATE, /* uint8_t  neg_drift_rate; Sensor
		                              *negative drift rate. */
		DEF_SELFCAP_TCH_DRIFT_RATE, /* uint8_t  pos_drift_rate; Sensor
		                             *positive drift rate. */
		DEF_SELFCAP_MAX_ON_DURATION, /* uint8_t  max_on_duration; Sensor
		                              *maximum on duration. */
		DEF_SELFCAP_DRIFT_HOLD_TIME, /* uint8_t  drift_hold_time; Sensor
		                              *drift hold time. */
		DEF_SELFCAP_ATCH_RECAL_DELAY,   /* uint8_t  pos_recal_delay;
		                                 *Sensor positive recalibration
		                                 *delay. */
		DEF_SELFCAP_ATCH_RECAL_THRESHOLD, /* recal_threshold_t
		                              *recal_threshold; Sensor
		                              *recalibration threshold. */
	},
	{
	selfcap_gain_per_node, /* Self Cap channel gain setting. */
	DEF_SELFCAP_NOISE_COUNTER_MEASURE, /* Self Cap noise counter measure
	                            *enable/disable. */
	DEF_SELFCAP_FILTER_LEVEL, /* Self Cap filter level setting. */
	DEF_SELFCAP_AUTO_OS, /* Self Cap auto oversamples setting. */
	},
	selfcap_data_blk,               /* Self Cap data block index. */
	PRIV_SELFCAP_DATA_BLK_SIZE, /* Self Cap data block size. */
	selfcap_y_nodes, /* Self Cap channel nodes. */
	DEF_SELFCAP_FILTER_CALLBACK     /* Self Cap filter callback function
	                                 *pointer. */
};

/**
 * Touch Library input configuration structure.
 */
touch_config_t touch_config = {
	NULL,
	&selfcap_config,                /* Pointer to Self Cap configuration
	                                 *structure. */
    DEF_TOUCH_PTC_ISR_LVL,  		/* PTC interrupt level. */
};

/*----------------------------------------------------------------------------
*                               prototypes
*  ----------------------------------------------------------------------------*/

/*! \brief configure keys, rotors and sliders.
 */
static touch_ret_t touch_sensors_config(void);

/*! \brief Self Cap measure complete callback function example prototype.
 */
void touch_selfcap_measure_complete_callback( void );

/*! \brief Initialize and enable PTC clock.
 */
void touch_configure_ptc_clock(void);
/*----------------------------------------------------------------------------
*  static functions
*  ----------------------------------------------------------------------------*/

void touch_configure_ptc_clock(void)
{
	struct system_gclk_chan_config gclk_chan_conf;

	system_gclk_chan_get_config_defaults(&gclk_chan_conf);

	gclk_chan_conf.source_generator = GCLK_GENERATOR_1;

	system_gclk_chan_set_config(PTC_GCLK_ID, &gclk_chan_conf);

	system_gclk_chan_enable(PTC_GCLK_ID);

	system_apb_clock_set_mask(SYSTEM_CLOCK_APB_APBC, PTC_APBC_BITMASK);
}

/* ! [setup] */

/*! \brief Example application entry function.
 */
touch_ret_t touch_sensors_init(void)
{
	touch_ret_t touch_ret = TOUCH_SUCCESS;

	/* Setup and enable generic clock source for PTC module. */
	touch_configure_ptc_clock();

	touch_time.measurement_period_ms = DEF_TOUCH_MEASUREMENT_PERIOD_MS;

	/* Initialize touch library for Self Cap operation. */
	touch_ret = touch_selfcap_sensors_init(&touch_config);
	if (touch_ret != TOUCH_SUCCESS) {
		while (1u) {    /* Check API Error return code. */
		}
	}

#if DEF_TOUCH_QDEBUG_ENABLE == 1
	QDebug_Init();
#endif

	/* configure the touch library sensors. */
	touch_ret = touch_sensors_config();
	if (touch_ret != TOUCH_SUCCESS) {
		while (1u) {    /* Check API Error return code. */
		}
	}

	touch_ret = touch_selfcap_sensors_calibrate();
	if (touch_ret != TOUCH_SUCCESS) {
		while (1u) {    /* Check API Error return code. */
		}
	}

	return (touch_ret);
}

touch_ret_t touch_sensors_measure(void)
{
	touch_ret_t touch_ret = TOUCH_SUCCESS;

	if (touch_time.time_to_measure_touch == 1u) {
		touch_ret = touch_selfcap_sensors_measure(
				touch_time.current_time_ms,
				NORMAL_ACQ_MODE,
				touch_selfcap_measure_complete_callback);

		if ((touch_ret != TOUCH_ACQ_INCOMPLETE) &&
				(touch_ret == TOUCH_SUCCESS)) {
			touch_time.time_to_measure_touch = 0u;
		} else if ((touch_ret != TOUCH_SUCCESS) &&
				(touch_ret != TOUCH_ACQ_INCOMPLETE)) {
			while (1) {
			}

			/* Reaching this point can be due to -
			 *     1. The api has retured an error due to a invalid
			 * input parameter.
			 *     2. The api has been called during a invalid Touch
			 * Library state. */
		}
	}

	return (touch_ret);
}

/*! \brief Example configuration for touch keys, rotors and sliders.
 *
 * \retval TOUCH_SUCCESS   Configuration setting successful.
 * \retval TOUCH_INVALID_xx   Invalid input configuration setting.
 */
touch_ret_t touch_sensors_config(void)
{
	touch_ret_t touch_ret = TOUCH_SUCCESS;
	sensor_id_t sensor_id;

	$$$TOUCH_SENSORS_CONFIG$$$

	return (touch_ret);
}

/*! \brief Self Cap measure complete callback function.
 *
 * This function is called by the library when the touch measurement
 * process for Self Cap sensors is completed.
 *
 * \param p_measure_data   Base address of touch_measure_data_t instance.
 * \note A touch_selfcap_measure_complete_callback() call signifies that fresh
 * values of touch status, rotor/slider position, measured signals,
 * references and Sensor data is available.
 * The Self Cap measured data can be accessed using the p_measure_data
 * pointer.
 */
void touch_selfcap_measure_complete_callback( void )
{
#if DEF_TOUCH_QDEBUG_ENABLE == 1

	/* Send out the Touch debug information data each time when Touch
	 *   measurement process is completed .
	 *   The Touch Signal and Touch Delta values are always sent.
	 *   Touch Status change, Rotor-Slider Position change and Sensor
	 * Reference
	 *   values can be optionally sent using the masks below.
	 */
	QDebug_SendData( TOUCH_CHANNEL_REF_CHANGE |
			TOUCH_ROTOR_SLIDER_POS_CHANGE |
			TOUCH_STATUS_CHANGE );

	/* QT600 two-way QDebug communication application Example. */
	/* Process any commands received from QTouch Studio. */
	QDebug_ProcessCommands();
#endif

	if (!(p_selfcap_measure_data->acq_status & TOUCH_BURST_AGAIN)) {
		/* Set the QTouch measurement done flag. */
		p_selfcap_measure_data->measurement_done_touch = 1u;
	}
}
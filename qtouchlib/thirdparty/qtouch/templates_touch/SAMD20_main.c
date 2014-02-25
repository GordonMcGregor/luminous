/* This source file is part of the ATMEL QTouch Library Release 5.0 */

/*****************************************************************************
 *
 * \file
 *
 * \brief  This file contains the SAMD20 QTouch library sample user application.
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

/**
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
#include <asf.h>

/*! \brief Initialize timer
 *
 */
void timer_init( void );

/*! \brief RTC timer overflow callback
 *
 */
void rtc_overflow_callback(void);

/*! \brief Configure the RTC timer callback
 *
 */
void configure_rtc_callbacks(void);

/*! \brief Configure the RTC timer count after which interrupts comes
 *
 */
void configure_rtc_count(void);

/*! \brief Set timer period.Called from Qdebug when application
 *     has to change the touch time measurement
 */
void set_timer_period(void);

/*! \brief RTC timer overflow callback
 *
 */
void rtc_overflow_callback(void)
{
	/* Do something on RTC overflow here */
	touch_time.time_to_measure_touch = 1;
	touch_time.current_time_ms = touch_time.current_time_ms +
			touch_time.measurement_period_ms;
}

/*! \brief Configure the RTC timer callback
 *
 */
void configure_rtc_callbacks(void)
{
	/* register callback */
	rtc_count_register_callback(
			rtc_overflow_callback, RTC_COUNT_CALLBACK_OVERFLOW);
	/* Enable callback */
	rtc_count_enable_callback(RTC_COUNT_CALLBACK_OVERFLOW);
}

/*! \brief Configure the RTC timer count after which interrupts comes
 *
 */
void configure_rtc_count(void)
{
	struct rtc_count_config config_rtc_count;
	rtc_count_get_config_defaults(&config_rtc_count);

	config_rtc_count.prescaler           = RTC_MODE0_CTRL_PRESCALER_DIV32;
	config_rtc_count.mode                = RTC_COUNT_MODE_16BIT;
	config_rtc_count.continuously_update = true;
	/* initialize rtc */
	rtc_count_init(&config_rtc_count);

	/* enable rtc */
	rtc_count_enable();
}

/*! \brief Initialize timer
 *
 */
void timer_init(void)
{
	/* Configure and enable RTC */
	configure_rtc_count();

	/* Configure and enable callback */
	configure_rtc_callbacks();

	/* Set Timer Period */

	rtc_count_set_period(DEF_TOUCH_MEASUREMENT_PERIOD_MS);
}
/*! \brief Set timer period.Called from Qdebug when application
 *     has to change the touch time measurement
 */
void set_timer_period(void )
{
	rtc_count_set_period(touch_time.measurement_period_ms);
}

/*! \brief Main function
 *
 */
int main(void)
{
	/**
	 * Initialize and configure system and generic clocks.
	 * Use conf_clocks.h to configure system and generic clocks.
	 * This example project uses Internal 8MHz oscillator.
	 * The PTC module clock is provided using GCLK generator 1.
	 */
	system_init();

	/**
	 * Initialize timer.
	 * This example projects uses RTC timer for providing timing
	 * info to QTouch library.
	 */
	timer_init();

	/**
	 * Initialize QTouch library and configure touch sensors.
	 */
	touch_sensors_init();

	/* Configure System Sleep mode to STANDBY. */
	system_set_sleepmode(SYSTEM_SLEEPMODE_STANDBY);

	while (1) {
		/**
		 * Goto STANDBY sleep mode, unless woken by timer or PTC interrupt.
		 */
		system_sleep();

		/**
		 * Start touch sensor measurement, if touch_time.time_to_measure_touch flag is set by timer.
		 */
		touch_sensors_measure();

		/**
		* Update touch status once measurement complete flag is set.
		*/

		/**
		 * Self Cap method
		 * if ((p_selfcap_measure_data->measurement_done_touch == 1u))  for self cap
		 * Touch sensor ON/OFF status or rotor/slider position.
		 *
		 * Self Cap method
		 * uint8_t sensor_state =
		 *GET_SELFCAP_SENSOR_STATE(SENSOR_NUMBER);
		 * uint8_t rotor_slider_position =
		 *GET_SELFCAP_ROTOR_SLIDER_POSITION(SENSOR_NUMBER);

		 */

		/**
		 * Mutual Cap method
		 * if ((p_mutlcap_measure_data->measurement_done_touch == 1u))  for mutual cap
		 * Touch sensor ON/OFF status or rotor/slider position.
		 *
		 *
		 * uint8_t sensor_state =
		 *GET_MUTLCAP_SENSOR_STATE(SENSOR_NUMBER);
		 * uint8_t rotor_slider_position =
		 *GET_MUTLCAP_ROTOR_SLIDER_POSITION(SENSOR_NUMBER);
	 */
	}
}

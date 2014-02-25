/**
 * \file
 *
 * \brief QTouch Demo for SAM4L.
 *
 * Copyright (c) 2012 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
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
 */
/**
 * \mainpage QTouch Demo for SAM4L
 *
 * \section Purpose
 *
 * The QTouch demo will help new users get familiar with using QTouch on Atmel's
 * SAM4L microcontrollers. This basic application shows how to initialize
 * the QTouch library and send touch related debug data to QTouch Analyzer
 * PC Software using QT600 based QDebug interface.
 *
 * \section Requirements
 *
 * This package can be used with SAM4L-EK evaluation kits.
 *
 * \section Description
 *
 * The Example application can be used with a SAM4L based atmel or user board.
 * In order to transfer the Touch related debug information from the user board
 * to QTouch Analyzer PC software, QT600 USB Bridge board is required.
 * The QDebug component in this Example application uses Software based SPI
 * to transfer data to the PC Application.  The User should connect the required
 * pins from the user board to the QT600 USB bridge board to establish proper
 * communication between the PC Application and the user board.
 *
 */

#include "asf.h"

/**
 *  \brief AST callback
 */
 static void ast_per_callback(void);
 /**
 *  \brief Configure AST
 */
 void configure_ast(void);

/**
 *  \brief Configure AST
 */
 void configure_ast(void)
{
	struct ast_config ast_conf;
	ast_enable(AST);
	ast_conf.mode = AST_COUNTER_MODE;
	ast_conf.osc_type = AST_OSC_32KHZ;
	ast_conf.psel = 1;
	ast_conf.counter = 0;
	ast_set_config(AST, &ast_conf);

	ast_clear_interrupt_flag(AST, AST_INTERRUPT_PER);
	ast_write_periodic0_value(AST, 9);

	ast_set_callback(AST, AST_INTERRUPT_PER, ast_per_callback,
	AST_PER_IRQn, 0);
}
/**
 *  \brief Asynchronous timer (ASF) callback for the QTouch acquisition.
 *  and clear Watchdog counter - generates interrupt every desired period
 */
static void ast_per_callback(void)
{
	touch_sensors_update_time();
	ast_clear_interrupt_flag(AST, AST_INTERRUPT_PER);
}

/**
 *  \brief Lower power and QTouch Demo for SAM4L entry point.
 *  \return Unused (ANSI-C compatibility).
 */
int main(void)
{

	// Initialize board features
	board_init();

	// Initialize clock
	sysclk_init();
	//set RC32K
	bpm_set_clk32_source(BPM,BPM_CLK32_SOURCE_RC32K);			
	// Enable RC32
		if(!osc_is_ready(OSC_ID_RC32K))
		{
			osc_enable(OSC_ID_RC32K);
			osc_wait_ready(OSC_ID_RC32K);
		}
	// Configure AST Controller
	configure_ast();
		
	//CATB & Related clocks - for QTouch Sensors
	sysclk_enable_peripheral_clock(CATB);
	sysclk_enable_peripheral_clock(PDCA);

	// Initialize QTouch Library.
	touch_sensors_init();

	while (1)
	{
          // Do QTouch measurement and update Touch Status.
		  touch_sensors_measure();
	}

}// end main function

/******************************************************************************* 
*   $FILE:  TWI_Master.c
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

/*============================ INCLUDES ======================================*/
#include "touch.h"

#if defined(_DEBUG_INTERFACE_) && defined(QDEBUG_TWI)
#include "TWI_Master.h"
#include "QDebugTransport.h"
#include "avr_compiler.h"

/*============================ LOCAL VARIABLES ===============================*/
TWI_Master_t twiMaster;    /*!< TWI master module. */

/*============================ IMPLEMENTATION ================================*/

unsigned int gMsTimeout;
#define NOT_TIMEDOUT()					(gMsTimeout)
#define SET_TIMEOUT(timeout)			(gMsTimeout = timeout)


/*============================================================================
Name    :   TWI_Master_Init
------------------------------------------------------------------------------
Purpose :   Initialize TWI Interface
Input   :   n/a
Output  :   n/A
Notes   :	Called from QDebug_Init QDebug.c
============================================================================*/
void TWI_Master_Init(void)
{
	/*  This driver is written to use TWIC on the QT600-ATXMEGA128A1-QT16 board.
        If this driver is used for another board it must be checked which
        TWI module to use and the pointer to the selected TWI module must be 
        set below. 	
	*/
	
	/* Enable Lo interrupt level. */
	PMIC.CTRL |= PMIC_LOLVLEN_bm;
  
  	// Initialize TWI master. 
	twiMaster.interface = &TWIC;
	twiMaster.interface->MASTER.BAUD 	= 	TWI_BAUDSETTING;
	twiMaster.interface->MASTER.CTRLA 	= 	TWI_MASTER_INTLVL_LO_gc |
	                               			TWI_MASTER_RIEN_bm |
	                               			TWI_MASTER_WIEN_bm |
	                               			TWI_MASTER_ENABLE_bm;
	twiMaster.interface->MASTER.STATUS 	= 	TWI_MASTER_BUSSTATE_IDLE_gc;
	twiMaster.status = TWIM_STATUS_READY;
}

/*============================================================================
Name    :   TWI_Retrieve_Message
------------------------------------------------------------------------------
Purpose :   Read one frame using TWI Interface
Input   :   n/a
Output  :   returns true if transmission sucessful
Notes   :	Called from Receive_Message in QDebugTransport.c
============================================================================*/
bool TWI_Retrieve_Message(void)
{
	SET_TIMEOUT(10);	// 10ms timeout
  
	//Initiate transaction if bus is ready. 
	if (twiMaster.status == TWIM_STATUS_READY)
	{
		twiMaster.status = TWIM_STATUS_BUSY;
		twiMaster.result = TWIM_RESULT_UNKNOWN;
	
		twiMaster.address = SLAVE_ADDRESS<<1;
		twiMaster.bytesWritten = 0;
		twiMaster.bytesRead = 0;
	
		// send the START condition + Address + 'R/_W = 1'
		uint8_t readAddress = twiMaster.address | 0x01;
		twiMaster.interface->MASTER.ADDR = readAddress;
	}
	else
	{
		return false;
	}
	
	// Wait until transaction is complete or timed out
	while ((twiMaster.status != TWIM_STATUS_READY) && NOT_TIMEDOUT());	
	
	twiMaster.status= TWIM_STATUS_READY;
	
	if (gMsTimeout)
		return true;
	else
	  	return false;	
}

/*============================================================================
Name    :   TWI_Send_Message
------------------------------------------------------------------------------
Purpose :   Send one frame using TWI Interface
Input   :   n/a
Output  :   returns true if transmission sucessful
Notes   :	Called from Send_Message in QDebugTransport.c
============================================================================*/
bool TWI_Send_Message(void)
{
	SET_TIMEOUT(10);	// 10ms timeout
	
  	//Initiate transaction if bus is ready. 
	if (twiMaster.status == TWIM_STATUS_READY)
	{
		twiMaster.status = TWIM_STATUS_BUSY;
		twiMaster.result = TWIM_RESULT_UNKNOWN;
		twiMaster.address = SLAVE_ADDRESS<<1;
		twiMaster.bytesToWrite = (TX_Buffer[1]<<8)+TX_Buffer[2];
		twiMaster.bytesWritten = 0;

		// If write command, send the START condition + Address +
		// 'R/_W = 0'
		if (twiMaster.bytesToWrite > 0)
		{
			uint8_t writeAddress = twiMaster.address & ~0x01;
			twiMaster.interface->MASTER.ADDR = writeAddress;
		}
	} 
	else
	{
		return false;
	}
	
	// Wait until transaction is complete or timed out 
	while ((twiMaster.status != TWIM_STATUS_READY) &&  NOT_TIMEDOUT());
	
	twiMaster.status= TWIM_STATUS_READY;
	
	if (gMsTimeout)
		return true;
	else
	  	return false;	
}


/*============================================================================
Name    :   TWI_MasterInterruptHandler
------------------------------------------------------------------------------
Purpose :   Handles TWI data after a TWI interrupt has occured
Input   :   n/a
Output  :   n/a
Notes   :	TWI Master Interrupt handler
============================================================================*/
ISR(TWIC_TWIM_vect)
{
	TWI_MasterInterruptHandler();
}

void TWI_MasterInterruptHandler(void)
{
	uint8_t currentStatus = twiMaster.interface->MASTER.STATUS;

	// If arbitration lost or bus error. 
	if ((currentStatus & TWI_MASTER_ARBLOST_bm) ||
	    (currentStatus & TWI_MASTER_BUSERR_bm))
	{
		TWI_MasterArbitrationLostBusErrorHandler();
	}

	// If master write interrupt. 
	else if (currentStatus & TWI_MASTER_WIF_bm) {
		TWI_MasterWriteHandler();
	}

	// If master read interrupt.
	else if (currentStatus & TWI_MASTER_RIF_bm) {
		TWI_MasterReadHandler();
	}

	// If unexpected state. 
	else {
		TWI_MasterTransactionFinished(TWIM_RESULT_FAIL);
	}
}


/*============================================================================
Name    :   TWI_MasterArbitrationLostBusErrorHandler
------------------------------------------------------------------------------
Purpose :   Take action if bus error
Input   :   n/a
Output  :   n/a
Notes   :	
============================================================================*/
void TWI_MasterArbitrationLostBusErrorHandler(void)
{
	uint8_t currentStatus = twiMaster.interface->MASTER.STATUS;

	// If bus error.
	if (currentStatus & TWI_MASTER_BUSERR_bm) {
		twiMaster.result = TWIM_RESULT_BUS_ERROR;
	}
	// If arbitration lost.
	else {
		twiMaster.result = TWIM_RESULT_ARBITRATION_LOST;
	}

	// Clear interrupt flag. 
	twiMaster.interface->MASTER.STATUS = currentStatus | TWI_MASTER_ARBLOST_bm;

	twiMaster.status = TWIM_STATUS_READY;
}


/*============================================================================
Name    :   TWI_MasterWriteHandler
------------------------------------------------------------------------------
Purpose :   Handles TWI 
Input   :   n/a
Output  :   n/a
Notes   :	Called from TWI Interrupt routine
============================================================================*/
void TWI_MasterWriteHandler(void)
{
	// Local variables used in if tests to avoid compiler warning. 
	uint16_t bytesToWrite  = twiMaster.bytesToWrite;
	//uint16_t bytesToRead   = twiMaster.bytesToRead;

	// If NOT acknowledged (NACK) by slave cancel the transaction.
	if (twiMaster.interface->MASTER.STATUS & TWI_MASTER_RXACK_bm) {
		twiMaster.interface->MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
		twiMaster.result = TWIM_RESULT_NACK_RECEIVED;
		twiMaster.status = TWIM_STATUS_READY;
	}

	// If more bytes to write, send data. 
	else if (twiMaster.bytesWritten <= bytesToWrite)
	{
		uint8_t data = TX_Buffer[twiMaster.bytesWritten];
		twiMaster.interface->MASTER.DATA = data;
		++twiMaster.bytesWritten;
	}

	// If transaction finished, send STOP condition and set RESULT OK. 
	else {
		twiMaster.interface->MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
		TWI_MasterTransactionFinished(TWIM_RESULT_OK);
	}
}

/*============================================================================
Name    :   TWI_MasterReadHandler
------------------------------------------------------------------------------
Purpose :   Handles TWI 
Input   :   n/a
Output  :   n/a
Notes   :	Called from TWI Interrupt routine
============================================================================*/
void TWI_MasterReadHandler(void)
{
	uint8_t FrameInProgress;
	
  	uint8_t data = twiMaster.interface->MASTER.DATA;
	FrameInProgress = RxHandler(data);
	
  	// If more bytes to read, issue ACK and start a byte read. 
	if (FrameInProgress)
	{
		twiMaster.interface->MASTER.CTRLC = TWI_MASTER_CMD_RECVTRANS_gc;
	}	
	else// If transaction finished, issue NACK and STOP condition. 
	{
		twiMaster.interface->MASTER.CTRLC = TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc;
		TWI_MasterTransactionFinished(TWIM_RESULT_OK);
	}
}

/*============================================================================
Name    :   TWI_MasterTransactionFinished
------------------------------------------------------------------------------
Purpose :   Updates TWI status variables 
Input   :   n/a
Output  :   n/a
Notes   :	Called from TWI Interrupt routine
=============================================================================*/
void TWI_MasterTransactionFinished(uint8_t result)
{
	twiMaster.result = result;
	twiMaster.status = TWIM_STATUS_READY;
}
#endif




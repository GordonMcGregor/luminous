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
#include <avr/io.h>
#include <avr/interrupt.h>
#include "TWI_Master.h"
#include "QDebugTransport.h"


/*============================ LOCAL VARIABLES ==============================*/
static volatile unsigned char TWI_addr;    			// Transceiver buffer
static volatile unsigned int TWI_msgSize;            // Number of bytes to be transmitted.

volatile union TWI_statusReg TWI_statusReg = {0};    // TWI_statusReg is defined in TWI_Master.h

/*============================ GLOBAL VARIABLES ================================*/

unsigned int gMsTimeout;
#define NOT_TIMEDOUT()					(gMsTimeout)
#define SET_TIMEOUT(timeout)			(gMsTimeout = timeout)


/*============================================================================
Name    :   TWI_Master_Init
------------------------------------------------------------------------------
Purpose :   Initialize the TWI
Input   :   n/a
Output  :   n/a
Notes   :   Called from QDebug_Init in QDebug.c
============================================================================*/
void TWI_Master_Init(void)
{
    // This driver is modified for the QT600-TINY88-QT8 board.
    // If using the driver for another tinyAVR or megaAVR device the registry setup may have to be modified.

  	TWHSR = 0; 									// Use Normal mode: Fscl=110KHz
  	TWSR = 	0;                         			// Set Prescaler to be 1 .
	TWBR = 10;                					// Bit rate Fscl=100KHz/220KHz.
  	TWDR = 0xFF;                              	// Default content = SDA released.
  	TWCR = (1<<TWEN)|                         	// Enable TWI-interface and release TWI pins.
         	(0<<TWIE)|(0<<TWINT)|              	// Disable Interupt.
         	(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|   	// No Signal requests.
         	(0<<TWWC);                         	//
}


/*============================================================================
Name    :   TWI_Transceiver_Busy
------------------------------------------------------------------------------
Purpose :   Test if the TWI_ISR is busy transmitting
Input   :   n/a
Output  :   returns true if the TWI is busy transmitting
Notes   :   Called from TWI transmitter functions
============================================================================*/
unsigned char TWI_Transceiver_Busy( void )
{
	// IF TWI Interrupt is enabled then the Transceiver is busy
  	return ( TWCR & (1<<TWIE) );
}

/*============================================================================
Name    :   TWI_Send_Message
------------------------------------------------------------------------------
Purpose :   Send the frame stored in TX_Buffer to USB Bridge
Input   :   n/a
Output  :   n/a
Notes   :   Called from Send_Message in QDebugTransport.c
============================================================================*/
void TWI_Send_Message( void )
{
    SET_TIMEOUT(100);	// 100ms timeout

    TWI_msgSize = ((TX_Buffer[1]<<8)+TX_Buffer[2]);

    TWI_addr  =(TWISLAVE_ADDR<<TWI_ADR_BITS) | (FALSE<<TWI_READ_BIT);

    TWI_statusReg.all = 0;

    TWCR = (1<<TWEN)|                    		// TWI Interface enabled.
         (1<<TWIE)|(1<<TWINT)|              // Enable TWI Interupt and clear the flag.
         (0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|   // Initiate a START condition.
         (0<<TWWC);

  	while (TWI_Transceiver_Busy()&& NOT_TIMEDOUT());// Wait until TWI is ready for next transmission.
}

/*============================================================================
Name    :   TWI_Send_Message
------------------------------------------------------------------------------
Purpose :   Read one frame from USB Bridge. Store the frame in RX_Buffer
Input   :   n/a
Output  :   n/a
Notes   :   Called from Receive_Message in QDebugTransport.c
============================================================================*/
void TWI_Retrieve_Message( void )
{
  	unsigned int i;

   	SET_TIMEOUT(100);	// 100ms timeout

  	TWI_addr  = (TWISLAVE_ADDR<<TWI_ADR_BITS) | (TRUE<<TWI_READ_BIT);

  	TWI_statusReg.all = 0;

  	TWCR = 	(1<<TWEN)|                             // TWI Interface enabled.
         	(1<<TWIE)|(1<<TWINT)|                  // Enable TWI Interupt and clear the flag.
         	(0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|       // Initiate a START condition.
         	(0<<TWWC);                             //

  	while (TWI_Transceiver_Busy()&& NOT_TIMEDOUT());// Wait until TWI is ready for next transmission.

	if( ! TWI_statusReg.lastTransOK )               // Last transmission completed successfully.
		for ( i=0; i<=TWI_msgSize; i++ )           	// Copy data from Transceiver buffer.
		  	RX_Buffer[i] = 0;						// Clear data if transmission not successful
}

/*============================================================================
Name    :   TWI_ISR
------------------------------------------------------------------------------
Purpose :   This function is the Interrupt Service Routine (ISR), and called
			when the TWI interrupt is triggered;
Input   :   n/a
Output  :   n/a
Notes   :   Executes automaticly when a TWI event has occured
============================================================================*/

ISR(TWI_vect)
{
  static unsigned int TWI_bufPtr;
  volatile unsigned char FrameInProgress;

  switch (TWSR)
  {
  case TWI_START:             	// START has been transmitted

  case TWI_REP_START:         	// Repeated START has been transmitted
    TWI_bufPtr = 0;         	// Set buffer pointer to the TWI Address location
    TWDR = TWI_addr;
	TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC);
	break;

  case TWI_MTX_ADR_ACK:       	// SLA+W has been tramsmitted and ACK received
  case TWI_MTX_DATA_ACK:      	// Data byte has been transmitted and ACK received

	if (TWI_bufPtr <= TWI_msgSize)
    {
        TWDR = TX_Buffer[TWI_bufPtr++];
		TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC);
    }
	else                    // Send STOP after last byte
    {
        TWI_statusReg.lastTransOK = TRUE;                 // Set status bits to completed successfully.
		TWCR = (1<<TWEN)|(0<<TWIE)|(1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|(0<<TWWC);
	}
    break;

  case TWI_MRX_ADR_ACK:       // SLA+R has been transmitted and ACK received

	TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC);
	break;

  case TWI_MRX_DATA_ACK:      // Data byte has been received and ACK transmitted

	FrameInProgress=RxHandler(TWDR);

	if (FrameInProgress )                  	// Detect the last byte to NACK it.
	{
		// Send ACK after reception
		TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC);
	}
	else
	{
		//TWI_Protocol_State=TWI_STATE_MSG_START;			// TWI ready to receive new packet
		// Send NACK after reception
		TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC);
	}

	break;

  case TWI_MRX_DATA_NACK:     // Data byte has been received and NACK tramsmitted

	RxHandler(TWDR);

    TWI_statusReg.lastTransOK = TRUE;                 // Set status bits to completed successfully.
	// Initiate a STOP condition.
	TWCR = (1<<TWEN)|(0<<TWIE)|(1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|(0<<TWWC);
	break;

  case TWI_ARB_LOST:          // Arbitration lost
	// Initiate a (RE)START condition.
	TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|(0<<TWWC);
	break;

  	case TWI_MTX_ADR_NACK:      // SLA+W has been transmitted and NACK received
    case TWI_MRX_ADR_NACK:      // SLA+R has been transmitted and NACK received
    case TWI_MTX_DATA_NACK:     // Data byte has been tramsmitted and NACK received
//    case TWI_NO_STATE              // No relevant state information available; TWINT = “0”

  case TWI_BUS_ERROR:         // Bus error due to an illegal START or STOP condition
    default:
                                                        // Reset TWI Interface
      TWCR = (1<<TWEN)|                                 // Enable TWI-interface and release TWI pins
             (0<<TWIE)|(0<<TWINT)|                      // Disable Interupt
             (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // No Signal requests
             (0<<TWWC);                                 //
  }
}
#endif /* _DEBUG_INTERFACE_ */


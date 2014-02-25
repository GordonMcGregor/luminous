/*******************************************************************************
*   $FILE:  SPI_Master.c
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

#if defined(_DEBUG_INTERFACE_) && defined(QDEBUG_SPI)
#include "avr_compiler.h"
#include "SPI_Master.h"
#include "QDebugTransport.h"

/*============================ IMPLEMENTATION ================================*/
#define JOIN1( A, B, C ) A ## B ## C
#define CONCAT( A, B, C ) JOIN1( A, B, C )
/*============================================================================
Name    :   SPI_Master_Init
------------------------------------------------------------------------------
Purpose :   Initialize SPI Interface
Input   :   n/a
Output  :   n/a
Notes   :	Called from QDebug_Init in QDebug.c
============================================================================*/
void SPI_Master_Init (void)
{
    // MOSI, SCK and SS are outputs. Pullup on MISO
    //SPI_DDR |= (1<<SS) | (1<<MOSI) | (1<<SCK) ;
    CONCAT( PORT, SPI_SS, _DIR) =  (1<<SS) | (1<<MOSI) | (1<<SCK) ;

    //SPI_DDR &=~(1<<MISO);
    CONCAT( PORT, SPI_SS, _DIR) &=~(1<<MISO);

   //SPI_PORT &=~ ((1<<SS) | (1<<SCK) | (1<<MOSI)) ;
    CONCAT( PORT, SPI_SS, _OUT)  &=~ ((1<<SS) | (1<<SCK) | (1<<MOSI)) ;

	//SPI_PORT |= (1<<MISO);
    CONCAT( PORT, SPI_SS, _OUT) |= (1<<MISO);

  	SPIC_CTRL = (1<<SPI_ENABLE_bp) | (1<<SPI_MASTER_bp) |
		(0<<SPI_PRESCALER1_bp)|(1<<SPI_PRESCALER0_bp);
}

/*============================================================================
Name    :   SPI_Send_Byte
------------------------------------------------------------------------------
Purpose :   Send and Read one byte using SPI Interface
Input   :   Data to send to slave
Output  :   Data read from slave
Notes   :	Called from SPI_Send_Message in this file
============================================================================*/
uint8_t SPI_Send_Byte(uint8_t c)
{
    uint8_t data;

    SPIC_DATA = c;   			        		// Write byte to SPI register
    while (!(SPIC_STATUS & (1<<SPI_IF_bp)));    // Wait until byte is sent
    data = SPIC_DATA;         					// Read byte from SPI register

   	delay_us(50);

    return data;
}


/*============================================================================
Name    :   SPI_Send_Message
------------------------------------------------------------------------------
Purpose :   Send and Read one frame using SPI Interface
Input   :   n/a
Output  :   n/a
Notes   :	Called from Send_Message in QDebugTransport.c
============================================================================*/
void SPI_Send_Message(void)
{
    unsigned int i;
	volatile uint8_t FrameInProgress;

	// Send our message upstream
    for (i=0; i <= TX_index; i++)
    {
		FrameInProgress = RxHandler(SPI_Send_Byte(TX_Buffer[i]));
    }

	// Do we need to receive even more bytes?
	while (FrameInProgress)
		FrameInProgress = RxHandler(SPI_Send_Byte(0));
}
#endif

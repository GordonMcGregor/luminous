/*
 * \file
 *
 * \brief 
 *
 * Copyright (C) 2009 Atmel Corporation. All rights reserved.
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 * Atmel AVR product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

#ifndef TWI_MASTER_H_INCLUDED
#define TWI_MASTER_H_INCLUDED

/*============================ INCLUDES ======================================*/
#include "avr_compiler.h"

/*============================ MACROS ========================================*/
//! Baud register setting calculation. Formula described in xmega datasheet.
#define TWI_BAUD(F_SYS, F_TWI) ((F_SYS / (2 * F_TWI)) - 5)

#define CPU_SPEED   8000000
#define BAUDRATE	100000
#define TWI_BAUDSETTING TWI_BAUD(CPU_SPEED, BAUDRATE)
#define SLAVE_ADDRESS    0x10


#define TWIM_STATUS_READY              0
#define TWIM_STATUS_BUSY               1

/*============================ TYPES ========================================*/
typedef enum TWIM_RESULT_enum {
	TWIM_RESULT_UNKNOWN          = (0x00<<0),
	TWIM_RESULT_OK               = (0x01<<0),
	TWIM_RESULT_BUFFER_OVERFLOW  = (0x02<<0),
	TWIM_RESULT_ARBITRATION_LOST = (0x03<<0),
	TWIM_RESULT_BUS_ERROR        = (0x04<<0),
	TWIM_RESULT_NACK_RECEIVED    = (0x05<<0),
	TWIM_RESULT_FAIL             = (0x06<<0),
} TWIM_RESULT_t;


/*! \brief TWI master driver struct
 *
 *  TWI master struct. Holds pointer to TWI module,
 *  buffers and necessary varibles.
 */
typedef struct TWI_Master {
	TWI_t *interface;               /*!< Pointer to what interface to use */
	register8_t address;            /*!< Slave address */
	register16_t bytesToWrite;    	/*!< Number of bytes to write */
	register16_t bytesToRead;     	/*!< Number of bytes to read */
	register16_t bytesWritten;		/*!< Number of bytes written */
	register16_t bytesRead;         /*!< Number of bytes read */
	register8_t status;             /*!< Status of transaction */
	register8_t result;             /*!< Result of transaction */
}TWI_Master_t;


/*============================ PROTOTYPES ====================================*/
void TWI_Master_Init(void);
bool TWI_Send_Message(void);
bool TWI_Retrieve_Message(void);
void TWI_MasterInterruptHandler(void);
void TWI_MasterArbitrationLostBusErrorHandler(void);
void TWI_MasterWriteHandler(void);
void TWI_MasterReadHandler(void);
void TWI_MasterTransactionFinished(uint8_t result);

#endif
/* EOF */

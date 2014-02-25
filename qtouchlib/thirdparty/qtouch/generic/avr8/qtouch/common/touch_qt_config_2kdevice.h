/*******************************************************************************
*   Atmel Corporation:  http://www.atmel.com
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


#ifndef _TOUCH_CONFIG_H_
#define _TOUCH_CONFIG_H_

#include "touch.h"


/**************************************************************/
/* Please do not change any part of the code below this line. */
/**************************************************************/
#if (QT_DELAY_CYCLES == 0)
#error 'QT_DELAY_CYCLES can only have values in the range 1-255.'
#endif

#if !(defined(__AVR32__) || defined(__ICCAVR32__))
	#if ((QT_NUM_CHANNELS == 1) || (QT_NUM_CHANNELS == 2)|| (QT_NUM_CHANNELS == 4))
		#ifdef _SNS_SNSK_SAME_PORT_
			#define INTRABURST 1
		#else
                     
                        #define INTRABURST 0
		#endif
			#define INTRABURST_1 0
                        #define INTRABURST_2 0
                        #define QTOUCH_SNS_PORT_COUNT 1
			#define _STATIC_PORT_PIN_CONF_ 0
	#else
	#error 'QT_NUM_CHANNELS specified is not supported.'
	#endif
#else

#endif

#if (defined(__IAR_SYSTEMS_ASM__) || defined(__ASSEMBLER__) || defined(__ICCAVR32__) || defined(__AVR32__))
    #if (defined(__IAR_SYSTEMS_ASM__) || defined(__ICCAVR32__))
        #if defined(__IAR_SYSTEMS_ASM__)
            #include <ioavr.h>
            #define p_1 r16
            #define p_2 r17
            #define p_3 r18
            #define p_4 r19
            #define p_5 r20
            #define r_v r16
            #define FILE_HEADER NAME qt_asm_iar
            #define FILE_FOOTER END
            #define FILE_SEGMENT RSEG CODE
            #define GLOBAL_FUNCTION PUBLIC

            #define GLOBAL_VAR EQU
        #elif defined(__ICCAVR32__) 
            #include <avr32/io.h>
            #define p_1 r12
            #define p_2 r11
            #define p_3 r10
            #define r_v r12
            #define _UC3A_
           // #define _DEBUG_INTERFACE_
            #define _QTOUCH_
            //#define FILE_HEADER NAME qt_asm_iar
            //#define FILE_FOOTER END
            //#define FILE_SEGMENT RSEG CODE
            #define FILE_HEADER
            #define FILE_FOOTER
            #define FILE_SEGMENT
            #define GLOBAL_FUNCTION PUBLIC
        #endif
    #elif (defined(__ASSEMBLER__) || defined(__AVR32__))
        #if defined(__AVR32__)
            #include <avr32/io.h>

            #define p_1 r12
            #define p_2 r11
            #define p_3 r10
            #define r_v r12

            //#define _DEBUG_INTERFACE_
            #define _QTOUCH_
            #define _UC3A_
            #define _STATIC_PORT_PIN_CONF_ 0
            #define QTOUCH_SNS_PORT_COUNT 1
            #define INTRABURST_1 0
            #define INTRABURST_2 0
        #else
            #define __SFR_OFFSET 0
            #include <avr/io.h>   
            #define p_1 r24
            #define p_2 r22
            #define p_3 r20
            #define p_4 r18
            #define p_5 r16
            #define r_v r24
            
            #define GLOBAL_VAR .EQU
        #endif
    
        #define FILE_HEADER
        #define FILE_FOOTER 
        #define FILE_SEGMENT 
        #define GLOBAL_FUNCTION .global
    #else    
        #error 'Assembler not supported.'
    #endif
    
    #define JOIN( x, y ) x ## y
    #define JOIN1( A, B, C ) A ## B ## C
    #define REG( REGISTER, SIDE ) JOIN( REGISTER, SIDE )
    #define CONCAT( A, B, C ) JOIN1( A, B, C )
    
    #define sreg_save r2
    
    #define _00011001_	nop
    #define _01101001_	brne _111_
    #define _01101011_	brne _222_
    #define _10001110_	rjmp . 
    #if (defined (__AVR32__) || defined (__ICCAVR32__))
        #define _10100011_	_111_: sub r9,1
        #define _10100111_	_222_: sub r9,1
        #define _11100011_	mov r9,((QT_DELAY_CYCLES - 1)/3)
    #else
        #define _10100011_	_111_: dec r19
        #define _10100111_	_222_: dec r19
        #define _11100011_	ldi r19,((QT_DELAY_CYCLES - 1)/3)
    #endif

#else
    #ifndef _DEBUG_INTERFACE_
        /*#define _DEBUG_INTERFACE_*/
    #endif
    #ifndef _QTOUCH_
        #define _QTOUCH_
    #endif
#endif

#endif /*_TOUCH_CONFIG_H_*/

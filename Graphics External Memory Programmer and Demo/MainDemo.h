/*********************************************************************
 * Microchip Graphic Library Demo Application
 * The header file joins all header files used in the project.
 *********************************************************************
 * FileName:        MainDemo.h
 * Dependencies:    See INCLUDES section below
 * Processor:       PIC24F, PIC24H, dsPIC, PIC32
 * Compiler:        C30/C32
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * Copyright � 2008 Microchip Technology Inc.  All rights reserved.
 * Microchip licenses to you the right to use, modify, copy and distribute
 * Software only when embedded on a Microchip microcontroller or digital
 * signal controller, which is integrated into your product or third party
 * product (pursuant to the sublicense terms in the accompanying license
 * agreement).  
 *
 * You should refer to the license agreement accompanying this Software
 * for additional information regarding your rights and obligations.
 *
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED �AS IS� WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY
 * OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR
 * PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR
 * OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,
 * BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT
 * DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL,
 * INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA,
 * COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY
 * CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
 * OR OTHER SIMILAR COSTS.
 *
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Anton Alkhimenok		05/29/2007
 ********************************************************************/
#ifndef _MAINDEMO_H
    #define _MAINDEMO_H

////////////////////////////// INCLUDES //////////////////////////////
    #if defined(__dsPIC33F__)
        #include <p33Fxxxx.h>
    #elif defined(__PIC24H__)
        #include <p24Hxxxx.h>
    #elif defined(__PIC32MX__)
        #include <plib.h>
    #else
        #include <p24Fxxxx.h>
    #endif
    #include "GenericTypeDefs.h"
    #include "Graphics/Graphics.h"
    #include "FlashProgrammer.h"
#endif

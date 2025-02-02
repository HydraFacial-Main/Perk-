/*****************************************************************************
 * Microchip Graphics Library Demo Application
 * This program shows how to use the external memory to store fonts and bitmaps.
 * 
 * To run this demo:
 * external.hex file in "Fonts and Bitmaps" folder must be programmed into external
 * flash memory installed on Graphics PICTail Plus board. To do this
 * "Graphics PICTail Board Memory Programmer" project can be used. Please refer to
 * "Getting started -> Graphics External Memory Demo" section in the help file for more information.
 *
 * To create a new hex file containing data for font and bitmap to be located in
 * the external flash memory use 'Bitmap and Font converter.exe' utility included in the Graphics Library.
 * With hex file the utility will create a C file containing structures for referencing
 * the font and bitmap in the hex file. This C file must be included in the project.
 * Please refer to the help built in the utility.
 *
 * In applications with external memory USE_BITMAP_EXTERNAL and USE_FONT_EXTERNAL
 * must be defined in GraphicsConfig.h. 
 *
 *****************************************************************************
 * FileName:        MainDemo.c
 * Dependencies:    MainDemo.h
 * Processor:       PIC24F, PIC24H, dsPIC, PIC32
 * Compiler:       	MPLAB C30, MPLAB C32
 * Linker:          MPLAB LINK30, MPLAB LINK32
 * Company:         Microchip Technology Incorporated
 *
 * Software License Agreement
 *
 * Copyright � 2010 Microchip Technology Inc.  All rights reserved.
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
 * Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 09/01/10	   ...
 *****************************************************************************/
#include "MainDemo.h"
//#include "SST25VF016.h"   
//#include "TouchScreen.h"

// Configuration bits
#if defined(__dsPIC33F__) || defined(__PIC24H__)
	_FOSCSEL(FNOSC_PRI);
	_FOSC(FCKSM_CSECMD &OSCIOFNC_OFF &POSCMD_XT);
	_FWDT(FWDTEN_OFF);
#elif defined(__PIC32MX__)
	#pragma config FPLLODIV = DIV_2, FPLLMUL = MUL_20, FPLLIDIV = DIV_1, FWDTEN = OFF, FCKSM = CSECME, FPBDIV = DIV_8
	#pragma config OSCIOFNC = ON, POSCMOD = XT, FSOSCEN = ON, FNOSC = PRIPLL
	#pragma config CP = OFF, BWP = OFF, PWP = OFF    
#else
    #if defined(__PIC24FJ256GB110__)
_CONFIG1(JTAGEN_OFF & GCP_OFF & GWRP_OFF & COE_OFF & FWDTEN_OFF & ICS_PGx2)
_CONFIG2(0xF7FF & IESO_OFF & FCKSM_CSDCMD & OSCIOFNC_OFF & POSCMOD_HS & FNOSC_PRIPLL & PLLDIV_DIV2 & IOL1WAY_OFF)
    #endif
    #if defined(__PIC24FJ256GA110__)
_CONFIG1(JTAGEN_OFF & GCP_OFF & GWRP_OFF & COE_OFF & FWDTEN_OFF & ICS_PGx2)
_CONFIG2(IESO_OFF & FCKSM_CSDCMD & OSCIOFNC_OFF & POSCMOD_HS & FNOSC_PRIPLL & IOL1WAY_OFF)
    #endif
    #if defined(__PIC24FJ128GA010__)
_CONFIG2(FNOSC_PRIPLL & POSCMOD_XT) // Primary XT OSC with PLL
_CONFIG1(JTAGEN_OFF & FWDTEN_OFF)   // JTAG off, watchdog timer off
    #endif
	#if defined (__PIC24FJ256GB210__)
_CONFIG1( WDTPS_PS32768 & FWPSA_PR128 & ALTVREF_ALTVREDIS & WINDIS_OFF & FWDTEN_OFF & ICS_PGx2 & GWRP_OFF & GCP_OFF & JTAGEN_OFF) 
_CONFIG2( POSCMOD_HS & IOL1WAY_OFF & OSCIOFNC_OFF & OSCIOFNC_OFF & FCKSM_CSDCMD & FNOSC_PRIPLL & PLL96MHZ_ON & PLLDIV_DIV2 & IESO_OFF)
_CONFIG3( WPFP_WPFP255 & SOSCSEL_SOSC & WUTSEL_LEG & WPDIS_WPDIS & WPCFG_WPCFGDIS & WPEND_WPENDMEM) 
	#endif
	#if defined (__PIC24FJ256DA210__)
_CONFIG1( WDTPS_PS32768 & FWPSA_PR128 & ALTVREF_ALTVREDIS & WINDIS_OFF & FWDTEN_OFF & ICS_PGx2 & GWRP_OFF & GCP_OFF & JTAGEN_OFF) 
_CONFIG2( POSCMOD_HS & IOL1WAY_OFF & OSCIOFNC_OFF & OSCIOFNC_OFF & FCKSM_CSDCMD & FNOSC_PRIPLL & PLL96MHZ_ON & PLLDIV_DIV2 & IESO_OFF)
_CONFIG3( WPFP_WPFP255 & SOSCSEL_SOSC & WUTSEL_LEG & ALTPMP_ALTPMPEN & WPDIS_WPDIS & WPCFG_WPCFGDIS & WPEND_WPENDMEM) 
	#endif	
#endif

///////////////////////////// FONTS AND BITMAPS ///////////////////////////////
// This font is located in internal flash
extern const FONT_FLASH internalfont;

// This picture is located in internal flash
extern const IMAGE_FLASH   internalbitmap;

// This font must be stored in external flash memory installed on
// Graphics PICTail Plus board
extern FONT_EXTERNAL        externalfont;

// This bitmap must be stored in external flash memory installed on
// Graphics PICTail Plus board
//extern IMAGE_EXTERNAL      externalbitmap;

extern IMAGE_EXTERNAL Splash;
extern IMAGE_EXTERNAL HomeI;
extern IMAGE_EXTERNAL HF_Mode;
extern IMAGE_EXTERNAL Home;
extern IMAGE_EXTERNAL Aux_Mode;
extern IMAGE_EXTERNAL LED_Mode;
extern IMAGE_EXTERNAL Clean_Mode;
extern IMAGE_EXTERNAL Yes;
extern IMAGE_EXTERNAL Stop_gray;
extern IMAGE_EXTERNAL Stop;
extern IMAGE_EXTERNAL Start_gray;
extern IMAGE_EXTERNAL Start;
extern IMAGE_EXTERNAL Standby;
extern IMAGE_EXTERNAL Reset_gray;
extern IMAGE_EXTERNAL Reminder;
extern IMAGE_EXTERNAL pumpO;
extern IMAGE_EXTERNAL pumpF;
extern IMAGE_EXTERNAL plusP;
extern IMAGE_EXTERNAL ON_gray;
extern IMAGE_EXTERNAL OFF_gray;
extern IMAGE_EXTERNAL No;
extern IMAGE_EXTERNAL minusM;
extern IMAGE_EXTERNAL Days;
extern IMAGE_EXTERNAL Code_Entry;
/************************************************************************
* Macros: SST39PMPInit()
*                                                                       
* Overview: initializes PMP only
*                                                                       
* Input: none                                                          
*                                                                       
* Output: none
*                                                                       
************************************************************************/
#define SST39PMPInit()          \
    {                           \
        while(PMMODEbits.BUSY); \
        PMMODE = 0x0a49;        \
        PMAEN = 0x0003;         \
        PMCON = 0x9320;         \
    }

/************************************************************************
* Macros: LCDPMPInit()
*                                                                       
* Overview: initializes PMP only
*                                                                       
* Input: none                                                          
*                                                                       
* Output: none
*                                                                       
************************************************************************/
#define LCDPMPInit()            \
    {                           \
        while(PMMODEbits.BUSY); \
        PMMODE = 0x0204;        \
        PMAEN = 0x0000;         \
        PMCON = 0x8300;         \
    }

#define DEMODELAY 	2000

/* */

int main(void)
{
    SHORT   width, height;

    #if defined (PIC24FJ256DA210_DEV_BOARD)
    
	    _ANSG8 = 0; /* S1 */
	    _ANSE9 = 0; /* S2 */
	    _ANSB5 = 0; /* S3 */
        
    #else
    /////////////////////////////////////////////////////////////////////////////
    // ADC Explorer 16 Development Board Errata (work around 2)
    // RB15 should be output
    /////////////////////////////////////////////////////////////////////////////
    #ifndef MULTI_MEDIA_BOARD_DM00123
	    LATBbits.LATB15 = 0;
	    TRISBbits.TRISB15 = 0;
    #endif
    #endif

    /////////////////////////////////////////////////////////////////////////////
    #if defined(__dsPIC33F__) || defined(__PIC24H__)

	    // Configure Oscillator to operate the device at 40Mhz
	    // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
	    // Fosc= 8M*40(2*2)=80Mhz for 8M input clock
	    PLLFBD = 38;                    // M=40
	    CLKDIVbits.PLLPOST = 0;         // N1=2
	    CLKDIVbits.PLLPRE = 0;          // N2=2
	    OSCTUN = 0;                     // Tune FRC oscillator, if FRC is used
	
	    // Disable Watch Dog Timer
	    RCONbits.SWDTEN = 0;
	
	    // Clock switching to incorporate PLL
	    __builtin_write_OSCCONH(0x03);  // Initiate Clock Switch to Primary
	
	    // Oscillator with PLL (NOSC=0b011)
	    __builtin_write_OSCCONL(0x01);  // Start clock switching
	    while(OSCCONbits.COSC != 0b011);
	
	    // Wait for Clock switch to occur	
	    // Wait for PLL to lock
	    while(OSCCONbits.LOCK != 1)
	    { };
	    
    #elif defined(__PIC32MX__)
    
	    INTEnableSystemMultiVectoredInt();
    	SYSTEMConfig(GetSystemClock(), SYS_CFG_WAIT_STATES|SYS_CFG_PCACHE);
    	
    #endif

    #if defined(__dsPIC33FJ128GP804__) || defined(__PIC24HJ128GP504__)
    
    	AD1PCFGL = 0xffff;
    	
    #endif

    #if defined (GFX_PICTAIL_V3) || defined (PIC24FJ256DA210_DEV_BOARD)

    // Initialize graphics
    InitGraph();
    // Initialize flash installed on Graphics PICTail Plus board 3
    FLASHInit();

    #else

    // Initialize IOs related to parallel flash memory
    // installed on Graphics PICTail Plus board 2
    FLASHInit();
    // Initialize graphics
    InitGraph();

    #endif

  //  SST25Init();                    // initialize GFX3 SST25 flash SPI

    //TouchInit();                    // initialize touch screen
 //   TickInit();                     // initialize tick counter (for random number generation)

//    TouchCalibration();
//    TouchStoreCalibration();
/*
    if(GRAPHICS_LIBRARY_VERSION != SST25ReadWord(ADDRESS_VERSION))
    {
        TouchCalibration();
        TouchStoreCalibration();
    }
*/
    // Load touch screen calibration parameters from memory
  //  TouchLoadCalibration();

	// check if programming is prompted    
    if(GetHWButtonProgram() == HW_BUTTON_PRESS)
    {
	    // The ProgramFlash() function will not return. 
	    // Application must be reset after programming.
	    ProgramFlash();
	} 

    while(1)
    {

        // Display text with font located in internal flash
        // Set current font
        SetFont((void *) &internalfont);

        // Get text width and height
        width = GetTextWidth("Font from internal flash", (void *) &internalfont);
        height = GetTextHeight((void *) &internalfont);

        // Put green text in the center
        SetColor(BRIGHTGREEN);
        OutTextXY((GetMaxX() - width) >> 1, (GetMaxY() - height) >> 1, "Font from internal flash");

        // Wait a couple of seconds and clean screen
        DelayMs(DEMODELAY);
        SetColor(BLACK);
        ClearDevice();

        // Display text with font located in external memory
        // Set current font
        SetFont((void *) &externalfont);

        // Get text width and height
        width = GetTextWidth("Font from external flash", (void *) &externalfont);
        height = GetTextHeight((void *) &externalfont);

        // Put red text in the center
        SetColor(BRIGHTRED);
        OutTextXY((GetMaxX() - width) >> 1, (GetMaxY() - height) >> 1, "Font from external flash");

        // Wait a couple of seconds and clean screen
        DelayMs(DEMODELAY);
        SetColor(BLACK);
        ClearDevice();

		// reset back the font to use internal font
        SetFont((void *) &internalfont);

        // Display picture located in internal memory
        // Get bitmap width and height
        width = GetImageWidth((void *) &internalbitmap);
        height = GetImageHeight((void *) &internalbitmap);

        // Put bitmap in the center
        PutImage((GetMaxX() - width) >> 1, (GetMaxY() - height) >> 1, (void *) &internalbitmap, 1);

        // Display some white text
        SetColor(WHITE);
        OutTextXY(0, 0, "Bitmap from internal flash");

        // Wait a couple of seconds and clean screen
        DelayMs(DEMODELAY);
        SetColor(BLACK);
        ClearDevice();

        // Display picture located in external memory
        // Get bitmap width and height
//        width = GetImageWidth((void *) &externalbitmap);
 //       height = GetImageHeight((void *) &externalbitmap);

        // Put bitmap in the center
//        PutImage((GetMaxX() - width) >> 1, (GetMaxY() - height) >> 1, (void *) &externalbitmap, 1);
        PutImage(0, 0, (void *) &Splash, 1);
/*
        // Display some white text
        SetColor(WHITE);
        OutTextXY(0, 0, "Bitmap from external flash");
*/
        // Wait a couple of seconds and clean screen
        DelayMs(DEMODELAY);
        SetColor(BLACK);
        ClearDevice();

        PutImage(0, 0, (void *) &HomeI, 1);

        DelayMs(DEMODELAY);
        SetColor(BLACK);
        ClearDevice();

        PutImage(0, 0, (void *) &Home, 1);

        DelayMs(DEMODELAY);
        SetColor(BLACK);
        ClearDevice();

        PutImage(0, 0, (void *) &HF_Mode, 1);

        DelayMs(DEMODELAY);
        SetColor(BLACK);
        ClearDevice();

        PutImage(0, 0, (void *) &Aux_Mode, 1);

        DelayMs(DEMODELAY);
        SetColor(BLACK);
        ClearDevice();

        PutImage(0, 0, (void *) &LED_Mode, 1);

    }
}

/*********************************************************************
* Function: WORD ExternalMemoryCallback(EXTDATA* memory, LONG offset, WORD nCount, void* buffer)
*
* PreCondition: none
*
* Input:  memory - pointer to the bitmap or font external memory structures
*                  (FONT_EXTERNAL or BITMAP_EXTERNAL)
*         offset - data offset
*         nCount - number of bytes to be transferred to the buffer
*         buffer - pointer to the buffer
*
* Output: number of bytes were transferred.
*
* Side Effects: none
*
* Overview: this function must be implemented in application. Graphics Library calls it
*           each time the data from external memory is required. The application must copy 
*           required data to the buffer provided.
*
* Note: none
*
********************************************************************/

// If there are several memories in the system they can be selected by IDs.
// In this demo ID for memory chip installed on Graphics PICTail board is assumed to be 0.
#define SST39_MEMORY    0

/* */

WORD ExternalMemoryCallback(IMAGE_EXTERNAL *memory, LONG offset, WORD nCount, void *buffer)
{
    if(memory->ID == SST39_MEMORY)
    {

        // Read data requested into buffer provided
        #if defined (GFX_PICTAIL_V3) || defined (PIC24FJ256DA210_DEV_BOARD)
        SST25ReadArray(memory->address + offset, // address to read from
        (BYTE *)buffer, nCount);
        #else
        SST39PMPInit();
        SST39ReadArray(memory->address + offset, // address to read from
        (BYTE *)buffer, nCount);
        LCDPMPInit();
        #endif
    }

    return (nCount);
}

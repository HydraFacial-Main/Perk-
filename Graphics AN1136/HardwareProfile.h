/*********************************************************************
 *
 *	Hardware specific definitions
 *
 *********************************************************************
 * FileName:        HardwareProfile.h
 * Dependencies:    None
 * Processor:       PIC24F, PIC24H, dsPIC, PIC32
 * Compiler:        Microchip C32 v1.00 or higher
 *					Microchip C30 v3.01 or higher
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * Copyright � 2002-2008 Microchip Technology Inc.  All rights 
 * reserved.
 *
 * Microchip licenses to you the right to use, modify, copy, and 
 * distribute: 
 * (i)  the Software when embedded on a Microchip microcontroller or 
 *      digital signal controller product (�Device�) which is 
 *      integrated into Licensee�s product; or
 * (ii) ONLY the Software driver source files ENC28J60.c and 
 *      ENC28J60.h ported to a non-Microchip device used in 
 *      conjunction with a Microchip ethernet controller for the 
 *      sole purpose of interfacing with the ethernet controller. 
 *
 * You should refer to the license agreement accompanying this 
 * Software for additional information regarding your rights and 
 * obligations.
 *
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED �AS IS� WITHOUT 
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT 
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A 
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL 
 * MICROCHIP BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR 
 * CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF 
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS 
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE 
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER 
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT 
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE.
 *
 *
 * Author               Date		Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Howard Schlunder		10/03/06	Original, copied from Compiler.h
 * Jayanth Murthy       06/25/09    dsPIC & PIC24H support 
 * Pradeep Budagutta	15 Sep 2009 Added PIC24FJ256DA210 Development Board Support
 ********************************************************************/

#if defined (__PIC24F__) || defined(__dsPIC33F__) || defined(__PIC24H__)

	#if defined (__PIC24FJ256DA210__)
		/*********************************************************************
	     * Hardware Configuration for 
	     * PIC24FJ256DA210 Development Board
	     * Display TFT-G240320LTSW-118W-E
	     ********************************************************************/
//		#include "Alternative Configurations/HardwareProfile_PIC24FJ256DA210_DEV_BOARD_16PMP_MCHP_DA210_TFT_G240320LTSW_118W_E.h"
		
		/*********************************************************************
	     * Hardware Configuration for 
	     * PIC24FJ256DA210 Development Board
	     * Display PH480272T-005-I11Q
	     ********************************************************************/
		#include "C:/Microchip Spa Pro Demo/Graphics AN1136/Alternative Configurations/HardwareProfile_PIC24FJ256DA210_DEV_BOARD_16PMP_MCHP_DA210_PH480272T_005_I11Q.h"

	#else
		/*********************************************************************
	     * Hardware Configuration for 
	     * Explorer 16
    	 * Graphics PicTail v3
	     * Display TFT-G240320LTSW-118W-E
	     ********************************************************************/
		#include "Alternative Configurations/HardwareProfile_GFX_PICTAIL_V3_8PMP_SSD1926_TFT_G240320LTSW_118W_E.h"
		//#include "Alternative Configurations/HardwareProfile_GFX_PICTAIL_V3_16PMP_SSD1926_TFT_G240320LTSW_118W_E.h"

		/*********************************************************************
	     * Hardware Configuration for 
	     * Explorer 16
    	 * Graphics PicTail v3
	     * Display PH480272T-005-I11Q
	     ********************************************************************/
		//#include "Alternative Configurations/HardwareProfile_GFX_PICTAIL_V3_8PMP_SSD1926_PH480272T_005_I11Q.h"
		//#include "Alternative Configurations/HardwareProfile_GFX_PICTAIL_V3_16PMP_SSD1926_PH480272T_005_I11Q.h"

		/*********************************************************************
	     * Hardware Configuration for 
	     * Explorer 16
     	 * Graphics PicTail v2
         * Display LGDP4531
	     ********************************************************************/
		//#include "Alternative Configurations/HardwareProfile_GFX_PICTAIL_V2_8PMP_LGDP4531.h"
		
	#endif

#elif defined (__PIC32MX__) 

    /*********************************************************************
     * Hardware Configuration for 
     * Explorer 16
     * Graphics PicTail v3
     * Display TFT-G240320LTSW-118W-E
     ********************************************************************/
    #include "Alternative Configurations/HardwareProfile_GFX_PICTAIL_V3_8PMP_SSD1926_TFT_G240320LTSW_118W_E.h"
	//#include "Alternative Configurations/HardwareProfile_GFX_PICTAIL_V3_16PMP_SSD1926_TFT_G240320LTSW_118W_E.h"

    /*********************************************************************
     * Hardware Configuration for 
     * Explorer 16
     * Graphics PicTail v3
     * Display PH480272T-005-I11Q
     ********************************************************************/
	//#include "Alternative Configurations/HardwareProfile_GFX_PICTAIL_V3_8PMP_SSD1926_PH480272T_005_I11Q.h"
	//#include "Alternative Configurations/HardwareProfile_GFX_PICTAIL_V3_16PMP_SSD1926_PH480272T_005_I11Q.h"

    /*********************************************************************
     * Hardware Configuration for 
     * Explorer 16
     * Graphics PicTail v2
     * Display LGDP4531
     ********************************************************************/
	//#include "Alternative Configurations/HardwareProfile_GFX_PICTAIL_V2_8PMP_LGDP4531.h"

    /*********************************************************************
     * Hardware Configuration for 
     * Starter Kit
     * Graphics PicTail v3
     * Display TFT-G240320LTSW-118W-E
     ********************************************************************/
	//#include "Alternative Configurations/HardwareProfile_GFX_PICTAIL_V3_8PMP_PIC32_STK_SSD1926_TFT_G240320LTSW_118W_E.h"
	//#include "Alternative Configurations/HardwareProfile_GFX_PICTAIL_V3_16PMP_PIC32_STK_SSD1926_TFT_G240320LTSW_118W_E.h"

	//#include "Alternative Configurations/HardwareProfile_GFX_PICTAIL_V3_8PMP_PIC32_USB_STK_SSD1926_TFT_G240320LTSW_118W_E.h"
	//#include "Alternative Configurations/HardwareProfile_GFX_PICTAIL_V3_16PMP_PIC32_USB_STK_SSD1926_TFT_G240320LTSW_118W_E.h"

    /*********************************************************************
     * Hardware Configuration for 
     * Starter Kit
     * Graphics PicTail v3
     * Display PH480272T-005-I11Q
     ********************************************************************/
	//#include "Alternative Configurations/HardwareProfile_GFX_PICTAIL_V3_8PMP_PIC32_STK_SSD1926_PH480272T_005_I11Q.h"
	//#include "Alternative Configurations/HardwareProfile_GFX_PICTAIL_V3_16PMP_PIC32_STK_SSD1926_PH480272T_005_I11Q.h"
	
	//#include "Alternative Configurations/HardwareProfile_GFX_PICTAIL_V3_8PMP_PIC32_USB_STK_SSD1926_PH480272T_005_I11Q.h"
	//#include "Alternative Configurations/HardwareProfile_GFX_PICTAIL_V3_16PMP_PIC32_USB_STK_SSD1926_PH480272T_005_I11Q.h"
	
    /*********************************************************************
     * Hardware Configuration for 
     * Starter Kit
     * MultiMedia Development Board
     * Display TFT-G240320LTSW-118W-E
     ********************************************************************/
    //#include "Alternative Configurations/HardwareProfile_MULTI_MEDIA_BOARD_DM00123_16PMP_PIC32_STK_SSD1926_TFT_G240320LTSW_118W_E.h"
    //#include "Alternative Configurations/HardwareProfile_MULTI_MEDIA_BOARD_DM00123_16PMP_PIC32_USB_STK_SSD1926_TFT_G240320LTSW_118W_E.h"
    //#include "Alternative Configurations/HardwareProfile_MULTI_MEDIA_BOARD_DM00123_16PMP_PIC32_ENET_STK_SSD1926_TFT_G240320LTSW_118W_E.h"

    //#include "Alternative Configurations/HardwareProfile_MULTI_MEDIA_BOARD_DM00123_8PMP_PIC32_STK_SSD1926_TFT_G240320LTSW_118W_E.h"
    //#include "Alternative Configurations/HardwareProfile_MULTI_MEDIA_BOARD_DM00123_8PMP_PIC32_ENET_STK_SSD1926_TFT_G240320LTSW_118W_E.h"
    //#include "Alternative Configurations/HardwareProfile_MULTI_MEDIA_BOARD_DM00123_8PMP_PIC32_USB_STK_SSD1926_TFT_G240320LTSW_118W_E.h"

#endif

			// Description: SD-SPI Chip Select Output bit
//			#define SD_CS   PORTAbits.RA6	//G8

			// Description: SD-SPI Chip Select TRIS bit
//			#define SD_CS_TRIS  TRISAbits.TRISA6

			// Description: SD-SPI Card Detect Input bit
			#define SD_CD   PORTAbits.RA15

			// Description: SD-SPI Card Detect TRIS bit
			#define SD_CD_TRIS  TRISAbits.TRISA15

			// Description: SD-SPI Write Protect Check Input bit
			#define SD_WE   PORTCbits.RC13   //SD_WE   PORTAbits.RA7

			// Description: SD-SPI Write Protect Check TRIS bit
			#define SD_WE_TRIS  TRISCbits.TRISC13  //SD_WE_TRIS  TRISAbits.TRISA7

			// Registers for the SPI module you want to use
			// Description: The main SPI control register
			#define SPICON1 SPI1CON1

			// Description: The SPI status register
			#define SPISTAT SPI1STAT

			// Description: The SPI Buffer
			#define SPIBUF  SPI1BUF

			// Description: The receive buffer full bit in the SPI status register
			#define SPISTAT_RBF SPI1STATbits.SPIRBF

			// Description: The bitwise define for the SPI control register (i.e. _____bits)
			#define SPICON1bits SPI1CON1bits

			// Description: The bitwise define for the SPI status register (i.e. _____bits)
			#define SPISTATbits SPI1STATbits

			// Description: The enable bit for the SPI module
			#define SPIENABLE   SPISTATbits.SPIEN

    	    #define SPICON2  SPI2CON2

			// Tris pins for SCK/SDI/SDO lines


			// Description: The TRIS bit for the SCK pin
			#define SPICLOCK    TRISFbits.TRISF3	//D8
			// Description: The TRIS bit for the SDI pin
			#define SPIIN       TRISCbits.TRISC14	//B0
			// Description: The TRIS bit for the SDO pin
			#define SPIOUT      TRISGbits.TRISG6	//B1

#define     SCL1_TRIS    TRISAbits.TRISA6	// SmartChip  tris bit, PORTE pin 7
    

#define     SCL1         LATAbits.LATA6   // I2C clock, PORTB pin 4
#define     SDA1         PORTAbits.RA7   // I2C data, PORTB pin 5
#define     SDA1_TRIS    TRISAbits.TRISA7// SDA tris bit, PORTB pin 5

#define		PWM_TRIS	TRISBbits.TRISB5

//port b
    #define		AMBER_ON  0x01
    #define		GREEN_ON  0x02
    #define		RED_ON  0x04
    #define		BACKLIGHT_ON  0x08
	#define   	BUZZER_ON  0x10
	#define   	PUMP_ON  0x20
	#define   	CS_SD_HIGH  0x40
	#define   	RED12_ON  0x80

    #define		AMBER_OFF  0xFE
    #define		GREEN_OFF  0xFD
    #define		RED_OFF  0xFB
    #define		BACKLIGHT_OFF  0xF7
	#define   	BUZZER_OFF  0xEF
	#define   	PUMP_OFF  0xDF
	#define   	CS_SD_LOW  0xBF
	#define   	RED12_OFF  0x7F

//port a
	#define		TEST1_ON 0x80
	#define		TEST2_ON 0x40
	#define		TEST3_ON 0x20
	#define		RFID_SIG 0x04
	#define		FLOAT_SWITCH 0x02
	#define		STANDBY_SWITCH 0x01

	#define		TEST1_OFF 0x7F
	#define		TEST2_OFF 0xBF
	#define		TEST3_OFF	0xDF


#define    HOME_IMAGE	1
#define    CLEANSE_IMAGE	2
#define    SETTINGS_IMAGE	3
#define    REMINDER_IMAGE	4
#define	CREDIT_IMAGE 	5
#define		SMART_FACIAL	6
#define	WARNING_IMAGE	7
#define		DAYS_LEFT	8




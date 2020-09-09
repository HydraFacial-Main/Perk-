/*****************************************************************************
 * Microchip Graphics Library Demo Application
 * This program shows how to use the Graphics Objects Layer.
 *****************************************************************************
 * FileName:        AN1136Demo.c
 * Dependencies:    AN1136Demo.h
 * Processor:       PIC24F, PIC24H, dsPIC, PIC32F
 * Compiler:       	MPLAB C30 V3.00, MPLAB C32
 * Linker:          MPLAB LINK30, MPLAB LINK32
 * Company:         Microchip Technology Incorporated
 *
 * Software License Agreement
 *
 * Copyright © 2008 Microchip Technology Inc.  All rights reserved.
 * Microchip licenses to you the right to use, modify, copy and distribute
 * Software only when embedded on a Microchip microcontroller or digital
 * signal controller, which is integrated into your product or third party
 * product (pursuant to the sublicense terms in the accompanying license
 * agreement).  
 *
 * You should refer to the license agreement accompanying this Software
 * for additional information regarding your rights and obligations.
 *
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS” WITHOUT WARRANTY OF ANY
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
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Paolo A. Tamayo		07/20/07	...
 *****************************************************************************/
#include "AN1136Demo.h"
#include "FSconfig.h"
#include "UART.h"
#include "C:/Microchip Wave II Images Touchscreen Codes/Microchip/Include/MDD File System/FSIO.h"
#include "I2C.h"

            #include "usb_config.h"
            #include "USB/usb.h"
            #include "USB/usb_host_msd.h"
            #include "USB/usb_host_hid_parser.h"
            #include "USB/usb_host_hid.h"

// Configuration bits
#if defined(__dsPIC33F__) || defined(__PIC24H__)
_FOSCSEL(FNOSC_PRI);
_FOSC(FCKSM_CSECMD &OSCIOFNC_OFF &POSCMD_XT);
_FWDT(FWDTEN_OFF);
#elif defined(__PIC32MX__)
    #pragma config FPLLODIV = DIV_1, FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FWDTEN = OFF, FCKSM = CSECME, FPBDIV = DIV_1
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


/////////////////////////////////////////////////////////////////////////////
//                              OBJECT'S IDs
/////////////////////////////////////////////////////////////////////////////

#define ID_WHOLE 		20
#define ID_SMART_FACIAL 		21
#define ID_CLEANSE_MODE		22
#define ID_SETTINGS_MODE 		23

#define ID_HOME 		24
#define ID_ON 		25
#define ID_OFF 		26
#define ID_SETTING1 		27
#define ID_SETTING2 		28
#define ID_SETTING3 		29
#define ID_SETTING4 		30
#define ID_SETTING5 		31

#define ID_YES 		32
#define ID_NO 		33

#define ID_BUTTON1 		34
#define ID_BUTTON2 		35
#define ID_BUTTON3 		36

#define ID_0 		41
#define ID_1 		42
#define ID_2 		43
#define ID_3 		44
#define ID_4 		45
#define ID_5 		46
#define ID_6 		47
#define ID_7 		48
#define ID_8 		49
#define ID_9 		50
#define ID_BS 		51
#define ID_ENTER 		52

#define ID_BR_INVISIBLE 		55
#define ID_BL_INVISIBLE 		56

//Display: (24,76), (248,119)

#define BOX_LEFT	24
#define BOX_TOP		76
#define BOX_RIGHT	248
#define BOX_BOTTOM	119
#define BOX_BACKGROUND	WHITE
#define TEXT_COLOR	GRAY4
#define TEXT_LEFT	28
#define TEXT_TOP	72

#define CIRCLE_X	50
#define CIRCLE_Y	50
#define CIRCLE_SIZE	10

#define TRANSMIT_MSG_SIZE 20

/////////////////////////////////////////////////////////////////////////////
//                            LOCAL PROTOTYPES
/////////////////////////////////////////////////////////////////////////////
#define WAIT_UNTIL_FINISH(x)    while(!x)

/////////////////////////////////////////////////////////////////////////////
//                            IMAGES USED
/////////////////////////////////////////////////////////////////////////////
extern const FONT_FLASH BigFonts;
extern const FONT_FLASH GOLMediumFont;
extern const FONT_FLASH GOLSmallFont;
/*
extern IMAGE_EXTERNAL Clean;
extern IMAGE_EXTERNAL L100;
extern IMAGE_EXTERNAL L75;
extern IMAGE_EXTERNAL L50;
extern IMAGE_EXTERNAL L25;
*/
extern FONT_EXTERNAL HelveticaNeueLTStd_Roman_14;

extern IMAGE_EXTERNAL Warning; 
extern IMAGE_EXTERNAL Settings;
extern IMAGE_EXTERNAL Reminder;
extern IMAGE_EXTERNAL on;
extern IMAGE_EXTERNAL off;
extern IMAGE_EXTERNAL Main;
extern IMAGE_EXTERNAL Intro; 
extern IMAGE_EXTERNAL Clean; 
extern IMAGE_EXTERNAL L100;
extern IMAGE_EXTERNAL L75;
extern IMAGE_EXTERNAL L50;
extern IMAGE_EXTERNAL L25;
extern IMAGE_EXTERNAL L5;
extern IMAGE_EXTERNAL L4;
extern IMAGE_EXTERNAL L3;
extern IMAGE_EXTERNAL L2;
extern IMAGE_EXTERNAL L1;
extern IMAGE_EXTERNAL L00;
extern IMAGE_EXTERNAL black;
extern IMAGE_EXTERNAL Credit;

extern BYTE DaysLeft;
extern BYTE continuousFlag;
extern BYTE pinOKFlag;
extern DWORD lastJulian;
extern BYTE SDcard[100][14];
extern BYTE codeCount;


/////////////////////////////////////////////////////////////////////////////
//                                  MAIN
/////////////////////////////////////////////////////////////////////////////
GOL_SCHEME                  *altScheme; // alternative style scheme
WORD                        update = 0; // variable to update customized graphics
BYTE imageState = 0;
BYTE portB = 0x40;	//CS_SD high;
BYTE portBsave;
BYTE portBsave1;
BYTE portA = 0x00;
BYTE buttonReleased = 0;
BYTE vacuumLevel;
BYTE startFlag;
SHORT LEDseconds;
BYTE tempSeconds = 0;
BYTE ledBlinkTimer = 0;
BYTE buzzerTime;
BYTE blinkFlag;
BYTE changeBlink = FALSE;
BYTE buttonTimer = 0;
volatile BYTE buttonDelay = 0;
SHORT oldSeconds = 0;
BYTE hiddenButtonTimer = 0;
BYTE hiddenButtonFlagT = 0;
BYTE hiddenButtonFlagB = 0;
BYTE position = 0;
BYTE daysTimer;
char receiveBuffer[20];
BYTE seconds;
BYTE minutes;
WORD newMinutes;
BYTE hours;
BYTE date;
BYTE month;
BYTE year;
BYTE date1;
char String[20];
BYTE 	serialNumberFlag = 0;
WORD RTCtimer = 36;
FSFILE * pointer;
FSFILE * pointerSave;
char SDserialNumber[14];
char SDpinNumber[14];
BYTE pinFlag;
volatile WORD timer2 = 0;
volatile DWORD julianDate;
WORD daysLeftTimer = 50;	//5 sec
BYTE daysLeftOld = 0;
BYTE RTCflag = FALSE;
BYTE changeVacuumFlag = FALSE;
BYTE pumpOnFlag = FALSE;
BYTE pumpValue;

BYTE floatSwitchFlag = FALSE;
BYTE floatSwitchOnDetected = FALSE;
BYTE floatSwitchOffDetected = FALSE;
BYTE cleanseTimer;
BYTE percentValue;
IMAGE_EXTERNAL *ptrImage1;
BYTE standbyDetectedFlag = FALSE;
BYTE standbyMode = FALSE;
BYTE warningImage = FALSE;
BYTE warningOffImage = FALSE;
BYTE standbyFlag = FALSE;
BYTE normalDetectedFlag = FALSE;
BYTE standbySetFlag = FALSE;

BYTE XmtBuffer[TRANSMIT_MSG_SIZE];
BYTE ptrXmt = 0;
BYTE xmtFlag = FALSE;
BYTE                    usbMSDStatus;                       // MSD device status
BYTE                    usbErrorCode;                       // USB error
BYTE                    usbHIDStatus;                       // HID device status
BYTE                    usbMSDStatus;                       // MSD device status

BYTE  mediaPresent = FALSE;
	SHORT FScounter;


const XCHAR             ErrMsgStandard[] = {'U','S','B',' ','E','r','r','o','r',0};
const XCHAR             ErrNotSupported[] = {'n','o','t',' ','s','u','p','p','o','r','t','e','d','!',0};
const XCHAR             ErrMsgFailedStr[] = {'F','a','i','l','e','d',0};
const XCHAR             ErrMsgHUBAttachedStr[] = {'H','U','B',0};
const XCHAR             ErrMsgUDAttachedStr[] = {'D','e','v','i','c','e',0};
const XCHAR             ErrMsgEnumerationStr[] = {'E','n','u','m','e','r','a','t','i','o','n',0};
const XCHAR             ErrMsgClientInitStr[] = {'C','l','i','e','n','t',' ','I','n','i','t','i','a','l','i','z','a','t','i','o','n',0};
const XCHAR             ErrMsgOutofMemoryStr[] = {'O','u','t',' ','o','f',' ','M','e','m','o','r','y',0};
const XCHAR             ErrMsgUnpecifiedErrStr[] = {'U','n','s','p','e','c','i','f','i','e','d',0};
const XCHAR             MsgTouchToProceedStr[] = {'T','o','u','c','h',' ','t','o',' ','p','r','o','c','e','e','d',0};

const XCHAR				strSerialNumber[] = {'E','n','t','e','r',' ','s','e','r','i','a','l',' ','n','u','m','b','e','r',0};
const XCHAR				strCode[] = {'E','n','t','e','r',' ','c','o','d','e',0};

#define LED_BLINK_TIME		5	//0.5 SEC
#define BUZZER_TIME		5	//0.5 SEC
#define LEVEL1	2250
#define LEVEL2	2235	
#define LEVEL3	2220	
#define LEVEL4	2210	
#define LEVEL5	2180
#define PWM_DELAY  10
/*	
2270
2260
2240
2225
2190
*/







void            TickInit(void);                 // starts tick counter
unsigned long CheckButtons(GOL_MSG *pMsg);
void HydraFacial(void);
void SystemClean(void);
void ClearButtons(void);
void uitoa(WORD Value, BYTE* Buffer);
void ChangeTime(int seconds);
void HomeSetup(void);
void CreditCode(void);
BYTE Debounce(void);
void uitoa(WORD Value, BYTE* Buffer);
void ReminderImage(void);
void SettingsImage(void);
void CreateButtons(void);
void MonitorDriveMedia(void);
BOOL USB_ApplicationEventHandler(BYTE address, USB_EVENT event, void *data, DWORD size);

/* */
int main(void)
{
	char tempPIN[10];
	char tempCode[20];
	char tempString[20];
	volatile WORD temp1;
	volatile DWORD temp2;
	BYTE difference;
	BYTE intI, intJ;
	volatile BYTE address, data;

    GOL_MSG msg;                    // GOL message structure to interact with GOL
    
     #if defined(PIC24FJ256DA210_DEV_BOARD)
    
    _ANSG8 = 0; /* S1 */
    _ANSE9 = 0; /* S2 */
    _ANSB5 = 0; /* S3 */
        
		// Configure SPI1 PPS pins (ENC28J60/ENCX24J600/MRF24WB0M or other PICtail Plus cards)
		__builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS

		RPOR8bits.RP16R = 8;       // assign RP16 for SCK1
		RPOR10bits.RP21R = 7;       // assign RP21 for SDO1
		RPINR20bits.SDI1R = 11;    // assign RP11 for SDI1

		// Assign U1RX To Pin RPI34
		RPINR18bits.U1RXR = 34;
		// Assign U1TX To Pin RP19
		RPOR9bits.RP19R = 3;

		// Assign PWM (OC 1) To Pin RP18
		RPOR9bits.RP18R = 18;

		__builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS                

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
    
    // Set PMD0 pin functionality to digital
    AD1PCFGL = AD1PCFGL | 0x1000;
    
    #elif defined(__PIC32MX__)
    INTEnableSystemMultiVectoredInt();
    SYSTEMConfigPerformance(GetSystemClock());
    #ifdef MULTI_MEDIA_BOARD_DM00123
    CPLDInitialize();
    CPLDSetGraphicsConfiguration(GRAPHICS_HW_CONFIG);
    CPLDSetSPIFlashConfiguration(SPI_FLASH_CHANNEL);
    #endif // #ifdef MULTI_MEDIA_BOARD_DM00123
    #endif // #if defined(__dsPIC33F__) || defined(__PIC24H__)

    GOLInit();                      // initialize graphics library &

    #if defined (GFX_PICTAIL_V1) || defined (GFX_PICTAIL_V2)
    EEPROMInit();                   // initialize Exp.16 EEPROM SPI
    BeepInit();
    #else
	    #if defined (USE_SST25VF016)
	    SST25Init();                    // initialize GFX3 SST25 flash SPI
	    #endif
    #endif
    
    G1CON3bits.DPENOE = 0;		//disable GEN
    PMCON3bits.PTEN17 = 0;		// disable PMA17

    SCL1_TRIS = 0;
	PWM_TRIS = 0;


//    SD_CS = 1;                     //Initialize Chip Select line

//    UARTInit();

    SPICON1 = 0x0000;              // power on state
    SPICON1 |= 0x20; 	//sync_mode;          // select serial mode 
    SPICON1bits.CKP = 1;
    SPICON1bits.CKE = 0;

	SPICLOCK = 0;
	SPIOUT = 0;                  // define SDO1 as output (master or slave)
	SPIIN = 1;                  // define SDI1 as input (master or slave)
	SPIENABLE = 1;             // enable synchronous serial port


	MPCInit();
	portB |= GREEN_ON;
	portB &= PUMP_OFF;
	portB &= RED_OFF;
    portB |= CS_SD_HIGH;	//Initialize Chip Select line
	portB |= BACKLIGHT_ON;

	WriteMPCB(portB);
	WriteMPCA(portA);

    TouchInit();                    // initialize touch screen
    TickInit();                     // initialize tick counter (for random number generation)

    U1TXREG = 0x42;

//    HardwareButtonInit();           // Initialize the hardware buttons

//        TouchCalibration();
//        TouchStoreCalibration();

	
    // If it's a new board (EEPROM_VERSION byte is not programed) calibrate touch screen
    #if defined (GFX_PICTAIL_V1) || defined (GFX_PICTAIL_V2)
    if(GRAPHICS_LIBRARY_VERSION != EEPROMReadWord(ADDRESS_VERSION))
    {
        TouchCalibration();
        TouchStoreCalibration();
    }

    #else
	    #if defined (USE_SST25VF016)
	    if(GRAPHICS_LIBRARY_VERSION != SST25ReadWord(ADDRESS_VERSION))
	    {
	        TouchCalibration();
	        TouchStoreCalibration();
	    }
	    #elif defined (USE_SST39LF400)
		WORD tempArray[12], tempWord = 0x1234;
		
		SST39LF400Init(tempArray);
		tempWord = SST39LF400ReadWord(ADDRESS_VERSION);
		SST39LF400DeInit(tempArray);

	    if(GRAPHICS_LIBRARY_VERSION != tempWord)
	    {
	        TouchCalibration();
	        TouchStoreCalibration();
	    }
	    #endif
    #endif

    // Load touch screen calibration parameters from memory
    TouchLoadCalibration();
//	SCL1_TRIS = 0;
//	DaysLeft = 30;

	//make sure RTC on
	seconds = ReadRTC(0x00);
	if((seconds & 0x80) == 0x80)
	{
		WriteRTC(0x00, 0x00);	//no - turn on

		date = ReadRTC(0x04) & 0x0F;
		date += (ReadRTC(0x04) >> 4) * 10;
		julianDate = (DWORD)date;

		month = ReadRTC(0x05) & 0x0F;
		month += (ReadRTC(0x05) >> 4) * 10;

		year = ReadRTC(0x06) & 0x0F;
		year = (ReadRTC(0x06) >> 4) * 12;

		switch(month)
		{
			case 1:
				break;

			case 2:
				julianDate += 31;
				break;

			case 3:
				julianDate += 59;
				break;

			case 4:
				julianDate += 90;
				break;

			case 5:
				julianDate += 120;
				break;

			case 6:
				julianDate += 151;
				break;

			case 7:
				julianDate += 181;
				break;

			case 8:
				julianDate += 212;
				break;

			case 9:
				julianDate += 243;
				break;

			case 10:
				julianDate += 273;
				break;

			case 11:
				julianDate += 304;
				break;

			case 12:
				julianDate += 334;
				break;

			default:
				break;

		}

		julianDate += (DWORD)year * 365;
		lastJulian = julianDate;
		TouchStoreCalibration();	//store in flash
		
	}

	// Configure SPI1 PPS pins (ENC28J60/ENCX24J600/MRF24WB0M or other PICtail Plus cards)
	__builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS

	RPOR8bits.RP16R = 8;       // assign RP16 for SCK1
	RPOR10bits.RP21R = 7;       // assign RP21 for SDO1
	RPINR20bits.SDI1R = 11;    // assign RP11 for SDI1
//		RPINR20bits.SDI1R = 37;    // assign RPI37 for SDI1

	__builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS                

	FScounter = 3;

	while(FScounter != 0)
	{
		if(!FSInit())	//try three times to read SD card
		{
			FScounter--;	
			
		}
		else
		{
			break;
		}
	}

	if(FScounter != 0)	//SD card found
	{
		// Open file 1 in read mode
		pointer = FSfopen ("Codes.txt", "r");
		if (pointer == NULL)
		  	while(1);

		intJ = 0;

	  	while(1)
	  	{
	  		// Check if this is the end of the file
	  		if (FSfeof (pointer))
	  		  break;

	  		// Read one 14-byte object
	  		if (FSfread (tempCode, 14, 1, pointer) != 1)
	  		  while(1);


	  		for(intI = 0; intI < 14; intI++)
	  		{
	  			SDcard[intJ][intI] = tempCode[intI];
	  			
	  		}

			intJ++;

		}

		codeCount = intJ;

		FSfclose (pointer);

		//now write to flash
		StoreSD();
		TouchStoreCalibration();
			
	}



 //   altScheme = GOLCreateScheme();  // create alternative style scheme
 //   altScheme->TextColor0 = BLACK;
//    altScheme->TextColor1 = BRIGHTBLUE;
    altScheme = GOLCreateScheme();  // create alternative style scheme
    altScheme->TextColor0 = GRAY0;
    altScheme->TextColor1 = GRAY0;
    altScheme->CommonBkColor = GRAY0;
//    altScheme->pFont = GRAY0;
    altScheme->EmbossDkColor = GRAY0;
    altScheme->EmbossLtColor = GRAY0;
    altScheme->Color0 = GRAY0;
    altScheme->Color1 = GRAY0;

	CreateButtons();


//    TouchCalibration();


    SetColor(BLACK);
    ClearDevice();



    if(!continuousFlag)
    {
    	if(DaysLeft == 0)
    	{
    		CreditCode();
    	
    	}
    	else
    	{
    		PutImage(0, 0, (void *) &Intro, 1);
    		ptrImage1 = &Intro;
    	}

    }
    else
    {
    	PutImage(0, 0, (void *) &Intro, 1);
    	ptrImage1 = &Intro;
			
    }

    update = 1;                     // to initialize the user graphics

    while(1)
    {
        if(GOLDraw())
        {                           // Draw GOL object
            TouchGetMsg(&msg);      // Get message from touch screen
            GOLMsg(&msg);           // Process message
        }

		if(standbyFlag == FALSE)
		{
			if(changeBlink)	//red LED flashing?
			{
				changeBlink = FALSE;
	
				if(blinkFlag)
				{
					//red led on
					portB |= RED_ON;
				
					WriteMPCB(portB);
	
				}
				else
				{
					//red LED off
					portB &= RED_OFF;
				
					WriteMPCB(portB);
				}
			}
	
			data = ReadMPC(0x12) & FLOAT_SWITCH;
			if(!data)
			{
				buttonDelay = 30;
	
				while(buttonDelay != 0)	//debounce
				{
				}
	
				data = ReadMPC(0x12) & FLOAT_SWITCH;
				if(!data)	//still low?
				{
					if(floatSwitchOnDetected == FALSE)
					{
						floatSwitchFlag = TRUE;
						floatSwitchOnDetected = TRUE;
						floatSwitchOffDetected = FALSE;
					}
				}
				
			}
			else
			{
				if(floatSwitchFlag)
				{
					if(floatSwitchOffDetected == FALSE)
					{
						floatSwitchFlag = FALSE;
						floatSwitchOffDetected = TRUE;
						floatSwitchOnDetected = FALSE;
					
						portB &= RED_OFF;
				
						WriteMPCB(portB);
	
	
					}
				}
			}
			if(standbyMode == FALSE)
			{
				if(floatSwitchOnDetected)
				{
					if(warningImage == FALSE)
					{
						warningImage = TRUE;
						warningOffImage = FALSE;
						portB &= PUMP_OFF;
						WriteMPCB(portB);
	
						PutImage(0, 0, (void *) &Warning, 1);
						imageState = WARNING_IMAGE;
					}
				}
				if(floatSwitchOffDetected)
				{
					if(warningOffImage == FALSE)
					{
						warningOffImage = TRUE;
						warningImage = FALSE;
						pumpOnFlag = FALSE;
						portB &= PUMP_OFF;
						WriteMPCB(portB);
	
			    		PutImage(0, 0, (void *) &Intro, 1);
						imageState = 0;
					}
						
				}
				data = ReadMPC(0x12) & STANDBY_SWITCH;
		
				if(!data)
				{
					buttonDelay = 30;
		
					while(buttonDelay != 0)	//debounce
					{
					}
		
					data = ReadMPC(0x12) & STANDBY_SWITCH;
		
					if(!data)
					{
		
						if(standbyDetectedFlag == FALSE)
						{
							pumpOnFlag = FALSE;
							portB &= PUMP_OFF;
							WriteMPCB(portB);
			
							standbyDetectedFlag = TRUE;
							normalDetectedFlag = FALSE;
							standbyMode = TRUE;
							ReminderImage();
							//wait till off
							while(1)
							{
								data = ReadMPC(0x12) & STANDBY_SWITCH;
								if(data)
								{
									buttonDelay = 30;
			
									while(buttonDelay != 0)	//debounce
									{
									
									}
									data = ReadMPC(0x12) & STANDBY_SWITCH;
									if(data)
									{
										break;
									}
								}	
							}	
				
						}
					}
				}	

			}
		
		}
		data = ReadMPC(0x12) & STANDBY_SWITCH;
		if(!data)
		{
			buttonDelay = 30;

			while(buttonDelay != 0)	//debounce
			{
			}

			data = ReadMPC(0x12) & STANDBY_SWITCH;

			data = ReadMPC(0x12) & STANDBY_SWITCH;

			if(!data)
			{

			if(normalDetectedFlag == FALSE)
				{
					normalDetectedFlag = TRUE;
					standbyDetectedFlag = FALSE;
					changeBlink = FALSE;
					floatSwitchFlag = FALSE;
					floatSwitchOnDetected = FALSE;
					floatSwitchOffDetected = FALSE;
					warningImage = FALSE;
					standbyMode = FALSE;

					data = ReadMPC(0x12) & FLOAT_SWITCH;
					if(!data)
					{
						PutImage(0, 0, (void *) &Warning, 1);
						imageState = WARNING_IMAGE;
						
					}
					else
					{
						PutImage(0, 0, (void *) &Intro, 1);
						imageState = 0;
						
					}

					portB &= AMBER_OFF;
					portB |= GREEN_ON;
					portB |= BACKLIGHT_ON;
					WriteMPCB(portB);
					standbyFlag = FALSE;
					//wait till off
					while(1)
					{
						data = ReadMPC(0x12) & STANDBY_SWITCH;
						if(data)
						{
							buttonDelay = 30;

							while(buttonDelay != 0)	//debounce
							{
							
							}
							data = ReadMPC(0x12) & STANDBY_SWITCH;
							if(data)
							{
								break;
							}
						}	
					}	
	
				}
			}
		}

    }
}

/////////////////////////////////////////////////////////////////////////////
// Function: WORD GOLMsgCallback(WORD objMsg, OBJ_HEADER* pObj, GOL_MSG* pMsg)
// Input: objMsg - translated message for the object,
//        pObj - pointer to the object,
//        pMsg - pointer to the non-translated, raw GOL message
// Output: if the function returns non-zero the message will be processed by default
// Overview: it's a user defined function. GOLMsg() function calls it each

//           time the valid message for the object received
/////////////////////////////////////////////////////////////////////////////
WORD GOLMsgCallback(WORD objMsg, OBJ_HEADER *pObj, GOL_MSG *pMsg)
{
    WORD    objectID;
    SHORT   width, height;
	BYTE	foundFlag;
	DWORD multiplier;
	DWORD checksum;
	DWORD expectedChecksum;
	char tempString[20];
	char tempPIN[10];
	char tempCode[20];
	volatile WORD intI, intJ, intK;

    objectID = GetObjID(pObj);

    if(imageState == 0)
    {
    	if(buttonReleased == 0)
    	{
            if(objMsg == BTN_MSG_PRESSED)
            {   // check if button is pressed
    			buttonReleased = 1;

    			HomeSetup();
    			return(1);
    	        update = 1;
    			
            }
    			
    	}
    	if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
    	{
    		buttonReleased = 0;
    		
    	}


    }

    switch (imageState)
    {
    	case HOME_IMAGE:
    	    if(objectID == ID_SMART_FACIAL)
    	    {
    			if(buttonReleased == 0)
    			{
    		        if(objMsg == BTN_MSG_PRESSED)
    		        {   // check if button is pressed
        				buttonReleased = 1;

    					HydraFacial();
    			        update = 1;
    					
    		        }
    					
    			}
        		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
        		{
        			buttonReleased = 0;
        			
        		}

    	    }

    	    if(objectID == ID_CLEANSE_MODE)
    	    {
    	    	if(buttonReleased == 0)
    	    	{
    	            if(objMsg == BTN_MSG_PRESSED)
    	            {   // check if button is pressed
    	    			buttonReleased = 1;

    	    			SystemClean();
    	    	        update = 1;
    	    			
    	            }
    	    			
    	    	}
    	    	if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
    	    	{
    	    		buttonReleased = 0;
    	    		
    	    	}

    	    }

    	    if(objectID == ID_SETTINGS_MODE)
    	    {
    	    	if(buttonReleased == 0)
    	    	{
    	            if(objMsg == BTN_MSG_PRESSED)
    	            {   // check if button is pressed
    	    			buttonReleased = 1;

    	    			SettingsImage();
    	    	        update = 1;
    	    			
    	            }
    	    			
    	    	}
    	    	if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
    	    	{
    	    		buttonReleased = 0;
    	    		
    	    	}

    	    }


    	    if(objectID == ID_BL_INVISIBLE)
    	    {
    		  //	if(buttonReleased == 0)
    			{
    		        if(objMsg == BTN_MSG_PRESSED)
    		        {
        				buttonReleased = 1;
    					hiddenButtonFlagB = 1;
    					hiddenButtonTimer = 30;	//3 sec
				        SetColor(BRIGHTRED);	//red circle on
						FillCircle(CIRCLE_X, CIRCLE_Y, CIRCLE_SIZE);
    		        //update = 1;
    		        }
    					
    			}
        		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
        		{
        			buttonReleased = 0;
        			
        		}

    	    }

    	    if(objectID == ID_BR_INVISIBLE)
    	    {
    	 //		if(buttonReleased == 0)
    			{
    		        if(objMsg == BTN_MSG_PRESSED)
    		        {
        				buttonReleased = 1;
    					if(hiddenButtonFlagB == 1)
    					{
    						hiddenButtonFlagB = 0;
  					        SetColor(GRAY0);		//red circle off
							FillCircle(CIRCLE_X, CIRCLE_Y, CIRCLE_SIZE);
    				        
    				        CreditCode();
    						
    					}
    		        //update = 1;
    		        }
    		        else
    		        {
    					if(!continuousFlag)
    					{
    						uitoa(DaysLeft, (BYTE*)String);
    						SetColor(BLACK);
    						SetFont((void *) &BigFonts);
    	
    						while(!OutTextXY(400, 210, (XCHAR *)String));	//coordinates here???
    						daysLeftTimer = 50;
    							
    					}

    		        		
    		        }

    					
    			}
        		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
        		{
        			buttonReleased = 0;
        			
        		}

    	    }

    		break;

    	case SMART_FACIAL:
    	    if(objectID == ID_SETTING1)
    	    {
    			if(buttonReleased == 0)
    			{
    		        if(objMsg == BTN_MSG_PRESSED)
    		        {   // check if button is pressed
	    		        buttonReleased = 1;

    		    	    PutImage(0, 0, (void *) &L1, 1);
    		    	    if(pumpOnFlag)
    		    	    {
	    		    	    PutImage(163, 18, (void *) &on, 1);
							//set PWM to level 1
							OC1R = 2500;                    // PWM off

							timer2 = PWM_DELAY;		//3 sec delay

							while(timer2 != 0)
							{
								
							}

						    OC1R = LEVEL1;                    // PWM leve1 1


    		    	    }
						else
						{
							PutImage(163, 18, (void *) &off, 1);
							
						}


    		    	    update = 1;
    		        }
    					
    			}
        		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
        		{
        			buttonReleased = 0;
        			
        		}

    	    }

    	    if(objectID == ID_SETTING2)
    	    {
    			if(buttonReleased == 0)
    			{
    		        if(objMsg == BTN_MSG_PRESSED)
    		        {
   						buttonReleased = 1;
   						PutImage(0, 0, (void *) &L2, 1);

   						if(pumpOnFlag)
   						{
   						    PutImage(163, 18, (void *) &on, 1);
							//set PWM to level 2
							OC1R = 2500;                    // PWM off

							timer2 = PWM_DELAY;		//3 sec delay

							while(timer2 != 0)
							{
								
							}

							OC1R = LEVEL2;                    // PWM leve1 2
   						    	
   						}
   						else
   						{
   							PutImage(163, 18, (void *) &off, 1);
   							
   						}

    			        update = 1;

    		        }
    					
    			}
        		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
        		{
        			buttonReleased = 0;
        			
        		}


    	    }

    	    if(objectID == ID_SETTING3)
    	    {
    	    	if(buttonReleased == 0)
    	    	{
    	            if(objMsg == BTN_MSG_PRESSED)
    	            {
    	    			buttonReleased = 1;
    	    			PutImage(0, 0, (void *) &L3, 1);

    	    			if(pumpOnFlag)
    	    			{
    	    			    PutImage(163, 18, (void *) &on, 1);
							//set PWM to level 3
							OC1R = 2500;                    // PWM off

							timer2 = PWM_DELAY;		//3 sec delay

							while(timer2 != 0)
							{
								
							}

							OC1R = LEVEL3;                    // PWM leve1 3

    	    			}
    	    			else
    	    			{
    	    				PutImage(163, 18, (void *) &off, 1);
    	    				
    	    			}



    	    	        update = 1;

    	            }
    	    			
    	    	}
    	    	if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
    	    	{
    	    		buttonReleased = 0;
    	    		
    	    	}


    	    }

    	    if(objectID == ID_SETTING4)
    	    {
    	    	if(buttonReleased == 0)
    	    	{
    	            if(objMsg == BTN_MSG_PRESSED)
    	            {
    	    			buttonReleased = 1;
    	    			PutImage(0, 0, (void *) &L4, 1);

    	    			if(pumpOnFlag)
    	    			{
    	    			    PutImage(163, 18, (void *) &on, 1);
							//set PWM to level 4
							OC1R = 2500;                    // PWM off

							timer2 = PWM_DELAY;		//3 sec delay

							while(timer2 != 0)
							{
								
							}

							OC1R = LEVEL4;                    // PWM leve1 4
    	    			    	
    	    			}
    	    			else
    	    			{
    	    				PutImage(163, 18, (void *) &off, 1);
    	    				
    	    			}

    	    	        update = 1;

    	            }
    	    			
    	    	}
    	    	if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
    	    	{
    	    		buttonReleased = 0;
    	    		
    	    	}


    	    }

    	    if(objectID == ID_SETTING5)
    	    {
    	    	if(buttonReleased == 0)
    	    	{
    	            if(objMsg == BTN_MSG_PRESSED)
    	            {
    	    			buttonReleased = 1;
    	    			PutImage(0, 0, (void *) &L5, 1);

    	    			if(pumpOnFlag)
    	    			{
    	    			    PutImage(163, 18, (void *) &on, 1);
							//set PWM to level 5
							OC1R = 2500;                    // PWM off

							timer2 = PWM_DELAY;		//3 sec delay

							while(timer2 != 0)
							{
								
							}

							OC1R = LEVEL5;                    // PWM leve1 5
    	    			    	
    	    			}
    	    			else
    	    			{
    	    				PutImage(163, 18, (void *) &off, 1);
    	    				
    	    			}

    	    	        update = 1;

    	            }
    	    			
    	    	}
    	    	if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
    	    	{
    	    		buttonReleased = 0;
    	    		
    	    	}


    	    }

    	    if(objectID == ID_ON)
    	    {
    			if(buttonReleased == 0)
    			{
    		        if(objMsg == BTN_MSG_PRESSED)
    		        {
    	
        				buttonReleased = 1;
						if(!floatSwitchFlag)		//cannister not full?
    					{
							if(pumpOnFlag == FALSE)
							{
								if(!floatSwitchFlag)		//cannister not full?
								{
									pumpOnFlag = TRUE;
		
									PutImage(163, 18, (void *) &on, 1);
		
									portB |= PUMP_ON;
									WriteMPCB(portB);
								}	
								
							}
							else
							{
								PutImage(163, 18, (void *) &off, 1);
								pumpOnFlag = FALSE;
	
								portB &= PUMP_OFF;
								WriteMPCB(portB);
	
								
							}
						}
    			        update = 1;
    		        }
    					
    			}
        		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
        		{
					buttonDelay = 30;

					while(buttonDelay != 0)	//debounce
					{
					}

	        		if (objMsg == BTN_MSG_RELEASED)	//still released?
    	    		{
	        			buttonReleased = 0;
    				} 			
        		}

    	    }


    	    if(objectID == ID_HOME)
    	    {
    		 //	if(buttonReleased == 0)
    			{
    		        if(objMsg == BTN_MSG_PRESSED)
    		        {
        				buttonReleased = 1;
    					startFlag = 0;
		    			pumpOnFlag = FALSE;
    
		    			portB &= PUMP_OFF;
		    			WriteMPCB(portB);

    					HomeSetup();
    			        update = 1;
    		        }
    					
    			}
        		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
        		{
        			buttonReleased = 0;
        			
        		}

    	    }


    		break;

    	case SETTINGS_IMAGE:
    	    if(objectID == ID_BUTTON1)
    	    {
    			if(buttonReleased == 0)
    			{
    		        if(objMsg == BTN_MSG_PRESSED)
    		        {   // check if button is pressed
        				buttonReleased = 1;
    					buttonTimer = 10;


    		    	    update = 1;
    		        }
    					
    			}
        		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
        		{
        			buttonReleased = 0;
        			
        		}

			}
    	    if(objectID == ID_BUTTON2)
    	    {
    			if(buttonReleased == 0)
    			{
    		        if(objMsg == BTN_MSG_PRESSED)
    		        {
        				buttonReleased = 1;
    					buttonTimer = 10;


    			        update = 1;

    		        }
    					
    			}
        		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
        		{
        			buttonReleased = 0;
        			
        		}


    	    }

    	    if(objectID == ID_BUTTON3)
    	    {
    			if(buttonReleased == 0)
    			{
    		        if(objMsg == BTN_MSG_PRESSED)
    		        {
    	
        				buttonReleased = 1;
    				

    			        update = 1;
    		        }
    					
    			}
        		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
        		{
        			buttonReleased = 0;
        			
        		}

    	    }


    	    if(objectID == ID_HOME)
    	    {
//				if(buttonReleased == 0)
    			{
    		        if(objMsg == BTN_MSG_PRESSED)
    		        {
        				buttonReleased = 1;
		    			pumpOnFlag = FALSE;
    					startFlag = 0;
    
    					portB &= PUMP_OFF;
    					WriteMPCB(portB);

    					HomeSetup();
    			        update = 1;
    		        }
    					
    			}
        		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
        		{
        			buttonReleased = 0;
        			
        		}

    	    }

    
    		break;

		case CLEANSE_IMAGE:
		    if(objectID == ID_ON)
		    {
		    	if(buttonReleased == 0)
		    	{
		            if(objMsg == BTN_MSG_PRESSED)
		            {
		
		    			buttonReleased = 1;
		    			
		    			if(pumpOnFlag == FALSE)
		    			{
							if(!floatSwitchFlag)		//cannister not full?
							{
			    				pumpOnFlag = TRUE;
	
			    				PutImage(163, 18, (void *) &on, 1);
			    				PutImage(100,45, (void *) &L00, 1);
								cleanseTimer = 25;
								percentValue = 0;
	
								portB |= PUMP_ON;
								WriteMPCB(portB);
							}
		    				
		    			}
		    			else
		    			{
		    				PutImage(163, 18, (void *) &off, 1);
		    				PutImage(100,45, (void *) &L00, 1);
		    				pumpOnFlag = FALSE;
		    				pumpValue = 0;	//pump off
		    				percentValue = 0;
		    				portB &= PUMP_OFF;
		    				WriteMPCB(portB);

		    				
		    			}

		    	        update = 1;
		            }
		    			
		    	}
		    	if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
		    	{
		    		buttonReleased = 0;
		    		
		    	}

		    }

		    if(objectID == ID_HOME)
		    {
//				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
						buttonReleased = 1;
						startFlag = 0;
	
						standbyDetectedFlag = FALSE;
						pumpOnFlag = FALSE;
						portB &= PUMP_OFF;
						WriteMPCB(portB);

						cleanseTimer = 25;
						percentValue = 0;

						pumpOnFlag = FALSE;

						HomeSetup();
				        update = 1;
			        }
						
				}
				if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
				{
					buttonReleased = 0;
					
				}

		    }

	
			break;



    	case REMINDER_IMAGE:
    	    if(objectID == ID_YES)
    	    {
//				if(buttonReleased == 0)
    			{
    		        if(objMsg == BTN_MSG_PRESSED)
    		        {
    	
        				buttonReleased = 1;
        				//put breadboard in standby
						changeBlink = FALSE;
						floatSwitchFlag = FALSE;
						floatSwitchOnDetected = FALSE;
						floatSwitchOffDetected = FALSE;

						standbyFlag = TRUE;
        				portB &= RED_OFF;
        				portB &= GREEN_OFF;
        				portB |= AMBER_ON;
        				WriteMPCB(portB);
        				    				
						//turn LCD off
						portB &= BACKLIGHT_OFF;
						WriteMPCB(portB);

														  
    			        update = 1;
    		        }
    					
    			}
        		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
        		{
        			buttonReleased = 0;
        			
        		}

    	    }

    	    if(objectID == ID_NO)
    	    {
  //				if(buttonReleased == 0)
    			{
    		        if(objMsg == BTN_MSG_PRESSED)
    		        {
    	
        				buttonReleased = 1;
    				
    					SystemClean();

    			        update = 1;
    		        }
    					
    			}
        		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
        		{
        			buttonReleased = 0;
        			
        		}

    	    }

    	    if(objectID == ID_HOME)
    	    {
    	//		if(buttonReleased == 0)
    			{
    		        if(objMsg == BTN_MSG_PRESSED)
    		        {
        				buttonReleased = 1;
    					startFlag = 0;
    
    					HomeSetup();
    			        update = 1;
    		        }
    					
    			}
        		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
        		{
        			buttonReleased = 0;
        			
        		}

    	    }


    		break;

    		case CREDIT_IMAGE:
    		    if(objectID == ID_1)
    		    {
    				if(buttonReleased == 0)
    				{
    			        if(objMsg == BTN_MSG_PRESSED)
    			        {
    		
    						buttonReleased = 1;
    						String[position] = '1';
    						String[position + 1] = 0;
    			 	  		SetFont((void *) &BigFonts);
    				   		SetColor(TEXT_COLOR);
    				   		//SetColor(GRAY4);
    
    						while(!OutTextXY(TEXT_LEFT, TEXT_TOP, (XCHAR *)String));
    					
    						position++;
    
    				        update = 1;
    			        }
    						
    				}
    				if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
    				{
    					buttonReleased = 0;
    					
    				}
    
    		    }
    
    		    if(objectID == ID_2)
    		    {
    				if(buttonReleased == 0)
    				{
    			        if(objMsg == BTN_MSG_PRESSED)
    			        {
    		
    						buttonReleased = 1;
    						String[position] = '2';
    						String[position + 1] = 0;
    			 	  		SetFont((void *) &BigFonts);
    				   		SetColor(TEXT_COLOR);
    				   		//SetColor(GRAY4);
    
    						while(!OutTextXY(TEXT_LEFT, TEXT_TOP, (XCHAR *)String));
    					
    						position++;
    
    				        update = 1;
    			        }
    						
    				}
    				if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
    				{
    					buttonReleased = 0;
    					
    				}
    
    		    }
    
    		    if(objectID == ID_3)
    		    {
    				if(buttonReleased == 0)
    				{
    			        if(objMsg == BTN_MSG_PRESSED)
    			        {
    		
    						buttonReleased = 1;
    						String[position] = '3';
    						String[position + 1] = 0;
    			 	  		SetFont((void *) &BigFonts);
    				   		SetColor(TEXT_COLOR);
    				   		//SetColor(GRAY4);
    
    						while(!OutTextXY(TEXT_LEFT, TEXT_TOP, (XCHAR *)String));
    					
    						position++;
    
    				        update = 1;
    			        }
    						
    				}
    				if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
    				{
    					buttonReleased = 0;
    					
    				}
    
    		    }
    
    		    if(objectID == ID_4)
    		    {
    				if(buttonReleased == 0)
    				{
    			        if(objMsg == BTN_MSG_PRESSED)
    			        {
    		
    						buttonReleased = 1;
    						String[position] = '4';
    						String[position + 1] = 0;
    			 	  		SetFont((void *) &BigFonts);
    				   		SetColor(TEXT_COLOR);
    				   		//SetColor(GRAY4);
    
    						while(!OutTextXY(TEXT_LEFT, TEXT_TOP, (XCHAR *)String));
    					
    						position++;
    
    				        update = 1;
    			        }
    						
    				}
    				if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
    				{
    					buttonReleased = 0;
    					
    				}
    
    		    }
    
    		    if(objectID == ID_5)
    		    {
    				if(buttonReleased == 0)
    				{
    			        if(objMsg == BTN_MSG_PRESSED)
    			        {
    		
    						buttonReleased = 1;
    						String[position] = '5';
    						String[position + 1] = 0;
    			 	  		SetFont((void *) &BigFonts);
    				   		SetColor(TEXT_COLOR);
    				   		//SetColor(GRAY4);
    
    						while(!OutTextXY(TEXT_LEFT, TEXT_TOP, (XCHAR *)String));
    					
    						position++;
    
    				        update = 1;
    			        }
    						
    				}
    				if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
    				{
    					buttonReleased = 0;
    					
    				}
    
    		    }
    
    		    if(objectID == ID_6)
    		    {
    				if(buttonReleased == 0)
    				{
    			        if(objMsg == BTN_MSG_PRESSED)
    			        {
    		
    						buttonReleased = 1;
    						String[position] = '6';
    						String[position + 1] = 0;
    			 	  		SetFont((void *) &BigFonts);
    				   		SetColor(TEXT_COLOR);
    				   		//SetColor(GRAY4);
    
    						while(!OutTextXY(TEXT_LEFT, TEXT_TOP, (XCHAR *)String));
    					
    						position++;
    
    				        update = 1;
    			        }
    						
    				}
    				if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
    				{
    					buttonReleased = 0;
    					
    				}
    
    		    }
    
    		    if(objectID == ID_7)
    		    {
    				if(buttonReleased == 0)
    				{
    			        if(objMsg == BTN_MSG_PRESSED)
    			        {
    		
    						buttonReleased = 1;
    						String[position] = '7';
    						String[position + 1] = 0;
    			 	  		SetFont((void *) &BigFonts);
    				   		SetColor(TEXT_COLOR);
    				   		//SetColor(GRAY4);
    
    						while(!OutTextXY(TEXT_LEFT, TEXT_TOP, (XCHAR *)String));
    					
    						position++;
    
    				        update = 1;
    			        }
    						
    				}
    				if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
    				{
    					buttonReleased = 0;
    					
    				}
    
    		    }
    
    		    if(objectID == ID_8)
    		    {
    				if(buttonReleased == 0)
    				{
    			        if(objMsg == BTN_MSG_PRESSED)
    			        {
    		
    						buttonReleased = 1;
    						String[position] = '8';
    						String[position + 1] = 0;
    			 	  		SetFont((void *) &BigFonts);
    				   		SetColor(TEXT_COLOR);
    				   		//SetColor(GRAY4);
    
    						while(!OutTextXY(TEXT_LEFT, TEXT_TOP, (XCHAR *)String));
    					
    						position++;
    
    				        update = 1;
    			        }
    						
    				}
    				if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
    				{
    					buttonReleased = 0;
    					
    				}
    
    		    }
    
    		    if(objectID == ID_9)
    		    {
    				if(buttonReleased == 0)
    				{
    			        if(objMsg == BTN_MSG_PRESSED)
    			        {
    		
    						buttonReleased = 1;
    						String[position] = '9';
    						String[position + 1] = 0;
    			 	  		SetFont((void *) &BigFonts);
    				   		SetColor(TEXT_COLOR);
    				   		//SetColor(GRAY4);
    
    						while(!OutTextXY(TEXT_LEFT, TEXT_TOP, (XCHAR *)String));
    					
    						position++;
    
    				        update = 1;
    			        }
    						
    				}
    				if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
    				{
    					buttonReleased = 0;
    					
    				}
    
    		    }
    
    		    if(objectID == ID_0)
    		    {
    				if(buttonReleased == 0)
    				{
    			        if(objMsg == BTN_MSG_PRESSED)
    			        {
    		
    						buttonReleased = 1;
    						String[position] = '0';
    						String[position + 1] = 0;
    			 	  		SetFont((void *) &BigFonts);
    				   		SetColor(TEXT_COLOR);
    				   		//SetColor(GRAY4);
    
    						while(!OutTextXY(TEXT_LEFT, TEXT_TOP, (XCHAR *)String));
    					
    						position++;
    
    				        update = 1;
    			        }
    						
    				}
    				if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
    				{
    					buttonReleased = 0;
    					
    				}
    
    		    }
    
    		    if(objectID == ID_BS)
    		    {
    				if(buttonReleased == 0)
    				{
    			        if(objMsg == BTN_MSG_PRESSED)
    			        {
    						if(position != 0)
    						{
    							position--;
    							//erase digit
    							String[position] = 0;
    
    							SetColor(BOX_BACKGROUND);
    						 	while(!Bar(BOX_LEFT, BOX_TOP, BOX_RIGHT, BOX_BOTTOM));
    
    						   	SetFont((void *) &BigFonts);
    							SetColor(TEXT_COLOR);
    						   	//SetColor(GRAY4);
    
    						 	while(!OutTextXY(TEXT_LEFT, TEXT_TOP, (XCHAR *)String));
    //							DrawDigit(String);
    
    
    						}
    
    				        update = 1;
    			        }
    						
    				}
    				if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
    				{
    					buttonReleased = 0;
    					
    				}
    
    		    }
    
    		    if(objectID == ID_ENTER)
    		    {
    				if(buttonReleased == 0)
    				{
    			        if(objMsg == BTN_MSG_PRESSED)
    			        {
    				        ReadSD();
    				        for(intI = 0; intI < 14; intI++)
    				        {
    				        	SDserialNumber[intI] = SDcard[0][intI];
    				        	
    				        }
    
    				        for(intI = 0; intI < 14; intI++)
    				        {
    				        	SDpinNumber[intI] = SDcard[1][intI];
    				        	
    				        }
    
    						if(!pinOKFlag)
    						{
    							if(serialNumberFlag)
    							{					
    								serialNumberFlag = 0;
    								//new serial number has been entered
    								if(strncmp(SDserialNumber, String, 8) != 0)
    								{
    									SetColor(BOX_BACKGROUND);
    									while(!Bar(BOX_LEFT, BOX_TOP, BOX_RIGHT, BOX_BOTTOM));
    
    									SetFont((void *) &GOLMediumFont);
    									SetColor(BLACK);
    								
    									while(!OutTextXY(TEXT_LEFT, TEXT_TOP, "Invalid serial number"));		//must delay~~~
    
    									timer2 = 30;	//3 seconds
    
    									while(timer2 != 0)
    									{
    										
    									}
    									HomeSetup();
    									update = 1;
    									return;	//not enough characters - bail out
    								}
    								else
    								{
    									update = 1;
    									pinFlag = 1;
    									position = 0;
    									SetColor(BOX_BACKGROUND);
    									while(!Bar(BOX_LEFT, BOX_TOP, BOX_RIGHT, BOX_BOTTOM));
    
    									SetFont((void *) &GOLMediumFont);
    									SetColor(BLACK);
    								
    									while(!OutTextXY(TEXT_LEFT, TEXT_TOP, "Enter PIN number"));
    									return;
    									
    								}
    							}
    							if(pinFlag)
    							{
    								pinFlag = 0;
    								//new serial number has been entered
    								if(strncmp(SDpinNumber, String, 6) != 0)
    								{
    									SetColor(BOX_BACKGROUND);
    									while(!Bar(BOX_LEFT, BOX_TOP, BOX_RIGHT, BOX_BOTTOM));
    
    									SetFont((void *) &GOLMediumFont);
    									SetColor(BLACK);
    								
    									while(!OutTextXY(TEXT_LEFT, TEXT_TOP, "Invalid PIN number"));
    									timer2 = 30;	//3 seconds
    
    									while(timer2 != 0)
    									{
    										
    									}
    									timer2 = 30;	//3 seconds
    
    									while(timer2 != 0)
    									{
    										
    									}
    
    									HomeSetup();		//must delay
    			
    									update = 1;
    									return;	//not enough characters - bail out
    								}
    								else
    								{
    									position = 0;
    									pinOKFlag = 1;
    									TouchStoreCalibration();	//store in flash
    
    									SetColor(BOX_BACKGROUND);
    									while(!Bar(BOX_LEFT, BOX_TOP, BOX_RIGHT, BOX_BOTTOM));
    
    									SetFont((void *) &GOLMediumFont);
    									SetColor(BLACK);
    								
    									while(!OutTextXY(TEXT_LEFT, TEXT_TOP, "Enter code"));
    									update = 1;
    									return;	
    										
    								}
    
    							}
    								
    						}
    
    
    						//Check string
    
    						intJ = 0;
    
    						for(intJ = 0; intJ < codeCount; intJ++)
    						{
    							for(intI = 0; intI < 14; intI++)
    							{
    								tempCode[intI] = SDcard[intJ][intI];
    								
    							}
    
    							foundFlag = 0;
    
    							if(strncmp("000000", String, 6) == 0)	//ignore erased codes
    							{
    								
    							}
    							else if(strncmp(&tempCode[5], String, 6) == 0)
    							{
    								if(tempCode[0] == 'C')
    								{
    									continuousFlag = 1;
    									foundFlag = 1;
    									DaysLeft = 0;
    										
    									SetColor(BOX_BACKGROUND);
    									while(!Bar(BOX_LEFT, BOX_TOP, BOX_RIGHT, BOX_BOTTOM));
    			
    									SetFont((void *) &GOLMediumFont);
    									SetColor(TEXT_COLOR);
    			
    									while(!OutTextXY(TEXT_LEFT, TEXT_TOP, "Valid Code"));
    
    
    								}
    								else
    								{
    									continuousFlag = 0;
    									DaysLeft += 30;
    									foundFlag = 1;
    										
    									SetColor(BOX_BACKGROUND);
    									while(!Bar(BOX_LEFT, BOX_TOP, BOX_RIGHT, BOX_BOTTOM));
    			
    									SetFont((void *) &GOLMediumFont);
    									SetColor(TEXT_COLOR);
    			
    									while(!OutTextXY(TEXT_LEFT, TEXT_TOP, "Valid Code"));
/*    										
    									width = GetImageWidth((void *) &Days);
    									height = GetImageHeight((void *) &Days);
    		
    									PutImage(480 - width, 272 - height, (void *) &Days, 1);
    									SetFont((void *) &GOLSmallFont);
*/    		
    									uitoa(DaysLeft, (BYTE*)String);
    		
    									while(!OutTextXY(365, 250, (XCHAR *)String));	//coordinate here???
    
    									for(intI = 0; intI < 12; intI++)
    									{
    										SDcard[intJ][intI] = '0';
    										
    									}
    									//store in flash
    									StoreSD();
    
    
    								}
    
    									
    
    								date = ReadRTC(0x04) & 0x0F;
    								date += (ReadRTC(0x04) >> 4) * 10;
    								julianDate = (DWORD)date;
    						
    								month = ReadRTC(0x05) & 0x0F;
    								month += (ReadRTC(0x05) >> 4) * 10;
    						
    								year = ReadRTC(0x06) & 0x0F;
    								year = (ReadRTC(0x06) >> 4) * 12;
    						
    								switch(month)
    								{
    									case 1:
    										break;
    						
    									case 2:
    										julianDate += 31;
    										break;
    						
    									case 3:
    										julianDate += 59;
    										break;
    						
    									case 4:
    										julianDate += 90;
    										break;
    						
    									case 5:
    										julianDate += 120;
    										break;
    						
    									case 6:
    										julianDate += 151;
    										break;
    						
    									case 7:
    										julianDate += 181;
    										break;
    						
    									case 8:
    										julianDate += 212;
    										break;
    						
    									case 9:
    										julianDate += 243;
    										break;
    						
    									case 10:
    										julianDate += 273;
    										break;
    						
    									case 11:
    										julianDate += 304;
    										break;
    						
    									case 12:
    										julianDate += 334;
    										break;
    						
    									default:
    										break;
    						
    								}
    						
    								julianDate += (DWORD)year * 365;
    								lastJulian = julianDate;
    								TouchStoreCalibration();	//store in flash
    
    								timer2 = 30;	//3 seconds
    	
    								while(timer2 != 0)
    								{
    									
    								}
    
    								break;
    
    							}
    
    									
    						}
    
    						if(!foundFlag)
    						{
    						 	SetColor(BOX_BACKGROUND);
    						 	while(!Bar(BOX_LEFT, BOX_TOP, BOX_RIGHT, BOX_BOTTOM));
    	
    						 	SetFont((void *) &GOLMediumFont);
    						 	SetColor(TEXT_COLOR);
    						 	
    						 	while(!OutTextXY(TEXT_LEFT, TEXT_TOP, "Invalid Code")); //must delay
    												 	
    							timer2 = 30;	//3 seconds
    
    						 	while(timer2 != 0)
    						 	{
    						 		
    						 	}
    	
    						}
    
    						HomeSetup();	
    
    				        update = 1;
    			        }
    						
    				}
    				if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
    				{
    					buttonReleased = 0;
    					
    				}
    
    		    }
    
    
    			break;
    

    	default:
    		break;
    }



    return (1);
}

/////////////////////////////////////////////////////////////////////////////
// Function: WORD GOLDrawCallback()
// Output: if the function returns non-zero the draw control will be passed to GOL
// Overview: it's a user defined function. GOLDraw() function calls it each
//           time when GOL objects drawing is completed. User drawing should be done here.
//           GOL will not change color, line type and clipping region settings while

//           this function returns zero.
/////////////////////////////////////////////////////////////////////////////
WORD GOLDrawCallback(void)
{
    WORD        value, y, x;    // variables for the slider position
    static WORD prevValue = 0;

    if(update)
    {

        /* User defined graphics:	
		    This draws a series of bars indicating the value/position of the 
			slider's thumb. The height of the bars follows the equation of a 
			parabola "(y-k)^2 = 4a(x-h) with vertex at (k, h) at (60,100) on 
			the display. The value 110 is the 4*a constant. x & y are calculated
			based on the value of the slider thumb. The bars are drawn from 
			60 to 260 in the x-axis and 10 to 100 in the y-axis. Bars are drawn
			every 6 pixels with width of 4 pixels.
			
			Only the bars that are added or removed are drawn. Thus resulting 
			in an efficient customized drawing. 
		*/

        // prevValue will have the current value after drawing or removing bars.
        // reset the update flag


        update = 0;
    }

    return (1);
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

/////////////////////////////////////////////////////////////////////////////
// Function: Timer3 ISR
// Input: none
// Output: none
// Overview: increments tick counter. Tick is approx. 1 ms.
/////////////////////////////////////////////////////////////////////////////
#ifdef __PIC32MX__
    #define __T3_ISR    __ISR(_TIMER_3_VECTOR, ipl4)
#else
    #define __T3_ISR    __attribute__((interrupt, shadow, auto_psv))
#endif

/* */
void __T3_ISR _T3Interrupt(void)
{
    // Clear flag
    #ifdef __PIC32MX__
    mT3ClearIntFlag();
    #else
    IFS0bits.T3IF = 0;
    #endif

	TouchProcessTouch();  

    if(buttonDelay != 0)
    {
    	buttonDelay--;
    }
  


}

    #define __U1RX_ISR    __attribute__((interrupt, shadow, auto_psv))

/*********************************************************************
 * Function:        void _ISR _U1RXInterrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Copies bytes to and from the local UART TX and 
 *					RX FIFOs
 *
 * Note:            None
 ********************************************************************/
void __U1RX_ISR _U1RXInterrupt(void)	//100 ms
//void _ISR __attribute__((__no_auto_psv__)) _U2RXInterrupt(void)
{
	BYTE i;

	// Store a received byte, if pending, if possible
	// Get the byte
	XmtBuffer[0] = U1RXREG;

	U1TXREG = 0x42;
		
	// Clear the interrupt flag so we don't keep entering this ISR
	IFS0bits.U1RXIF = 0;
	
}

    #define __U1TX_ISR    __attribute__((interrupt, shadow, auto_psv))

/*********************************************************************
 * Function:        void _ISR _U1TXInterrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Copies bytes to and from the local UART TX and 
 *					RX FIFOs
 *
 * Note:            None
 ********************************************************************/
void __U1RX_ISR _U1TXInterrupt(void)	//100 ms
//void _ISR __attribute__((__no_auto_psv__)) _U2TXInterrupt(void)
{

		// Clear the TX interrupt flag before transmitting again
		IFS0bits.U1TXIF = 0;

        U1TXREG = XmtBuffer[ptrXmt];
	
		if(XmtBuffer[ptrXmt] == 0x0D)
        {
			xmtFlag = FALSE;
			IEC0bits.U1TXIE = 0;     //disable interrupt
			
        }
        else
        {
            ptrXmt++;
            if(ptrXmt >= TRANSMIT_MSG_SIZE)
            {
                ptrXmt = 0; 
            	
            }
        }

}


/////////////////////////////////////////////////////////////////////////////
// Function: void TickInit(void)
// Input: none
// Output: none
// Overview: Initilizes the tick timer.
/////////////////////////////////////////////////////////////////////////////

/*********************************************************************
 * Section: Tick Delay
 *********************************************************************/
#define SAMPLE_PERIOD       500 // us
#define TICK_PERIOD			(GetPeripheralClock() * SAMPLE_PERIOD) / 4000000

/* */
void TickInit(void)
{

    TMR3 = 0;
    PR3 = TICK_PERIOD;
    IFS0bits.T3IF = 0;  //Clear flag
    IEC0bits.T3IE = 1;  //Enable interrupt
    T3CONbits.TON = 1;  //Run timer

    // Initialize Timer2
	
    TMR2 = 0;
    PR2 = 6250;		//100 ms timer
    IFS0bits.T2IF = 0;  //Clear flag
    IEC0bits.T2IE = 1;  //Enable interrupt
	T2CONbits.TCKPS1 = 1;	//256
	T2CONbits.TCKPS0 = 1;	//256
    T2CONbits.TON = 1;  //Run timer

    U1MODEbits.UARTEN = 1;    //UART enabled
    
    U1MODEbits.BRGH = 1;    //high speed
    U1STAbits.UTXEN = 1;    //transmit enabled
//	U1MODEbits.LPBACK = 1;

    
	U1BRG = 207;	//19.2 at 32 MHz
    
    IFS0bits.U1RXIF = 0;     //clear interrupt flag
    IEC0bits.U1RXIE = 1;     //set interrupt

    OC1CON1bits.OCTSEL = 7;      // peripheral clock
    OC1CON2bits.SYNCSEL = 0x1F;  // period defined in OC1RS register
//    OC1RS  = 5450;    // set PWM period	   ~ 340 us
    OC1RS  = 2500;    // set PWM period	   ~ 82 us
//    OC1R = 2500;                    // PWM off
    OC1R = 2190;                    // PWM off
    OC1CON1bits.OCM = 6;         // PWM mode on

    
}


/*****************************************************************************
  Function:
	void uitoa(WORD Value, BYTE* Buffer)

  Summary:
	Converts an unsigned integer to a decimal string.
	
  Description:
	Converts a 16-bit unsigned integer to a null-terminated decimal string.
	
  Precondition:
	None

  Parameters:
	Value	- The number to be converted
	Buffer	- Pointer in which to store the converted string

  Returns:
  	None
  ***************************************************************************/
void uitoa(WORD Value, BYTE* Buffer)
{
	BYTE i;
	WORD Digit;
	WORD Divisor;
	BOOL Printed = FALSE;

	if(Value)
	{
		for(i = 0, Divisor = 10000; i < 5u; i++)
		{
			Digit = Value/Divisor;
			if(Digit || Printed)
			{
				*Buffer++ = '0' + Digit;
				Value -= Digit*Divisor;
				Printed = TRUE;
			}
			Divisor /= 10;
		}
	}
	else
	{
		*Buffer++ = '0';
	}

	*Buffer = '\0';
}			    

void HomeSetup(void)
{
	SHORT width, height;

    ClearDevice();

    if(!continuousFlag)
    {
    	if(DaysLeft == 0)
    	{
    		CreditCode();
    		return;
    	}

    }

    PutImage(0, 0, (void *) &Main, 1);
	ptrImage1 = &Main;
	imageState = HOME_IMAGE;
		
}

void ReminderImage(void)
{
	imageState = REMINDER_IMAGE;

    ClearDevice();

    PutImage(0, 0, (void *) &Reminder, 1);
	ptrImage1 = &Reminder;

}

void HydraFacial(void)
{
    SHORT   width, height;
	BYTE SW1detected = 0;
	BYTE SW2detected = 0;

	unsigned long buttonIndex;
	BYTE i;

	imageState = SMART_FACIAL;

    ClearDevice();

    while(!PutImage(0, 0, (void *) &L3, 1));
    PutImage(163, 18, (void *) &off, 1);
    pumpOnFlag = FALSE;
	//put PWM in level 3
	OC1R = 2500;                    // PWM off

	timer2 = PWM_DELAY;		//3 sec delay

	while(timer2 != 0)
	{
		
	}

	OC1R = LEVEL3;                    // PWM leve1 3

}

void SettingsImage(void)
{
    SHORT   width, height;
	BYTE SW1detected = 0;
	BYTE SW2detected = 0;

	unsigned long buttonIndex;
	BYTE i;

	imageState = SETTINGS_IMAGE;

    ClearDevice();

    PutImage(0, 0, (void *) &Settings, 1);
	ptrImage1 = &Settings;



}


void SystemClean(void)
{
    SHORT   width, height;
	BYTE SW1detected = 0;
	BYTE SW2detected = 0;

	unsigned long buttonIndex;
	BYTE i;
	char String[4];


    ClearDevice();

    PutImage(0, 0, (void *) &Clean, 1);
	ptrImage1 = &Clean;
    PutImage(46,201, (void *) &L00, 1);
	PutImage(163, 18, (void *) &off, 1);

	//cleanseTimer = 25;
	//percentValue = 0;

//	pumpOnFlag = FALSE;
	imageState = CLEANSE_IMAGE;
	//put PWM in level 3
	OC1R = 2500;                    // PWM off

	timer2 = PWM_DELAY;		//3 sec delay

	while(timer2 != 0)
	{
		
	}

	OC1R = LEVEL3;                    // PWM leve1 3

}




//#define	CREDIT_IMAGE 	9

void CreditCode(void)
{
	unsigned long buttonIndex;
	BYTE i;
	BYTE enterFlag = 0;
	BYTE receiveBuffer[8];
	SHORT width;


	imageState = CREDIT_IMAGE;
	position = 0;

    ClearDevice();

    PutImage(0, 0, (void *) &Credit, 1);

	String[0] = 0;

	SetColor(BOX_BACKGROUND);
	Bar(BOX_LEFT, BOX_TOP, BOX_RIGHT, BOX_BOTTOM);

	SetFont((void *) &GOLMediumFont);
	SetColor(BLACK);
//Display: (24,76), (248,119)

	if(!pinOKFlag)
	{
		width = GetTextWidth((XCHAR *)strSerialNumber, (void *) &GOLMediumFont);
			   
		while(!OutTextXY((272 - width) >> 1, 76, (XCHAR *)strSerialNumber));
		serialNumberFlag = 1;
			
	}
	else
	{
		width = GetTextWidth((XCHAR *)strCode, (void *) &GOLMediumFont);
			   
		while(!OutTextXY((272 - width) >> 1, 76, (XCHAR *)strCode));
		update = 1;
		 	
	}


}



    #define __T2_ISR    __attribute__((interrupt, shadow, auto_psv))


void __T2_ISR _T2Interrupt(void)	//100 ms
{
	SHORT height, width;
	volatile BYTE temp;
	SHORT save1;
	SHORT shift;
	BYTE clear;
	BYTE clear1;
	BYTE clear2;

    // Clear flag
    IFS0bits.T2IF = 0;

    if(imageState == CLEANSE_IMAGE)
    {
    	if(pumpOnFlag)
    	{
    		if(cleanseTimer != 0)
    		{
    			cleanseTimer--;
    			if(cleanseTimer == 0)
    			{
    				cleanseTimer = 10;
					percentValue++;
					switch(percentValue)
					{
						case 1:
							PutImage(46,201, (void *) &L25, 1);
							break;

						case 2:
							PutImage(46,201, (void *) &L50, 1);
							break;

						case 3:
							PutImage(46,201, (void *) &L75, 1);
							break;

						case 4:
							PutImage(46,201, (void *) &L100, 1);
							break;

						case 5:
							pumpOnFlag = FALSE;
							PutImage(163, 18, (void *) &off, 1);
							portB &= PUMP_OFF;
							WriteMPCB(portB);

							break;

						default:
							percentValue = 0;
							PutImage(100,45, (void *) &L00, 1);

							break;


					}

    			}
    		}
    	}
    }

    if((!continuousFlag) && (DaysLeft != 0))
    {
    	RTCtimer--;

    	if(RTCtimer == 0)
    	{
    		RTCtimer = 36;
    		RTCflag = TRUE;
    			
    	}
    		
    }

    if(daysLeftTimer != 0)
    {
    	daysLeftTimer--;
    	if(daysLeftTimer == 0)
    	{
    		if(imageState ==  HOME_IMAGE)
    		{
    			HomeSetup();

    		}
    		
    	}

    	
    }

    if(timer2 != 0)
    {
    	timer2--;
    }
    
    if(hiddenButtonFlagT || hiddenButtonFlagB)		//hidden button pressed?
    {
    	if(hiddenButtonTimer != 0)	//second must be pressedd within 3 sec
    	{
    		hiddenButtonTimer--;
    		if(hiddenButtonTimer == 0)
    		{
//				SetColor(GRAY0);		//red circle off
//				FillCircle(CIRCLE_X, CIRCLE_Y, CIRCLE_SIZE);
    			hiddenButtonFlagT = 0;	
    			hiddenButtonFlagB = 0;	
    		}
    	}
    }

    if(!continuousFlag)
    {
    	if(imageState ==  HOME_IMAGE)
    	{
    		if(DaysLeft < 11)
    		{
    			if(DaysLeft != daysLeftOld)
    			{
    				daysLeftOld = DaysLeft;
    				//less than 11 dayss
//    				width = GetImageWidth((void *) &Days);
//    				height = GetImageHeight((void *) &Days);
    
    				//PutImage(480 - width, 272 - height, (void *) &Days, 1);
    				SetFont((void *) &GOLSmallFont);
    
    				uitoa(DaysLeft, (BYTE*)String);
    
    				while(!OutTextXY(365, 250, (XCHAR *)String));	//coordinate here???
    				daysLeftTimer = 50;


    			}

    		}
    			
    	}
    		
    }

	if(floatSwitchFlag)		//cannister full?
	{
		if(ledBlinkTimer == 0)
		{
			ledBlinkTimer = LED_BLINK_TIME;
			if(blinkFlag)
			{
				blinkFlag = FALSE;
				changeBlink = TRUE;

			}
			else
			{
				blinkFlag = TRUE;	
				changeBlink = TRUE;
								
			}
		}
		else
		{
			ledBlinkTimer--;
		}
			
	}

    if(startFlag)	//LED's on?
    {
    	tempSeconds++;
    	if(tempSeconds >= 10)
    	{
    		tempSeconds = 0;
    		LEDseconds++;
    	}

    	if(oldSeconds != LEDseconds)
    	{
    		oldSeconds = LEDseconds;

    		ChangeTime(oldSeconds);

    	}


    }


    if(buttonTimer != 0)
    {
    	buttonTimer--;
    }


}

void ChangeTime(int seconds)
{
/*
    SHORT   width, height;
	BYTE minutes;
	BYTE second;
	BYTE temp;
	BYTE i;
	char character[4];
	char string[8];

	minutes = seconds / 60;

	temp = minutes * 60;

	second = seconds - temp;

    uitoa(minutes, (BYTE*)character);
	
	i = 0;

	if(strlen(character) < 2)
	{
		string[i++] = character[0];
		string[i++] = ':';
	}
	else
	{
		string[i++] = character[0];
		string[i++] = character[1];
		string[i++] = ':';
	}
	
    uitoa(second, (BYTE*)character);

	if(strlen(character) < 2)
	{
		string[i++] = '0';
		string[i++] = character[0];
	}
	else
	{
		string[i++] = character[0];
		string[i++] = character[1];
	}

	string[i] = 0;

	SetColor(GRAY0);
 	while(!Bar(86,66,399,144));		 

    SetFont((void *) &EurostileLTStd);

    // Get text width and height
    width = GetTextWidth(string, (void *) &EurostileLTStd);
    height = GetTextHeight((void *) &EurostileLTStd);

    SetFont((void *) &EurostileLTStd);

    SetColor(GRAY4);

    OutTextXY(242 - (width >> 1), 105 - (height >> 1), (XCHAR *)string);

	SetColor(BLACK);
*/	
}

void CreateButtons(void)
{
    BtnCreate
    (
        ID_WHOLE,                    // object’s ID
        0,
        0,
        480,
        272,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme


    BtnCreate
    (
        ID_SMART_FACIAL,                    // object’s ID
        0,
        73,
        247,
        156,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme


    BtnCreate
    (
        ID_CLEANSE_MODE,                    // object’s ID
        0,
        195,
        247,
        275,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme

    BtnCreate
    (
        ID_SETTINGS_MODE,                    // object’s ID
        0,
        318,
        247,
        399,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme


    BtnCreate
    (
        ID_HOME,                    // object’s ID
        21,
        18,
        61,
        67,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme

    BtnCreate
    (
        ID_ON,                    // object’s ID
        163,
        18,
        256,
        67,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme


    BtnCreate
    (
        ID_SETTING1,                    // object’s ID
        18,
        171,
        45,
        199,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme

    BtnCreate
    (
        ID_SETTING2,                    // object’s ID
        59,
        128,
        86,
        155,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme

    BtnCreate
    (
        ID_SETTING3,                    // object’s ID
        123,
        107,
        151,
        135,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme

    BtnCreate
    (
        ID_SETTING4,                    // object’s ID
        189,
        127,
        217,
        155,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme

    BtnCreate
    (
        ID_SETTING5,                    // object’s ID
        226,
        171,
        254,
        199,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme

    BtnCreate
    (
        ID_YES,                    // object’s ID
        0,
        275,
        124,
        354,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme

    BtnCreate
    (
        ID_NO,                    // object’s ID
        150,
        275,
        270,
        354,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme

    BtnCreate
    (
        ID_BUTTON1,                    // object’s ID
        150,
        275,
        270,
        354,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme

    BtnCreate
    (
        ID_BUTTON2,                    // object’s ID
        112,
        96,
        306,
        173,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme


    BtnCreate
    (
        ID_BUTTON3,                    // object’s ID
        112,
        183,
        306,
        259,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme


 
    BtnCreate		
    (
		ID_BL_INVISIBLE,		//bottom left invisible
		0, 
		202, 
		70, 
		272, 
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );

    BtnCreate		
    (
		ID_BR_INVISIBLE,		//bottom right invisible
		410, 
		202, 
		480, 
		272, 
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );
/*

	 Display: (24,76), (248,119)
      1: (30,128), (86,184)
      2: (107,128), (163,184)
      3: (186,128), (242,184)

      4: (30,195), (86,251)
      5: (107,195), (163,251)
      6: (186,195), (242,251)

      7: (30,262), (86,320)
      8: (107,262), (163,320)
      9: (186,262), (242,320)

     ?:(30,128), (86,387)
      0:(107,107), (163,387)
     ?:(186,186), (242,387)






*/
    BtnCreate		
    (
		ID_1,		//1
    	30, 
    	128, 
    	86, 
    	184, 
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme

    );

    BtnCreate		
    (
		ID_2,		//2
    	107, 
    	128, 
    	163, 
    	184, 
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );

//    3: (186,128), (242,184)
    BtnCreate		
    (
		ID_3,		//3
    	186, 
    	128, 
    	242, 
    	184, 
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );
  //  4: (30,195), (86,251)

    BtnCreate		
    (
		ID_4,		//4
    	30, 
    	195, 
    	86, 
    	251, 
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );

    //5: (107,195), (163,251)
    BtnCreate		
    (
		ID_5,		//5
    	107, 
    	195, 
    	163, 
    	251, 
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );

    //6: (186,195), (242,251)
    BtnCreate		
    (
		ID_6,		//6
    	186, 
    	195, 
    	242, 
    	251, 
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );
    //7: (30,262), (86,320)

    BtnCreate		
    (
		ID_7,		//7
    	30, 
    	262, 
    	86, 
    	320, 
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );

    //8: (107,262), (163,320)
    BtnCreate		
    (
		ID_8,		//8
    	107, 
    	262, 
    	163, 
    	320, 
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );

    //9: (186,262), (242,320)
    BtnCreate		
    (
		ID_9,		//9
    	186, 
    	262, 
    	242, 
    	320, 
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
     );

    //0:(107,309), (163,387)
    BtnCreate		
    (
		ID_0,		//0
    	107, 
    	309, 
    	163, 
    	387, 
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );

    //BS:(30,309), (86,387)
    BtnCreate	   
    (
		ID_BS,		//backspasce
    	30, 
    	309, 
    	86, 
    	387, 
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );

    //Enter:(186,309), (242,387)
    BtnCreate		
    (
		ID_ENTER,		//enter
    	186, 
    	309, 
    	242, 
    	387, 
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );

	
}
/****************************************************************************
  Function:
    BOOL USB_ApplicationEventHandler( BYTE address, USB_EVENT event,
                    void *data, DWORD size )

  Description:
    This routine handles USB events sent from the USB Embedded Host stack.

  Precondition:
    None

  Parameters:
    BYTE address    - Address of the USB device generating the event
    USB_EVENT event - Event that has occurred
    void *data      - Pointer to the data associated with the event
    DWORD size      - Size of the data pointed to by *data

  Return Values:
    TRUE    - The event was handled successfully
    FALSE   - The even was not handled successfully

  Remarks:
    We will default to returning TRUE for unknown events, and let operation
    proceed.  Other applications may wish to return FALSE, since we are not
    really handling the event.
  ***************************************************************************/
BOOL USB_ApplicationEventHandler(BYTE address, USB_EVENT event, void *data, DWORD size)
{
    WORD    yPos, TextHeight;

    switch(event)
    {
        case EVENT_VBUS_REQUEST_POWER:

            // We will let everything attach.
            return (TRUE);

        case EVENT_VBUS_RELEASE_POWER:

            // We are not monitoring power allocation, so we have
            // nothing to update.
            return (TRUE);

        case EVENT_HUB_ATTACH:
        case EVENT_UNSUPPORTED_DEVICE:
        case EVENT_CANNOT_ENUMERATE:
        case EVENT_CLIENT_INIT_ERROR:
        case EVENT_OUT_OF_MEMORY:
        case EVENT_UNSPECIFIED_ERROR:   // This should never occur
            usbErrorCode = USBHostDeviceStatus(1);

            // Shut down the USB.
            //USBHostShutdown();
            break;

        default:
            return (FALSE);
    }

    // USB error messages will only appear when in the Menu Screens.
//    if(screenState != DISPLAY_DEMOSELECTION)
  //      return (TRUE);

    // go back to demo selection when exiting this function.
//    screenState = CREATE_DEMOSELECTION;

    // The following code displays the different USB errors that can
    // occur. Example, inserting an unsupported device or device
    // does not enumerate.
    // clear the screen
    SetColor(WHITE);
    ClearDevice();

    // set up the font to display the error messages
    SetFont((void *) &GOLFontDefault);
    SetColor(BRIGHTBLUE);
    TextHeight = GetTextHeight((void *) &GOLFontDefault);
    yPos = TextHeight * 2;

    // output the standard USB error string
    MoveTo((GetMaxX() - GetTextWidth((XCHAR *)ErrMsgStandard, (void *) &GOLFontDefault)) >> 1, yPos);
    WAIT_UNTIL_FINISH(OutText((XCHAR *)ErrMsgStandard));
    yPos += TextHeight;

    switch(event)
    {
        case EVENT_HUB_ATTACH:
            MoveTo((GetMaxX() - GetTextWidth((XCHAR *)ErrMsgHUBAttachedStr, (void *) &GOLFontDefault)) >> 1, yPos);
            WAIT_UNTIL_FINISH(OutText((XCHAR *)ErrMsgHUBAttachedStr));
            yPos += TextHeight;
            MoveTo((GetMaxX() - GetTextWidth((XCHAR *)ErrNotSupported, (void *) &GOLFontDefault)) >> 1, yPos);
            WAIT_UNTIL_FINISH(OutText((XCHAR *)ErrNotSupported));
            break;

        case EVENT_UNSUPPORTED_DEVICE:
            MoveTo((GetMaxX() - GetTextWidth((XCHAR *)ErrMsgUDAttachedStr, (void *) &GOLFontDefault)) >> 1, yPos);
            WAIT_UNTIL_FINISH(OutText((XCHAR *)ErrMsgUDAttachedStr));
            yPos += TextHeight;
            MoveTo((GetMaxX() - GetTextWidth((XCHAR *)ErrNotSupported, (void *) &GOLFontDefault)) >> 1, yPos);
            WAIT_UNTIL_FINISH(OutText((XCHAR *)ErrNotSupported));
            break;

        case EVENT_CANNOT_ENUMERATE:
            MoveTo((GetMaxX() - GetTextWidth((XCHAR *)ErrMsgEnumerationStr, (void *) &GOLFontDefault)) >> 1, yPos);
            WAIT_UNTIL_FINISH(OutText((XCHAR *)ErrMsgEnumerationStr));
            yPos += TextHeight;
            MoveTo((GetMaxX() - GetTextWidth((XCHAR *)ErrMsgFailedStr, (void *) &GOLFontDefault)) >> 1, yPos);
            WAIT_UNTIL_FINISH(OutText((XCHAR *)ErrMsgFailedStr));
            break;

        case EVENT_CLIENT_INIT_ERROR:
            MoveTo((GetMaxX() - GetTextWidth((XCHAR *)ErrMsgClientInitStr, (void *) &GOLFontDefault)) >> 1, yPos);
            WAIT_UNTIL_FINISH(OutText((XCHAR *)ErrMsgClientInitStr));
            yPos += TextHeight;
            MoveTo((GetMaxX() - GetTextWidth((XCHAR *)ErrMsgFailedStr, (void *) &GOLFontDefault)) >> 1, yPos);
            WAIT_UNTIL_FINISH(OutText((XCHAR *)ErrMsgFailedStr));
            break;

        case EVENT_OUT_OF_MEMORY:
            MoveTo((GetMaxX() - GetTextWidth((XCHAR *)ErrMsgOutofMemoryStr, (void *) &GOLFontDefault)) >> 1, yPos);
            WAIT_UNTIL_FINISH(OutText((XCHAR *)ErrMsgOutofMemoryStr));
            break;

        case EVENT_UNSPECIFIED_ERROR:
            MoveTo((GetMaxX() - GetTextWidth((XCHAR *)ErrMsgUnpecifiedErrStr, (void *) &GOLFontDefault)) >> 1, yPos);
            WAIT_UNTIL_FINISH(OutText((XCHAR *)ErrMsgUnpecifiedErrStr));
            break;

        default:
            return (TRUE);
    }

    yPos += TextHeight;
    MoveTo((GetMaxX() - GetTextWidth((XCHAR *)MsgTouchToProceedStr, (void *) &GOLFontDefault)) >> 1, yPos);
    WAIT_UNTIL_FINISH(OutText((XCHAR *)MsgTouchToProceedStr));

    // wait for touch
    while(TouchGetX() == -1);

    return (TRUE);
}

/************************************************************************
Function: void MonitorDriveMedia( void )

Precondition: None

Overview: This function calls the background tasks necessary to support
          USB Host operation.  Upon initial insertion of the media, it
          initializes the file system support and reads the volume label.
          Upon removal of the media, the volume label is marked invalid.

Input: None

Output: None
*************************************************************************/
void MonitorDriveMedia(void)
{
    BYTE        mediaPresentNow;
    BYTE        mountTries;
    SearchRec   searchRecord;


    // Call to USBTasks() is assumed to be done in the main. If not you can
    // call it here.
    USBTasks();	
    mediaPresentNow = USBHostMSDSCSIMediaDetect();
    if(mediaPresentNow != mediaPresent)
    {
        if(mediaPresentNow)
        {
            mountTries = 10;
            while(!FSInit() && mountTries--);
            if(!mountTries)
            {
                //UART2PrintString("APP: Could not mount media\r\n");
                mediaPresent = FALSE;
            }
            else
            {
                mediaPresent = TRUE;

            }
        }
        else
        {
            mediaPresent = FALSE;
        }
    }
}

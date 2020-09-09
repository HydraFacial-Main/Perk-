///////////////////////////////////////////////////////////////////////
//
//  File: I2C.c
//
//  Copyright (C)  2001-2008, All Rights Reserved
//
//  Authored by : Alpha Resources, LLC
//                6717 Rovilla Rd.
//                Blacklick, OH 43004
//				  614-245-4059
//
//                Rob Reasons - Senior Software Engineer
//
//  Project     : Wave II
//
//
//  Description : This module contains the misc functions.
//
//  Functions   :	GetButton()
//				   
//                        
///////////////////////////////////////////////////////////////////////
#include "GenericTypeDefs.h"
#include "HardwareProfile.h"
#include <p24Fxxxx.h>

#define     ACKBIT      0x00            // ACK bit
#define     NAKBIT      0x80            // NAK bit
extern BYTE portB;
extern BYTE portA;

void bit_in(unsigned char *data);   // Bit input function
void bit_out(unsigned char data);   // Bit output function
WORD ReadMPC(BYTE address);
void WriteMPC(BYTE address, BYTE data);
void ack(void);
void nack(void);
void bstart(void);                      // Start condition
void bstop(void);                       // Stop condition
unsigned char byte_out(unsigned char data);
unsigned char byte_in(unsigned char ack);


void WriteMPC(BYTE address, BYTE data)
{
	bstart();                       // Generate Start condition
	byte_out(0x40);
	byte_out(address);
	byte_out(data);
	bstop();                        // Generate Stop condition

}


WORD ReadMPC(BYTE address)
{
	WORD data;


	bstart();                       // Generate Start condition
	byte_out(0x40);              // Output control byte
	byte_out(address);// Output address MSB

	bstart();                       // Generate Start condition
	byte_out(0x41);              // Output control byte
	data = byte_in(NAKBIT);        // Input data byte
	nack();                     // Begin ACK polling
	bstop();                        // Generate Stop condition

	return(data);

}

void WriteRTC(BYTE address, BYTE data)
{
	bstart();                       // Generate Start condition
	byte_out(0xD0);
	byte_out(address);
	byte_out(data);
	bstop();                        // Generate Stop condition

}


WORD ReadRTC(BYTE address)
{
	WORD data;


	bstart();                       // Generate Start condition
	byte_out(0xD0);              // Output control byte
	byte_out(address);// Output address MSB

	bstart();                       // Generate Start condition
	byte_out(0xD1);              // Output control byte
	data = byte_in(NAKBIT);        // Input data byte
	nack();                     // Begin ACK polling
	bstop();                        // Generate Stop condition

	return(data);

}

/********************************************************************
 * Function:        unsigned char byte_in(unsigned char ack)
 *
 * Description:     This function inputs a byte from the I2C bus.
 *                  Depending on the value of ack, it will also
 *                  transmit either an ACK or a NAK bit.
 *******************************************************************/
unsigned char byte_in(unsigned char ack)
{
    unsigned char i;                // Loop counter
    unsigned char retval;           // Return value

    retval = 0;
    for (i = 0; i < 8u; i++)         // Loop through each bit
    {
        retval = retval << 1;       // Shift left for next bit
        bit_in(&retval);            // Input bit
    }
    bit_out(ack);                   // Output ACK/NAK bit

    return retval;
} // end byte_in(void)


unsigned char byte_out(unsigned char data)
{
    unsigned char i;                // Loop counter
    unsigned char ack;              // ACK bit

    ack = 0;
    for (i = 0; i < 8u; i++)         // Loop through each bit
    {
        bit_out(data);              // Output bit
        data = data << 1;           // Shift left for next bit
    }
    bit_in(&ack);                   // Input ACK bit

    return ack;
} // end byte_out(unsigned char data)


/********************************************************************
 * Function:        void bit_out(unsigned char data)
 *
 * Description:     This function outputs a bit to the I2C bus.
 *******************************************************************/
void bit_out(unsigned char data)
{
	BYTE i;

    SCL1 = 0;                        // Ensure SCL is low
    Nop();
    if (data & 0x80)                // Check if next bit is high
    {
        SDA1_TRIS = 1;               // Release SDA to be pulled high
    }
    else
    {
        SDA1_TRIS = 0;               // Configure SDA as an output
        SDA1 = 0;                    // Pull SDA low
    }

    for(i = 0; i < 5; i++)
    {
    	
    }

    SCL1 = 1;                        // Pull SCL high to clock bit

    for(i = 0; i < 25; i++)
    {
    	
    }

    SCL1 = 0;                        // Pull SCL low for next bit

    for(i = 0; i < 20; i++)
    {
    	
    }


} // end bit_out(unsigned char data)

/********************************************************************
 * Function:        void bit_in(unsigned char *data)
 *
 * Description:     This function inputs a bit from the I2C bus.
 *******************************************************************/
void bit_in(unsigned char *data)
{
    SCL1 = 0;                        // Ensure SCL is low
    Nop();
    SDA1_TRIS = 1;                   // Configure SDA as an input
    SCL1 = 1;                        // Bring SCL high to begin transfer
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    *data &= 0xFE;                  // Assume next bit is low
    if (SDA1)                        // Check if SDA is high
    {
        *data |= 0x01;              // If high, set next bit
    }
    SCL1 = 0;                        // Bring SCL low again
	Nop();
	Nop();
	Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
	Nop();
} // end bit_in(unsigned char *data)


/********************************************************************
 * Function:        void bstart(void)
 *
 * Description:     This function generates an I2C Start condition.
 *******************************************************************/
void bstart(void)
{
	BYTE i;

    SDA1_TRIS = 1;                   // Ensure SDA is high
    SCL1 = 1;                        // Ensure SCL is high
    for(i = 0; i < 30; i++)
    {
    	
    }

    SDA1_TRIS = 0;                   // Configure SDA as an output
	for(i = 0; i < 30; i++)
	{
		
	}

    SDA1 = 0;                        // Pull SDA low

    SCL1 = 0;                        // Pull SCL low

	for(i = 0; i < 20; i++)
	{
		
	}
    
} // end bstart(void)

/********************************************************************
 * Function:        void bstop(void)
 *
 * Description:     This function generates an I2C Stop condition.
 *******************************************************************/
void bstop(void)
{
	BYTE i;

    SCL1 = 0;                        // Ensure SCL is low
    SDA1_TRIS = 0;                   // Configure SDA as an output
    SDA1 = 0;                        // Ensure SDA low

	for(i = 0; i < 25; i++)
	{
		
	}

    SCL1 = 1;                        // Pull SCL high

	for(i = 0; i < 25; i++)
	{
		
	}

    SDA1_TRIS = 1;                   // Allow SDA to be pulled high

	for(i = 0; i < 25; i++)
	{
		
	}
} // end bstop(void)


void ack(void)
{
	SDA1 = 0;
	SDA1_TRIS = 0;                   // Configure SDA as an output

	SCL1 = 1;
	Nop();
	Nop();
	SCL1 = 0;

}

void nack(void)
{
	SDA1 = 1;
	SDA1_TRIS = 0;                   // Configure SDA as an output
	SCL1 = 1;
	Nop();
	SCL1 = 0;

}

void MPCInit(void)
{
	WriteMPC(0x00, 0xFF);	//IODIRA, all input


	Nop();
	Nop();
	Nop();
	Nop();
	Nop();

	WriteMPC(0x0C, 0x07);	//pull ups for port a, 0 - 2

	Nop();
	Nop();
	Nop();
	Nop();
	Nop();
	Nop();
	Nop();
	Nop();
	Nop();
	Nop();
	Nop();


	WriteMPC(0x01, 0x00);	//IODIRB, all output


	Nop();
	Nop();
	Nop();
	Nop();
	Nop();

	WriteMPC(0x0D, 0x00);	//pull ups for port b, none

	Nop();
	Nop();
	Nop();
	Nop();
	Nop();


}

void WriteMPCB(BYTE port)
{

	WriteMPC(0x13, port);	//port b, data
	
}

void WriteMPCA(BYTE port)
{
	WriteMPC(0x12, port);	//port a, data
	
}


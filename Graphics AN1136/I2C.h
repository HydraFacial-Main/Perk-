///////////////////////////////////////////////////////////////////////
//
//  File Name   : I2C.h
//
//  Copyright (C)  2001-, All Rights Reserved
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
//  Description : This module contains all the utilites functions and variables.
//
//  Original Author : Rob Reasons
//
//
///////////////////////////////////////////////////////////////////////

#ifndef EEPROM_H

#define   EEPROM_H

#define PWM_ADDRESS	0x81
#define MAG_ADDRESS	0x84
#define LED_ADDRESS	0x85
#define PUMP_ADDRESS	0x82
#define STANDBY_ADDRESS	0x86

#define MAG_READ_ADDRESS	0x87
#define SWITCH_ADDRESS		0x83


void MCPInit(void);
void WriteMPCB(BYTE port);
void WriteRTC(BYTE address, BYTE data);
WORD ReadRTC(BYTE address);
void WriteMPCA(BYTE port);


#endif


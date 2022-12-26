/**
 * @file CbusLibXC8 hardware.h
 * @copyright (C) 2022 Konrad Orlowski     <syspixie@gmail.com>
 * 
 *  CbusLibXC8 is licensed under the:
 *      Creative Commons Attribution-NonCommercial-ShareAlike 4.0
 *      International License.
 *  To view a copy of this license, visit:
 *      http://creativecommons.org/licenses/by-nc-sa/4.0/
 *  Postal address: Creative Commons, PO Box 1866, Mountain View, CA 94042, USA
 * 
 * License summary:
 * 
 *  You are free to:
 *      Share, copy and redistribute the material in any medium or format;
 *      Adapt, remix, transform, and build upon the material.
 *  The licensor cannot revoke these freedoms as long as you follow the license
 *  terms.
 *
 *  Attribution: You must give appropriate credit, provide a link to the
 *  license, and indicate if changes were made. You may do so in any reasonable
 *  manner, but not in any way that suggests the licensor endorses you or your
 *  use.
 * 
 *  NonCommercial: You may not use the material for commercial purposes. **(see
 *  note below)
 *
 *  ShareAlike: If you remix, transform, or build upon the material, you must
 *  distribute your contributions under the same license as the original.
 *
 *  No additional restrictions: You may not apply legal terms or technological
 *  measures that legally restrict others from doing anything the license
 *  permits.
 * 
 * ** For commercial use, please contact the original copyright holder(s) to
 * agree licensing terms.
 * 
 *  This software is distributed in the hope that it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.
 * 
 *******************************************************************************
 * 
 * CbusLibXC8 and CbusLibBootXC8 is code that can form the basis of the firmware
 * required to build a Model Electronic Railway Group (MERG) CBUS compatible
 * module.  It was developed using the Microchip XC8 compiler and MPLAB X IDE,
 * and targets PIC18F MCUs with built-in CAN bus peripherals.
 * 
 * The software was created to meet the CBUS specification as detailed in the
 * "Developer's Guide for CBUS" version 6c.
 * 
 * Credit to Mike Bolton and Gil Fuchs for the development of CBUS, and to
 * Pete Brownlow, Ian Hogg, and many other MERG members, for the development
 * and testing of CBUS modules, hardware and firmware, over the past 15 years.
 * 
 * MERG CBUS: https://www.merg.org.uk/resources/cbus
 */

/**
 * @author Konrad Orlowski
 * @date November 2022
 */

#ifndef HARDWARE_H
#define	HARDWARE_H

#ifdef	__cplusplus
extern "C" {
#endif


#include "global.h"


#if defined(_18F25K80) || defined(_18LF25K80)
#define CPU_FAMILY_PIC18_K80
#define CPU_ID P18F25K80
#define FLASH_TOP 0x007FFF              // 32K bytes
#define FLASH_PAGE_SIZE 64
#define EEPROM_TOP 0x03FF               // 1024 bytes
typedef uint16_t flashAddr_t;
typedef uint8_t flashPageOffset_t;
#endif
#if defined(_18F26K80) || defined(_18LF26K80)
#define CPU_FAMILY_PIC18_K80
#define CPU_ID P18F26K80
#define FLASH_TOP 0x00FFFF              // 64K bytes
#define FLASH_PAGE_SIZE 64
#define EEPROM_TOP 0x03FF               // 1024 bytes
typedef uint16_t flashAddr_t;
typedef uint8_t flashPageOffset_t;
#endif
#if defined(_18F25K83) || defined(_18LF25K83)
#define CPU_FAMILY_PIC18_K83
#define CPU_ID P18F26K83        // There isn't a 25K83 definded, so we lie...
#define FLASH_TOP 0x007FFF              // 32K bytes
#define FLASH_PAGE_SIZE 128
#define EEPROM_TOP 0x03FF               // 1024 bytes
typedef uint16_t flashAddr_t;
typedef uint8_t flashPageOffset_t;
#endif
#if defined(_18F26K83) || defined(_18LF26K83)
#define CPU_FAMILY_PIC18_K83
#define CPU_ID P18F26K83
#define FLASH_TOP 0x00FFFF              // 64K bytes
#define FLASH_PAGE_SIZE 128
#define EEPROM_TOP 0x03FF               // 1024 bytes
typedef uint16_t flashAddr_t;
typedef uint8_t flashPageOffset_t;
#endif
#if defined(_18F27Q83)
#define CPU_FAMILY_PIC18_Q83Q84
#define CPU_ID P18F26K83        // There is a 27Q83 definded, but not in FCU
#define FLASH_TOP 0x01FFFF              // 128K bytes
#define FLASH_PAGE_SIZE 256
#define FLASH_PAGE_BUFFER_ADDRESS 0x3700
#define EEPROM_TOP 0x03FF               // 1024 bytes
typedef uint24_t flashAddr_t;
typedef uint16_t flashPageOffset_t;
#endif

#define _XTAL_FREQ 64000000             // Used by __delay_ms and __delay_us

#if defined(CPU_FAMILY_PIC18_K80)
#define INTERRUPT_DisableHigh() (INTCONbits.GIEH = 0)
#define INTERRUPT_EnableHigh() (INTCONbits.GIEH = 1)
#define INTERRUPT_DisableLow() (INTCONbits.GIEL = 0)
#define INTERRUPT_EnableLow() (INTCONbits.GIEL = 1)
#define ECAN_BUFFERS_BASE_ADDRESS 0x0F60    // Access Bank address (RXB0CON)
#endif
#if defined(CPU_FAMILY_PIC18_K83)
#define INTERRUPT_DisableHigh() (INTCON0bits.GIEH = 0)
#define INTERRUPT_EnableHigh() (INTCON0bits.GIEH = 1)
#define INTERRUPT_DisableLow() (INTCON0bits.GIEL = 0)
#define INTERRUPT_EnableLow() (INTCON0bits.GIEL = 1)
#define IVT_BASE_ADDRESS 0x000B50
#define IVT_NUM_VECTORS 80
#define ECAN_BUFFERS_BASE_ADDRESS 0x3F80    // Access Bank address (RXB0CON)
#endif
#if defined(CPU_FAMILY_PIC18_Q83Q84)
#define INTERRUPT_DisableHigh() (INTCON0bits.GIEH = 0)
#define INTERRUPT_EnableHigh() (INTCON0bits.GIEH = 1)
#define INTERRUPT_DisableLow() (INTCON0bits.GIEL = 0)
#define INTERRUPT_EnableLow() (INTCON0bits.GIEL = 1)
#define IVT_BASE_ADDRESS 0x000C50
#define IVT_NUM_VECTORS 129
#define CAN1_BUFFERS_BASE_ADDRESS 0x3800
#endif


#ifdef	__cplusplus
}
#endif

#endif	/* HARDWARE_H */

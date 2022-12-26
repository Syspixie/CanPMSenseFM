/**
 * @file CanPMSense adcc.h
 * @copyright (C) 2022 Konrad Orlowski     <syspixie@gmail.com>
 * 
 *  CanPMSense is licensed under the:
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
 * CanPMSense is the firmware for a point motor driver module which uses
 * current sense feedback as part of the control system.
 * 
 * Designed primarily for driving analog point motors that don't have an
 * automatic cutoff when the target position is reached (often called 'stall'
 * motors), but will work equally well with those that do.
 * 
 * The module is Model Electronic Railway Group (MERG) CBUS compatible.  The
 * software is based on the CbusLibXC8 library.  It was developed using the
 * Microchip XC8 compiler and MPLAB X IDE, and targets PIC18F MCUs with built-in
 * CAN bus peripherals.
 * 
 * CbusLibXC8: https://github.com/Syspixie/CbusLibXC8
 * MERG CBUS: https://www.merg.org.uk/resources/cbus
 */

/**
 * @author Konrad Orlowski
 * @date December 2022
 */

#ifndef ADCC_H
#define	ADCC_H

#ifdef	__cplusplus
extern "C" {
#endif


#include "global.h"


    typedef enum {
        channel_Fb1 =  0x0,
        channel_Fb4 =  0x5,
        channel_Fb3 =  0x8,
        channel_Fb2 =  0xC,
        channel_VSS =  0x3B,
        channel_Temp =  0x3C,
        channel_DAC1 =  0x3D,
        channel_FVR_Buffer1 =  0x3E,
        channel_FVR_Buffer2 =  0x3F
    } adcc_channel_t;


    void initAdcc(void);
    uint16_t adccRead(uint8_t channel);


#ifdef	__cplusplus
}
#endif

#endif	/* ADCC_H */

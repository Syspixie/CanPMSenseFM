/**
 * @file CanPMSense adcc.c
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
 * Code to interface to the ADC.
 * 
 * @author Konrad Orlowski
 * @date December 2022
 * 
 * @note Uses hardware Analog-to-Digital Converter with Computation (ADC2)
 * module, and Fixed Voltage Reference (FVR) module.
 */


#include "adcc.h"


#define CHANNEL_CHANGE_ADACQ 512    // 512 clock periods = 8us acquisition time


/**
 * Initialises the ADC peripheral.
 */
void initAdcc() {

    // Initialise reference voltage to 4.096V
    FVRCON = 0x83;  // CDAFVR off; FVREN enabled; TSRNG Lo_range; ADFVR 4x; TSEN disabled; 

    // ADLTH 0; 
    ADLTHL = 0x00;
    ADLTHH = 0x00;

    // ADUTH 0; 
    ADUTHL = 0x00;
    ADUTHH = 0x00;

    // ADSTPT 0; 
    ADSTPTL = 0x00;
    ADSTPTH = 0x00;

    // ADACC 0; 
    ADACCU = 0x00;

    // ADRPT 0; 
    ADRPT = 0x00;

    // ADACQ 0; 
    ADACQL = 0x00;
    ADACQH = 0x00;
    
    ADCAP = 0x00;   // ADCAP Additional uC disabled; 

    // ADPRE 0; 
    ADPREL = 0x00;
    ADPREH = 0x00;
    
    ADCON1 = 0x00;  // ADDSEN disabled; ADGPOL digital_low; ADIPEN disabled; ADPPOL Vss; 
    ADCON2 = 0x00;  // ADCRS 0; ADMD Basic_mode; ADACLR disabled; ADPSIS RES; 
    ADCON3 = 0x00;  // ADCALC First derivative of Single measurement; ADTMD disabled; ADSOI ADGO not cleared; 
    ADSTAT = 0x00;  // ADMATH registers not updated; 
    ADREF = 0x03;   // ADNREF VSS; ADPREF FVR; 
    ADACT = 0x00;   // ADACT disabled; 
    ADCLK = 0x1F;   // ADCS FOSC/64; 
    ADCON0 = 0x84;  // ADGO stop; ADFM right; ADON enabled; ADCS FOSC/ADCLK; ADCONT disabled; 
  
    // Turn on the ADC peripheral
    ADCON0bits.ADON = 1;
}

/**
 * Collects and reads the value of an ADC channel.
 * 
 * @param adccChan ADC channel
 * @return 12-bit ADC reading
 */
uint16_t adccRead(uint8_t channel) {

    static uint8_t prevChannel = 0xFF;

    // If changing channel...
    if (channel != prevChannel) {
        ADPCH = channel;                // Select the channel
        ADACQ = CHANNEL_CHANGE_ADACQ;   // Set acquisition delay
    }

    // Start the conversion
    ADCON0bits.ADGO = 1;

    // Wait for conversion to complete
    while (ADCON0bits.ADGO);

    // If changing channel...
    if (channel != prevChannel) {
        ADACQ = 0;                      // Clear acquisition delay
        prevChannel = channel;          // Update channel
    }

    // Return conversion value
//    bytes16_t v;
//    v.valueL = ADRESL;
//    v.valueH = ADRESH;
//    return v.value;
    return ADRES;
}

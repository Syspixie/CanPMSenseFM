/**
 * @file CanPMSense canpmsense.h
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

#ifndef CANPMSENSE_H
#define	CANPMSENSE_H

#ifdef	__cplusplus
extern "C" {
#endif


#include "global.h"
#include "ports.h"


// Node variables: default ADC settings
// (200mOhm resistor, 200 gain, 4.096V ADC reference, 4096 bits)
#define NV_ADC_MOVING 1380      // 34.5mA
#define NV_ADC_STALLED 2600     // 65.0mA


// get/set EnA1 aliases
#define EnA1_TRIS                 TRISBbits.TRISB5
#define EnA1_LAT                  LATBbits.LATB5
#define EnA1_PORT                 PORTBbits.RB5
#define EnA1_WPU                  WPUBbits.WPUB5
#define EnA1_OD                   ODCONBbits.ODCB5
#define EnA1_ANS                  ANSELBbits.ANSELB5
#define EnA1_SetHigh()            do { LATBbits.LATB5 = 1; } while(0)
#define EnA1_SetLow()             do { LATBbits.LATB5 = 0; } while(0)
#define EnA1_Toggle()             do { LATBbits.LATB5 = ~LATBbits.LATB5; } while(0)
#define EnA1_GetValue()           PORTBbits.RB5
#define EnA1_SetDigitalInput()    do { TRISBbits.TRISB5 = 1; } while(0)
#define EnA1_SetDigitalOutput()   do { TRISBbits.TRISB5 = 0; } while(0)
#define EnA1_SetPullup()          do { WPUBbits.WPUB5 = 1; } while(0)
#define EnA1_ResetPullup()        do { WPUBbits.WPUB5 = 0; } while(0)
#define EnA1_SetPushPull()        do { ODCONBbits.ODCB5 = 0; } while(0)
#define EnA1_SetOpenDrain()       do { ODCONBbits.ODCB5 = 1; } while(0)
#define EnA1_SetAnalogMode()      do { ANSELBbits.ANSELB5 = 1; } while(0)
#define EnA1_SetDigitalMode()     do { ANSELBbits.ANSELB5 = 0; } while(0)

// get/set EnB1 aliases
#define EnB1_TRIS                 TRISAbits.TRISA1
#define EnB1_LAT                  LATAbits.LATA1
#define EnB1_PORT                 PORTAbits.RA1
#define EnB1_WPU                  WPUAbits.WPUA1
#define EnB1_OD                   ODCONAbits.ODCA1
#define EnB1_ANS                  ANSELAbits.ANSELA1
#define EnB1_SetHigh()            do { LATAbits.LATA1 = 1; } while(0)
#define EnB1_SetLow()             do { LATAbits.LATA1 = 0; } while(0)
#define EnB1_Toggle()             do { LATAbits.LATA1 = ~LATAbits.LATA1; } while(0)
#define EnB1_GetValue()           PORTAbits.RA1
#define EnB1_SetDigitalInput()    do { TRISAbits.TRISA1 = 1; } while(0)
#define EnB1_SetDigitalOutput()   do { TRISAbits.TRISA1 = 0; } while(0)
#define EnB1_SetPullup()          do { WPUAbits.WPUA1 = 1; } while(0)
#define EnB1_ResetPullup()        do { WPUAbits.WPUA1 = 0; } while(0)
#define EnB1_SetPushPull()        do { ODCONAbits.ODCA1 = 0; } while(0)
#define EnB1_SetOpenDrain()       do { ODCONAbits.ODCA1 = 1; } while(0)
#define EnB1_SetAnalogMode()      do { ANSELAbits.ANSELA1 = 1; } while(0)
#define EnB1_SetDigitalMode()     do { ANSELAbits.ANSELA1 = 0; } while(0)

// get/set Fb1 aliases
#define Fb1_TRIS                 TRISAbits.TRISA0
#define Fb1_LAT                  LATAbits.LATA0
#define Fb1_PORT                 PORTAbits.RA0
#define Fb1_WPU                  WPUAbits.WPUA0
#define Fb1_OD                   ODCONAbits.ODCA0
#define Fb1_ANS                  ANSELAbits.ANSELA0
#define Fb1_SetHigh()            do { LATAbits.LATA0 = 1; } while(0)
#define Fb1_SetLow()             do { LATAbits.LATA0 = 0; } while(0)
#define Fb1_Toggle()             do { LATAbits.LATA0 = ~LATAbits.LATA0; } while(0)
#define Fb1_GetValue()           PORTAbits.RA0
#define Fb1_SetDigitalInput()    do { TRISAbits.TRISA0 = 1; } while(0)
#define Fb1_SetDigitalOutput()   do { TRISAbits.TRISA0 = 0; } while(0)
#define Fb1_SetPullup()          do { WPUAbits.WPUA0 = 1; } while(0)
#define Fb1_ResetPullup()        do { WPUAbits.WPUA0 = 0; } while(0)
#define Fb1_SetPushPull()        do { ODCONAbits.ODCA0 = 0; } while(0)
#define Fb1_SetOpenDrain()       do { ODCONAbits.ODCA0 = 1; } while(0)
#define Fb1_SetAnalogMode()      do { ANSELAbits.ANSELA0 = 1; } while(0)
#define Fb1_SetDigitalMode()     do { ANSELAbits.ANSELA0 = 0; } while(0)

// get/set EnA2 aliases
#define EnA2_TRIS                 TRISBbits.TRISB1
#define EnA2_LAT                  LATBbits.LATB1
#define EnA2_PORT                 PORTBbits.RB1
#define EnA2_WPU                  WPUBbits.WPUB1
#define EnA2_OD                   ODCONBbits.ODCB1
#define EnA2_ANS                  ANSELBbits.ANSELB1
#define EnA2_SetHigh()            do { LATBbits.LATB1 = 1; } while(0)
#define EnA2_SetLow()             do { LATBbits.LATB1 = 0; } while(0)
#define EnA2_Toggle()             do { LATBbits.LATB1 = ~LATBbits.LATB1; } while(0)
#define EnA2_GetValue()           PORTBbits.RB1
#define EnA2_SetDigitalInput()    do { TRISBbits.TRISB1 = 1; } while(0)
#define EnA2_SetDigitalOutput()   do { TRISBbits.TRISB1 = 0; } while(0)
#define EnA2_SetPullup()          do { WPUBbits.WPUB1 = 1; } while(0)
#define EnA2_ResetPullup()        do { WPUBbits.WPUB1 = 0; } while(0)
#define EnA2_SetPushPull()        do { ODCONBbits.ODCB1 = 0; } while(0)
#define EnA2_SetOpenDrain()       do { ODCONBbits.ODCB1 = 1; } while(0)
#define EnA2_SetAnalogMode()      do { ANSELBbits.ANSELB1 = 1; } while(0)
#define EnA2_SetDigitalMode()     do { ANSELBbits.ANSELB1 = 0; } while(0)

// get/set EnB2 aliases
#define EnB2_TRIS                 TRISAbits.TRISA3
#define EnB2_LAT                  LATAbits.LATA3
#define EnB2_PORT                 PORTAbits.RA3
#define EnB2_WPU                  WPUAbits.WPUA3
#define EnB2_OD                   ODCONAbits.ODCA3
#define EnB2_ANS                  ANSELAbits.ANSELA3
#define EnB2_SetHigh()            do { LATAbits.LATA3 = 1; } while(0)
#define EnB2_SetLow()             do { LATAbits.LATA3 = 0; } while(0)
#define EnB2_Toggle()             do { LATAbits.LATA3 = ~LATAbits.LATA3; } while(0)
#define EnB2_GetValue()           PORTAbits.RA3
#define EnB2_SetDigitalInput()    do { TRISAbits.TRISA3 = 1; } while(0)
#define EnB2_SetDigitalOutput()   do { TRISAbits.TRISA3 = 0; } while(0)
#define EnB2_SetPullup()          do { WPUAbits.WPUA3 = 1; } while(0)
#define EnB2_ResetPullup()        do { WPUAbits.WPUA3 = 0; } while(0)
#define EnB2_SetPushPull()        do { ODCONAbits.ODCA3 = 0; } while(0)
#define EnB2_SetOpenDrain()       do { ODCONAbits.ODCA3 = 1; } while(0)
#define EnB2_SetAnalogMode()      do { ANSELAbits.ANSELA3 = 1; } while(0)
#define EnB2_SetDigitalMode()     do { ANSELAbits.ANSELA3 = 0; } while(0)

// get/set Fb2 aliases
#define Fb2_TRIS                 TRISBbits.TRISB4
#define Fb2_LAT                  LATBbits.LATB4
#define Fb2_PORT                 PORTBbits.RB4
#define Fb2_WPU                  WPUBbits.WPUB4
#define Fb2_OD                   ODCONBbits.ODCB4
#define Fb2_ANS                  ANSELBbits.ANSELB4
#define Fb2_SetHigh()            do { LATBbits.LATB4 = 1; } while(0)
#define Fb2_SetLow()             do { LATBbits.LATB4 = 0; } while(0)
#define Fb2_Toggle()             do { LATBbits.LATB4 = ~LATBbits.LATB4; } while(0)
#define Fb2_GetValue()           PORTBbits.RB4
#define Fb2_SetDigitalInput()    do { TRISBbits.TRISB4 = 1; } while(0)
#define Fb2_SetDigitalOutput()   do { TRISBbits.TRISB4 = 0; } while(0)
#define Fb2_SetPullup()          do { WPUBbits.WPUB4 = 1; } while(0)
#define Fb2_ResetPullup()        do { WPUBbits.WPUB4 = 0; } while(0)
#define Fb2_SetPushPull()        do { ODCONBbits.ODCB4 = 0; } while(0)
#define Fb2_SetOpenDrain()       do { ODCONBbits.ODCB4 = 1; } while(0)
#define Fb2_SetAnalogMode()      do { ANSELBbits.ANSELB4 = 1; } while(0)
#define Fb2_SetDigitalMode()     do { ANSELBbits.ANSELB4 = 0; } while(0)

// get/set EnA3 aliases
#define EnA3_TRIS                 TRISCbits.TRISC6
#define EnA3_LAT                  LATCbits.LATC6
#define EnA3_PORT                 PORTCbits.RC6
#define EnA3_WPU                  WPUCbits.WPUC6
#define EnA3_OD                   ODCONCbits.ODCC6
#define EnA3_ANS                  ANSELCbits.ANSELC6
#define EnA3_SetHigh()            do { LATCbits.LATC6 = 1; } while(0)
#define EnA3_SetLow()             do { LATCbits.LATC6 = 0; } while(0)
#define EnA3_Toggle()             do { LATCbits.LATC6 = ~LATCbits.LATC6; } while(0)
#define EnA3_GetValue()           PORTCbits.RC6
#define EnA3_SetDigitalInput()    do { TRISCbits.TRISC6 = 1; } while(0)
#define EnA3_SetDigitalOutput()   do { TRISCbits.TRISC6 = 0; } while(0)
#define EnA3_SetPullup()          do { WPUCbits.WPUC6 = 1; } while(0)
#define EnA3_ResetPullup()        do { WPUCbits.WPUC6 = 0; } while(0)
#define EnA3_SetPushPull()        do { ODCONCbits.ODCC6 = 0; } while(0)
#define EnA3_SetOpenDrain()       do { ODCONCbits.ODCC6 = 1; } while(0)
#define EnA3_SetAnalogMode()      do { ANSELCbits.ANSELC6 = 1; } while(0)
#define EnA3_SetDigitalMode()     do { ANSELCbits.ANSELC6 = 0; } while(0)

// get/set EnB3 aliases
#define EnB3_TRIS                 TRISCbits.TRISC7
#define EnB3_LAT                  LATCbits.LATC7
#define EnB3_PORT                 PORTCbits.RC7
#define EnB3_WPU                  WPUCbits.WPUC7
#define EnB3_OD                   ODCONCbits.ODCC7
#define EnB3_ANS                  ANSELCbits.ANSELC7
#define EnB3_SetHigh()            do { LATCbits.LATC7 = 1; } while(0)
#define EnB3_SetLow()             do { LATCbits.LATC7 = 0; } while(0)
#define EnB3_Toggle()             do { LATCbits.LATC7 = ~LATCbits.LATC7; } while(0)
#define EnB3_GetValue()           PORTCbits.RC7
#define EnB3_SetDigitalInput()    do { TRISCbits.TRISC7 = 1; } while(0)
#define EnB3_SetDigitalOutput()   do { TRISCbits.TRISC7 = 0; } while(0)
#define EnB3_SetPullup()          do { WPUCbits.WPUC7 = 1; } while(0)
#define EnB3_ResetPullup()        do { WPUCbits.WPUC7 = 0; } while(0)
#define EnB3_SetPushPull()        do { ODCONCbits.ODCC7 = 0; } while(0)
#define EnB3_SetOpenDrain()       do { ODCONCbits.ODCC7 = 1; } while(0)
#define EnB3_SetAnalogMode()      do { ANSELCbits.ANSELC7 = 1; } while(0)
#define EnB3_SetDigitalMode()     do { ANSELCbits.ANSELC7 = 0; } while(0)

// get/set Fb3 aliases
#define Fb3_TRIS                 TRISBbits.TRISB0
#define Fb3_LAT                  LATBbits.LATB0
#define Fb3_PORT                 PORTBbits.RB0
#define Fb3_WPU                  WPUBbits.WPUB0
#define Fb3_OD                   ODCONBbits.ODCB0
#define Fb3_ANS                  ANSELBbits.ANSELB0
#define Fb3_SetHigh()            do { LATBbits.LATB0 = 1; } while(0)
#define Fb3_SetLow()             do { LATBbits.LATB0 = 0; } while(0)
#define Fb3_Toggle()             do { LATBbits.LATB0 = ~LATBbits.LATB0; } while(0)
#define Fb3_GetValue()           PORTBbits.RB0
#define Fb3_SetDigitalInput()    do { TRISBbits.TRISB0 = 1; } while(0)
#define Fb3_SetDigitalOutput()   do { TRISBbits.TRISB0 = 0; } while(0)
#define Fb3_SetPullup()          do { WPUBbits.WPUB0 = 1; } while(0)
#define Fb3_ResetPullup()        do { WPUBbits.WPUB0 = 0; } while(0)
#define Fb3_SetPushPull()        do { ODCONBbits.ODCB0 = 0; } while(0)
#define Fb3_SetOpenDrain()       do { ODCONBbits.ODCB0 = 1; } while(0)
#define Fb3_SetAnalogMode()      do { ANSELBbits.ANSELB0 = 1; } while(0)
#define Fb3_SetDigitalMode()     do { ANSELBbits.ANSELB0 = 0; } while(0)

// get/set EnA4 aliases
#define EnA4_TRIS                 TRISCbits.TRISC4
#define EnA4_LAT                  LATCbits.LATC4
#define EnA4_PORT                 PORTCbits.RC4
#define EnA4_WPU                  WPUCbits.WPUC4
#define EnA4_OD                   ODCONCbits.ODCC4
#define EnA4_ANS                  ANSELCbits.ANSELC4
#define EnA4_SetHigh()            do { LATCbits.LATC4 = 1; } while(0)
#define EnA4_SetLow()             do { LATCbits.LATC4 = 0; } while(0)
#define EnA4_Toggle()             do { LATCbits.LATC4 = ~LATCbits.LATC4; } while(0)
#define EnA4_GetValue()           PORTCbits.RC4
#define EnA4_SetDigitalInput()    do { TRISCbits.TRISC4 = 1; } while(0)
#define EnA4_SetDigitalOutput()   do { TRISCbits.TRISC4 = 0; } while(0)
#define EnA4_SetPullup()          do { WPUCbits.WPUC4 = 1; } while(0)
#define EnA4_ResetPullup()        do { WPUCbits.WPUC4 = 0; } while(0)
#define EnA4_SetPushPull()        do { ODCONCbits.ODCC4 = 0; } while(0)
#define EnA4_SetOpenDrain()       do { ODCONCbits.ODCC4 = 1; } while(0)
#define EnA4_SetAnalogMode()      do { ANSELCbits.ANSELC4 = 1; } while(0)
#define EnA4_SetDigitalMode()     do { ANSELCbits.ANSELC4 = 0; } while(0)

// get/set EnB4 aliases
#define EnB4_TRIS                 TRISCbits.TRISC5
#define EnB4_LAT                  LATCbits.LATC5
#define EnB4_PORT                 PORTCbits.RC5
#define EnB4_WPU                  WPUCbits.WPUC5
#define EnB4_OD                   ODCONCbits.ODCC5
#define EnB4_ANS                  ANSELCbits.ANSELC5
#define EnB4_SetHigh()            do { LATCbits.LATC5 = 1; } while(0)
#define EnB4_SetLow()             do { LATCbits.LATC5 = 0; } while(0)
#define EnB4_Toggle()             do { LATCbits.LATC5 = ~LATCbits.LATC5; } while(0)
#define EnB4_GetValue()           PORTCbits.RC5
#define EnB4_SetDigitalInput()    do { TRISCbits.TRISC5 = 1; } while(0)
#define EnB4_SetDigitalOutput()   do { TRISCbits.TRISC5 = 0; } while(0)
#define EnB4_SetPullup()          do { WPUCbits.WPUC5 = 1; } while(0)
#define EnB4_ResetPullup()        do { WPUCbits.WPUC5 = 0; } while(0)
#define EnB4_SetPushPull()        do { ODCONCbits.ODCC5 = 0; } while(0)
#define EnB4_SetOpenDrain()       do { ODCONCbits.ODCC5 = 1; } while(0)
#define EnB4_SetAnalogMode()      do { ANSELCbits.ANSELC5 = 1; } while(0)
#define EnB4_SetDigitalMode()     do { ANSELCbits.ANSELC5 = 0; } while(0)

// get/set Fb4 aliases
#define Fb4_TRIS                 TRISAbits.TRISA5
#define Fb4_LAT                  LATAbits.LATA5
#define Fb4_PORT                 PORTAbits.RA5
#define Fb4_WPU                  WPUAbits.WPUA5
#define Fb4_OD                   ODCONAbits.ODCA5
#define Fb4_ANS                  ANSELAbits.ANSELA5
#define Fb4_SetHigh()            do { LATAbits.LATA5 = 1; } while(0)
#define Fb4_SetLow()             do { LATAbits.LATA5 = 0; } while(0)
#define Fb4_Toggle()             do { LATAbits.LATA5 = ~LATAbits.LATA5; } while(0)
#define Fb4_GetValue()           PORTAbits.RA5
#define Fb4_SetDigitalInput()    do { TRISAbits.TRISA5 = 1; } while(0)
#define Fb4_SetDigitalOutput()   do { TRISAbits.TRISA5 = 0; } while(0)
#define Fb4_SetPullup()          do { WPUAbits.WPUA5 = 1; } while(0)
#define Fb4_ResetPullup()        do { WPUAbits.WPUA5 = 0; } while(0)
#define Fb4_SetPushPull()        do { ODCONAbits.ODCA5 = 0; } while(0)
#define Fb4_SetOpenDrain()       do { ODCONAbits.ODCA5 = 1; } while(0)
#define Fb4_SetAnalogMode()      do { ANSELAbits.ANSELA5 = 1; } while(0)
#define Fb4_SetDigitalMode()     do { ANSELAbits.ANSELA5 = 0; } while(0)

// get/set Err aliases
#define Err_TRIS                 TRISAbits.TRISA4
#define Err_LAT                  LATAbits.LATA4
#define Err_PORT                 PORTAbits.RA4
#define Err_WPU                  WPUAbits.WPUA4
#define Err_OD                   ODCONAbits.ODCA4
#define Err_ANS                  ANSELAbits.ANSELA4
#define Err_SetHigh()            do { LATAbits.LATA4 = 1; } while(0)
#define Err_SetLow()             do { LATAbits.LATA4 = 0; } while(0)
#define Err_Toggle()             do { LATAbits.LATA4 = ~LATAbits.LATA4; } while(0)
#define Err_GetValue()           PORTAbits.RA4
#define Err_SetDigitalInput()    do { TRISAbits.TRISA4 = 1; } while(0)
#define Err_SetDigitalOutput()   do { TRISAbits.TRISA4 = 0; } while(0)
#define Err_SetPullup()          do { WPUAbits.WPUA4 = 1; } while(0)
#define Err_ResetPullup()        do { WPUAbits.WPUA4 = 0; } while(0)
#define Err_SetPushPull()        do { ODCONAbits.ODCA4 = 0; } while(0)
#define Err_SetOpenDrain()       do { ODCONAbits.ODCA4 = 1; } while(0)
#define Err_SetAnalogMode()      do { ANSELAbits.ANSELA4 = 1; } while(0)
#define Err_SetDigitalMode()     do { ANSELAbits.ANSELA4 = 0; } while(0)


    void appResetEeprom(bool init);
    void appResetFlash(bool init);
    void appInit(void);
    void appProcessCbusEvent(uint8_t eventIndex);
    void appProcess(void);
    int8_t appGenerateCbusMessage(void);
    void appEnterFlimMode(void);
    void appLeaveFlimMode(void);
    bool appValidateNodeVar(uint8_t varIndex, uint8_t curValue, uint8_t newValue);
    void appNodeVarChanged(uint8_t varIndex, uint8_t oldValue, uint8_t curValue);
    bool appValidateEventVar(uint8_t eventIndex, uint8_t varIndex, uint8_t curValue, uint8_t newValue);
    void appEventVarChanged(uint8_t eventIndex, uint8_t varIndex, uint8_t oldValue, uint8_t curValue);
    void appEventRemoved(uint8_t eventIndex);
    void appTimerIsr(void);


#ifdef	__cplusplus
}
#endif

#endif	/* CANPMSENSE_H */

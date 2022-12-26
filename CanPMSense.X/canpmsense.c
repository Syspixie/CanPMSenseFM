/**
 * @file CanPMSense canpmsense.c
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
 * Application specific code for CanPMSense.
 * 
 * @author Konrad Orlowski
 * @date December 2022
 */


#include "canpmsense.h"
#include "millis.h"
#include "eeprom.h"
#include "flash.h"
#include "txmsg.h"
#include "event.h"
#include "module.h"
#include "adcc.h"


#define NUM_PM 4                // Number of point motors

#define UPDATE_MS 10            // 10ms between PM updates
#define ACTIVITY_WAIT_MS 50     // 50ms wait for activity
#define TIMEOUT_MS 4000         // 4s move timeout

#define DEBUG_MS 250            // 250ms between debug messages

// Point motor state
typedef enum {
    pmStateUnknown,             // Unknown position
    pmStateA,                   // Position A
    pmStateB,                   // Position B
    pmStateActivate,            // Waiting to be activated
    pmStateActivated,           // Set to move
    pmStateMoving,              // Confirmed moving or stalled
    pmStateAtTarget             // At target state
} pmState_t;

// Point motor flags
typedef union {
    struct {
        unsigned stallMode : 1;         // Checking for stall
        unsigned events : 1;            // Generate produced events
    };
    uint8_t byte;
} pmFlags_t;

// Point motor data
typedef struct {
    pmState_t state;            // Current state
    pmState_t targetState;      // Target state when moving
    uint16_t moveStarted;       // Time move started
    uint8_t adccChan;           // ADC channel for feedback
    uint16_t adccReading;       // Last ADC reading
    uint16_t filteredReading;   // Exponential filter of ADC readings
    uint16_t activeThreshold;   // ADC threshold for checking activity
    uint16_t senseThreshold;    // ADC threshold for sensing stall or completion
    uint8_t shift;              // Debounce shift register
    pmFlags_t flags;            // Flags
} pm_t;

// Point motor node variable data
typedef struct {
    uint16_t adcActive;         // ADC reading for PM moving
    uint16_t adcSense;          // ADC reading for PM stalled or completed
} nvPMData_t;

// Point motor node variable flags
typedef union {
    struct {
        unsigned startupActivate : 1;       // Activate PMs at startup
        unsigned activateSequential : 1;    // Activate PMs in sequence (rather than in parallel)
        unsigned activateToA : 1;           // Activate PMs to A...
        unsigned toAWhenUnknown : 1;        // ... but only when unknown
        unsigned cutoffOnStall : 1;         // Stop PM on stall or completion (rather than timeout)
    };
    uint8_t byte;
} nvFlags_t;

// Node variables
typedef struct {
    nvPMData_t pmData[NUM_PM];  // Point motor data
    nvFlags_t flags;            // Flags
    uint8_t debugPM;            // Debug
} appNV_t;


// NVs in FLASH memory
extern const appNV_t appNV __at(NODE_VAR_ADDRESS);


// PM data
pm_t pointMotor[NUM_PM] = {
    {pmStateActivate, pmStateUnknown, 0, channel_Fb1, 0, 0, 0, 0, 0, {.byte = 0}},
    {pmStateActivate, pmStateUnknown, 0, channel_Fb2, 0, 0, 0, 0, 0, {.byte = 0}},
    {pmStateActivate, pmStateUnknown, 0, channel_Fb3, 0, 0, 0, 0, 0, {.byte = 0}},
    {pmStateActivate, pmStateUnknown, 0, channel_Fb4, 0, 0, 0, 0, 0, {.byte = 0}}
};

bool updateAdccValues = false;  // Set by ISR to indicate update required
uint16_t lastDebug = 0;         // Time of last debug message
uint8_t activating = 0;         // PM activation state

uint8_t pmx;                    // PM index (0 to NUM_PM - 1)
pm_t* pm;               // Pointer to PM data corresponding to pmx


/**
 * Initialise EEPROM.
 * 
 * @param init Flag indicating EEPROM is uninitialised or wrong version.
 */
void appResetEeprom(bool init) {

    // Initialise application non-volatile values
    writeEeprom8(0, pmStateUnknown);    // Last known state of PM 1
    writeEeprom8(1, pmStateUnknown);    // Last known state of PM 2
    writeEeprom8(2, pmStateUnknown);    // Last known state of PM 3
    writeEeprom8(3, pmStateUnknown);    // Last known state of PM 4
}

/**
 * Initialise FLASH (events & node variables).
 * 
 * @param init Flag indicating FLASH is uninitialised or wrong version.
 * 
 * @note The calling routine calls flushFlashCache() after calling this routine.
 */
void appResetFlash(bool init) {

    // Initialise application node variables
    for (pmx = 0; pmx < NUM_PM; ++pmx) {
        // Default ADC settings
        writeFlashCached16((flashAddr_t) &appNV.pmData[pmx].adcActive, NV_ADC_MOVING);
        writeFlashCached16((flashAddr_t) &appNV.pmData[pmx].adcSense, NV_ADC_STALLED);
    }
    // Sequential PM activation; stop on stall or completion
    writeFlashCached8((flashAddr_t) &appNV.flags.byte, 0b00011111);
    // No debug messages
    writeFlashCached8((flashAddr_t) &appNV.debugPM, 0);
}

/**
 * Initialises I/O ports
 */
static void initAppPorts() {
    
    // Clear TRIS for outputs
    TRISA &= 0b11100101;    // A4:Err A3:EnB2 A1:EnB1
    TRISB &= 0b11011101;    // B5:EnA1 B1:EnA2
    TRISC &= 0b00001111;    // C7:EnB3 C6:EnA3 C5:EnB4 C4:EnA4

    // Set ANSEL for analog inputs
    ANSELA |= 0b00100001;   // A5:Fb4 A0:Fb1
    ANSELB |= 0b00010001;   // B4:Fb2 B0:Fb3
    ANSELC |= 0b00000000;

    // Clear WPU for outputs and analog inputs
    WPUA &= 0b11000100;     // A5:Fb4 A4:Err A3:EnB2 A1:EnB1 A0:Fb1
    WPUB &= 0b11001100;     // B5:EnA1 B4:Fb2 B1:EnA2 B0:Fb3
    WPUC &= 0b00001111;     // C7:EnB3 C6:EnA3 C5:EnB4 C4:EnA4
}

/**
 * Initialises the application.
 */
void appInit() {

    initAppPorts();
    initAdcc();

    // Initialise PM data
    for (pmx = 0; pmx < NUM_PM; ++pmx) {
        pm = &pointMotor[pmx];

        // Calculate thresholds for PM
        const nvPMData_t* nv = &appNV.pmData[pmx];
        pm->activeThreshold = nv->adcActive / 2;
        pm->senseThreshold = (nv->adcActive + nv->adcSense) / 2;
        pm->flags.stallMode = (pm->senseThreshold > nv->adcActive) ? 1 : 0;
    }

    // PM activation at startup
    if (appNV.flags.startupActivate) {
        activating = (appNV.flags.activateSequential) ? 1 : NUM_PM + 1;
    }
}

/**
 * A txmsg function to send a produced ACON or ACOF event.
 * 
 * @param en Event number.
 * @param opc Opcode.
 * @return 1: send response; 0: no response.
 * @post cbusMsg[] ACOx response.
 */
static int8_t txMsgACOx(uint8_t en, uint8_t opc) {

    cbusMsg[0] = opc;
    cbusMsg[3] = 0;
    cbusMsg[4] = en;

    return 1;
}

/**
 * Starts PM movement by setting state to activated.
 * 
 * @pre pmx and pm
 * @param targetState The state that the PM is moving to.
 */
static void setActivated(pmState_t targetState) {

    pm->state = pmStateActivated;
    pm->targetState = targetState;
    pm->moveStarted = getMillisShort();
    pm->shift = pm->flags.stallMode ? 0x00 : 0xFF;
    pm->flags.events = 0;
}

/**
 * Initiates a PM move to 'A'.
 * 
 * @pre pmx and pm
 */
static void moveToA() {

    setActivated(pmStateA);

    switch (pmx) {
        case 0: {
            EnB1_SetLow();
            EnA1_SetHigh();
        } break;
        case 1: {
            EnB2_SetLow();
            EnA2_SetHigh();
        } break;
        case 2: {
            EnB3_SetLow();
            EnA3_SetHigh();
        } break;
        case 3: {
            EnB4_SetLow();
            EnA4_SetHigh();
        } break;
    }
}

/**
 * Initiates a PM move to 'B'.
 * 
 * @pre pmx and pm
 */
static void moveToB() {

    setActivated(pmStateB);

    switch (pmx) {
        case 0: {
            EnA1_SetLow();
            EnB1_SetHigh();
        } break;
        case 1: {
            EnA2_SetLow();
            EnB2_SetHigh();
        } break;
        case 2: {
            EnA3_SetLow();
            EnB3_SetHigh();
        } break;
        case 3: {
            EnA4_SetLow();
            EnB4_SetHigh();
        } break;
    }
}

/**
 * Stops PM movement.
 * 
 * @pre pmx and pm
 * 
 * @note Final state is written to EEPROM.
 */
static void stopMove() {

    switch (pmx) {
        case 0: {
            EnA1_SetLow();
            EnB1_SetLow();
        } break;
        case 1: {
            EnA2_SetLow();
            EnB2_SetLow();
        } break;
        case 2: {
            EnA3_SetLow();
            EnB3_SetLow();
        } break;
        case 3: {
            EnA4_SetLow();
            EnB4_SetLow();
        } break;
    }

    // Determine final state
    pmState_t finalState;
    if (pm->state == pmStateAtTarget) {

        finalState = pm->targetState;   // Reached target

    } else {

        finalState = pmStateUnknown;    // Not at target
        pm->flags.events = 0;           // No events
    }
    pm->state = finalState;

    // Save this as the last known state
    pmState_t s = readEeprom8(pmx);
    if (s != finalState) writeEeprom8(pmx, finalState);

    // Generate events
    if (pm->flags.events) {

        if (finalState == pmStateA) {

            // Combined A:ON B:OFF event - ON
            enqueueTXMsg(txMsgACOx, pmx + 1, OPC_ACON);
            // Separate A:ON event
            enqueueTXMsg(txMsgACOx, NUM_PM + ((uint8_t) (pmx << 1)) + 1, OPC_ACON);

        } else {

            // Combined A:ON B:OFF event - OFF
            enqueueTXMsg(txMsgACOx, pmx + 1, OPC_ACOF);
            // Separate B:ON event
            enqueueTXMsg(txMsgACOx, NUM_PM + ((uint8_t) (pmx << 1)) + 2, OPC_ACON);
        }
    }
}

/**
 * Reads the ADC data for a PM.  Stops the PM if inactive, when target reached,
 * or when timed out.
 * 
 * @pre pmx and pm
 */
static void process() {

    // Get channel ADC reading
    pm->adccReading = adccRead(pm->adccChan);

    // Calculate new filtered value
    uint16_t v = pm->filteredReading << 3;
    v -= pm->filteredReading;
    v += pm->adccReading;
    pm->filteredReading = v >> 3;

    // Done if PM not activated or moving
    if (pm->state < pmStateActivated) return;

    // Check for timeout
    uint16_t m = getMillisShort() - pm->moveStarted;
    if (m > TIMEOUT_MS) {

        stopMove();
        return;
    }

    if (pm->state == pmStateActivated) {

        // Check for active current
        if (pm->adccReading >= pm->activeThreshold) {

            // Moving
            pm->state = pmStateMoving;

        } else if (m >= ACTIVITY_WAIT_MS) {

            // No movement detected
            stopMove();
            return;
        }

    } else if (pm->state == pmStateMoving) {

        if (pm->flags.stallMode) {

            // Check for stall current
            pm->shift <<= 1;
            if (pm->filteredReading >= pm->senseThreshold) pm->shift |= 1;
            if (pm->shift == 0xFF) pm->state = pmStateAtTarget;

        } else {

            // Check for lack of current
            pm->shift <<= 1;
            if (pm->filteredReading <= pm->senseThreshold) pm->shift |= 1;
            if (pm->shift == 0x00) pm->state = pmStateAtTarget;
        }
        
        // If we have arrived at target state...
        if (pm->state == pmStateAtTarget) {
            if (appNV.flags.cutoffOnStall) stopMove();
        }
    }
}

/**
 * Activation.  Gets the PM to a confirmed known state if possible.
 * 
 * @pre pmx and pm
 */
static void activate() {

    pmState_t s = pmStateUnknown;
    pmState_t sDef = pmStateUnknown;

    // Get any defined position
    if (appNV.flags.activateToA) {
        if (appNV.flags.toAWhenUnknown) {
            sDef = pmStateA;    // To A when unknown
        } else {
            s = pmStateA;       // To A regardless
        }
    }

    // ...or use last known position
    if (s == pmStateUnknown) s = readEeprom8(pmx);

    // ...or use default position
    if (s == pmStateUnknown) s = sDef;

    if (s == pmStateA) {
        moveToA();
    } else if (s == pmStateB) {
        moveToB();
    } else {
        stopMove();
    }
}

/**
 * Main application processing.
 */
void appProcess(void) {

    if (!updateAdccValues) return;      // Set every UPDATE_MS by ISR
    updateAdccValues = false;

    // PM activation
    if (activating) {

        // Parallel activation...
        if (activating > NUM_PM) {
            activating = 0;

            for (pmx = 0; pmx < NUM_PM; ++pmx) {
                pm = &pointMotor[pmx];

                activate();
            }

        // Sequential PM initialisation...
        } else {

            pmx = activating - 1;
            pm = &pointMotor[pmx];

            if (pm->state == pmStateActivate) activate();

            // If no longer activated or moving, activation is complete
            if (pm->state < pmStateActivated) {
                if (++activating > NUM_PM) activating = 0;     // Next PM, or done
            }
        }
    }

    // Process all PMs
    for (pmx = 0; pmx < NUM_PM; ++pmx) {
        pm = &pointMotor[pmx];

        process();
    }
}

/**
 * Get an event variable value.
 * 
 * @param  eventIndex Event index.
 * @param varIndex Event variable index.
 * @return Event variable value or, if error, 0.
 */
static uint8_t getEV(uint8_t eventIndex, uint8_t varIndex) {

    uint8_t ev = 0;
    getEventVarValue(eventIndex, varIndex, &ev);
    return ev;
}

/**
 * Simple event.
 * 
 * @pre cbusMsg[] incoming message.
 * @param eventIndex Index of event.
 * @param ev1 Value of event value EV1
 * 
 * EV1  1..4 PM number
 * EV2  if 0: ON to A; OFF to B
 *      if >0: ON to B; OFF to A
 */
static void simpleEvent(uint8_t eventIndex, uint8_t ev1) {

    // Get EV2 (varIndex 1)
    uint8_t ev2 = getEV(eventIndex, 1);

    pmx = ev1 - 1;
    pm = &pointMotor[pmx];

    bool toB = (cbusMsg[0] & 0b00000001);   // OFF to B
    if (ev2 > 0) toB = !toB;                // Reversed
    if (toB) {
        moveToB();
    } else {
        moveToA();
    }
    pm->flags.events = 1;       // Generate events on completion
}

/**
 * Multi-PM event.
 * 
 * @pre cbusMsg[] incoming message.
 * @param eventIndex Index of event.
 * 
 * EV1  5
 * EV2  ON actions
 * EV3  OFF actions
 * 
 * Actions
 *      bit 0..3    0: indexed PM no movement
 *                  1: indexed PM move
 *      bit 4..7    0: default direction (ON action to A; OFF action to B)
 *                  1: reverse direction (ON action to B; OFF action to A)
 */
static void multiEvent(uint8_t eventIndex) {

    uint8_t varIndex;
    uint8_t invert;

    if (!(cbusMsg[0] & 0b00000001)) {   // ON event
        varIndex = 1;       // EV2
        invert = 0b0000;
    } else {                            // OFF event
        varIndex = 2;       // EV3
        invert = 0b1111;    // Invert reverse bits to mean the same as ON event
    }

    // Get move bits & reverse bits; initialise bitmask
    uint8_t mov = getEV(eventIndex, varIndex);
    uint8_t rev = (mov >> 4) ^ invert;
    uint8_t mask = 0b00000001;

    for (pmx = 0; pmx < NUM_PM; ++pmx) {

        // Determine required movement (if any)
        if (mov & mask) {
            pm = &pointMotor[pmx];

            if (rev & mask) {
                moveToB();
            } else {
                moveToA();
            }
            pm->flags.events = 1;       // Generate events on completion
        }

        // Next PM
        mask <<= 1;
    }
}

/**
 * Start of Day (SoD) event.
 */
static void sodEvent() {

    uint8_t en1 = 0;        // Combined ON OFF events
    uint8_t en2 = NUM_PM;   // Separate ON ON events

    for (pmx = 0; pmx < NUM_PM; ++pmx) {
        pm = &pointMotor[pmx];

        switch (pm->state) {
            case pmStateA: {
                enqueueTXMsg(txMsgACOx, ++en1, OPC_ACON);
                enqueueTXMsg(txMsgACOx, ++en2, OPC_ACON);
                ++en2;
            } break;
            case pmStateB: {
                enqueueTXMsg(txMsgACOx, ++en1, OPC_ACOF);
                ++en2;
                enqueueTXMsg(txMsgACOx, ++en2, OPC_ACON);
            } break;
            default: {
                ++en1;
                ++en2;
                ++en2;
            } break;
        }
    }
}

/**
 * Processes incoming events.
 * 
 * @pre cbusMsg[] incoming message.
 * @param eventIndex Index of event.
 */
void appProcessCbusEvent(uint8_t eventIndex) {

    // Ignore events during activation
    if (activating) return;

    // Get EV1 (varIndex 0)
    uint8_t ev1 = getEV(eventIndex, 0);

    if (ev1 >= 1 && ev1 <= NUM_PM) {
        simpleEvent(eventIndex, ev1);
    } else if (ev1 == (NUM_PM + 1)) {
        multiEvent(eventIndex);
    } else if (ev1 == 255) {
        sodEvent();
    }
}

/**
 * Sends periodic debug messages (PM ADC readings) if required.
 * 
 * @return 0 no response; >0 send response; <0 cmderr.
 * @post cbusMsg[] response.
 */
int8_t appGenerateCbusMessage(void) {

    // Debug...
    uint8_t debugPM = appNV.debugPM;
    if (debugPM > 0) {

        uint16_t t = getMillisShort();
        if (t - lastDebug >= DEBUG_MS) {
            lastDebug = t;

            cbusMsg[0] = OPC_ACDAT;

            if (debugPM <= NUM_PM) {
                pm_t* p = &pointMotor[debugPM - 1];

                cbusMsg[3] = debugPM;
                cbusMsg[4] = ((bytes16_t) p->adccReading).valueH;
                cbusMsg[5] = ((bytes16_t) p->adccReading).valueL;
                cbusMsg[6] = ((bytes16_t) p->filteredReading).valueH;
                cbusMsg[7] = ((bytes16_t) p->filteredReading).valueL;

            } else {

                cbusMsg[3] = activating;
                cbusMsg[4] = pointMotor[0].state;
                cbusMsg[5] = pointMotor[1].state;
                cbusMsg[6] = pointMotor[2].state;
                cbusMsg[7] = pointMotor[3].state;
            }

            return 1;
        }
    }

    return 0;
}

/**
 * Adds a produced event.
 * 
 * @pre pmx
 * @param en Event number.
 * @param ev1 EV1 value.
 */
void addProducedEvent(uint8_t en) {

    addEvent(cbusNodeNumber.value, en, 0, pmx + 1, true);
}

/**
 * Adds produced events when entering FLiM mode.
 */
void appEnterFlimMode(void) {

    uint8_t en1 = 0;        // Combined ON OFF events
    uint8_t en2 = NUM_PM;   // Separate ON ON events

    for (pmx = 0; pmx < NUM_PM; ++pmx) {
        addProducedEvent(++en1);    // ON at A; OFF at B
        addProducedEvent(++en2);    // ON at A
        addProducedEvent(++en2);    // ON at B
    }
}

/**
 * Removes all events when leaving FLiM mode.
 */
void appLeaveFlimMode(void) {

    removeAllEvents();
}


// <editor-fold defaultstate="expanded" desc="Interrupt service routines">


/**
 * Performs regular application operations (called every millisecond).
 */
void appTimerIsr() {

    static uint8_t count = 0;

    // Set flag every UPDATE_MS.
    if (++count == UPDATE_MS) {
        count = 0;
        updateAdccValues = true;
    }
}


// </editor-fold>

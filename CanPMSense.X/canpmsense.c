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
#include "event.h"
#include "module.h"
#include "adcc.h"


#define NUM_PM 4                // Number of point motors
#define PM_UPDATE_MS 10         // 10ms between PM updates
#define PM_DETECT_THRESHOLD 20  // 0.5mA detection threshold
#define PM_DETECTED_MS 25       // 25ms for PM to be detected
#define PM_MOVING_MS 200        // 200ms for PM to start moving
#define PM_TIMEOUT_MS 4000      // 4s for PM to complete move

#define NO_INDEX 0xFF           // No event index
#define EV1_HAPPENING 0         // Produced event
#define EV1_SIMPLE_ACTION 1     // Simple action event
#define EV1_MULTI_ACTION 2      // Multi-action event
#define EV1_SOD 255             // Stat of Day event

#define DEBUG_MS 250            // 250ms between debug messages

// Point motor state
typedef enum {
    pmStateUnknown,             // Unknown position
    pmStateA,                   // Position A
    pmStateB,                   // Position B
    pmStateActivate,            // Waiting to be activated
    pmStateActivated,           // Set to move
    pmStateDetected,            // Presence detected
    pmStateMoving,              // Confirmed moving
    pmStateAtTarget             // At target (stalled or off)
} pmState_t;

// Point motor flags
typedef union {
    struct {
        unsigned stallMode : 1; // Checking for stall
        unsigned events : 1;    // Generate produced events
    };
    uint8_t byte;
} pmFlags_t;

// Point motor data
typedef struct {
    uint8_t adcChan;            // ADC channel for feedback
    pmState_t state;            // Current state
    pmState_t startingState;    // Starting state when moving
    pmState_t targetState;      // Target state when moving
    uint16_t moveStarted;       // Time move started
    uint16_t adcThreshold;      // ADC threshold
    uint16_t adcReading;        // Last ADC reading
    uint8_t shift;              // Debounce shift register
    pmFlags_t flags;            // Flags
} pm_t;

// Point motor driver static mode
typedef enum {
    nvPMStaticOff,              // Drive turned off
    nvPMStaticOn,               // Drive stall
    nvPMStaticBrake             // Drive turned off in brake mode
} nvPMStatic_t;

// Point motor node variable data
typedef struct {
    uint16_t adcActive;         // ADC reading for PM moving
    uint16_t adcInactive;       // ADC reading for PM not moving
    nvPMStatic_t staticMode;    // Driver static mode
} nvPMData_t;

// General node variable flags
typedef union {
    struct {
        unsigned startupActivate : 1;       // Activate PMs at startup
        unsigned activateSequential : 1;    // Activate PMs in sequence (rather than in parallel)
        unsigned activateToA : 1;           // Activate PMs to A...
        unsigned toAWhenUnknown : 1;        // ... but only when unknown
    };
    uint8_t byte;
} nvFlags_t;

// Node variables
typedef struct {
    nvPMData_t pmData[NUM_PM];  // Point motor data
    nvFlags_t flags;            // General flags
    uint8_t debugPM;            // Debug
} appNV_t;


// NVs in FLASH memory
extern const appNV_t appNV __at(NODE_VAR_ADDRESS);


// PM data
pm_t pointMotor[NUM_PM] = {
    {channel_Fb1, pmStateActivate, pmStateUnknown, pmStateUnknown, 0, 0, 0, 0, {.byte = 0}},
    {channel_Fb2, pmStateActivate, pmStateUnknown, pmStateUnknown, 0, 0, 0, 0, {.byte = 0}},
    {channel_Fb3, pmStateActivate, pmStateUnknown, pmStateUnknown, 0, 0, 0, 0, {.byte = 0}},
    {channel_Fb4, pmStateActivate, pmStateUnknown, pmStateUnknown, 0, 0, 0, 0, {.byte = 0}}
};

bool updateAdccValues = false;  // Set by ISR to indicate update required
uint16_t lastDebug = 0;         // Time of last debug message
uint8_t activating = 0;         // PM activation state

uint8_t pmx;                    // PM index (0 to NUM_PM - 1)
pm_t* pm;                       // Pointer to PM data corresponding to pmx

// Produced events (happenings)
uint8_t atAHappenings[NUM_PM] = {NO_INDEX, NO_INDEX, NO_INDEX, NO_INDEX};
uint8_t atBHappenings[NUM_PM] = {NO_INDEX, NO_INDEX, NO_INDEX, NO_INDEX};


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

    // Alternate Tortoise and Cobalt point motors (... why not?)

    // Initialise application node variables
    for (pmx = 0; pmx < NUM_PM; pmx += 2) {     // PM 1 & 3
        writeFlashCached16((flashAddr_t) &appNV.pmData[pmx].adcActive, NV_TORTOISE_ACTIVE);
        writeFlashCached16((flashAddr_t) &appNV.pmData[pmx].adcInactive, NV_TORTOISE_INACTIVE);
        writeFlashCached8((flashAddr_t) &appNV.pmData[pmx].staticMode, nvPMStaticOn);
    }
    for (pmx = 1; pmx < NUM_PM; pmx += 2) {     // PM 2 & 4
        writeFlashCached16((flashAddr_t) &appNV.pmData[pmx].adcActive, NV_COBALT_ACTIVE);
        writeFlashCached16((flashAddr_t) &appNV.pmData[pmx].adcInactive, NV_COBALT_INACTIVE);
        writeFlashCached8((flashAddr_t) &appNV.pmData[pmx].staticMode, nvPMStaticOff);
    }
    // Sequential PM activation
    writeFlashCached8((flashAddr_t) &appNV.flags.byte, 0b00001111);
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

    // PM activation at startup
    nvFlags_t nvFlags = {.byte = readFlashCached8((flashAddr_t) &appNV.flags)};
    if (nvFlags.startupActivate) {
        activating = (nvFlags.activateSequential) ? 1 : NUM_PM + 1;
    }

    // Initialise happenings
    uint8_t ev;
    for (uint8_t eventIndex = 0; eventIndex < MAX_NUM_EVENTS; ++eventIndex) {

        // Get and check EV1
        uint8_t err = getEventVarValue(eventIndex, 0, &ev);
        if (err || ev != EV1_HAPPENING) continue;

        // Get EV2
        err = getEventVarValue(eventIndex, 1, &ev);
        if (err) continue;
        uint8_t ah = ev - 1;    // 0..7

        // PM index
        pmx = ah >> 1;
        if (pmx >= NUM_PM) continue;

        // Update happenings
        if (ah & 0b00000001) {
            atBHappenings[pmx] = eventIndex;
        } else {
            atAHappenings[pmx] = eventIndex;
        }
    }
}

/**
 * Starts PM movement by setting state to activated.
 * 
 * @pre pmx and pm
 * @param targetState The state that the PM is moving to.
 */
static void setActivated(pmState_t targetState) {

    pm->startingState = pm->state;
    pm->targetState = targetState;

    uint16_t nvActive = readFlashCached16((flashAddr_t) &appNV.pmData[pmx].adcActive);
    uint16_t nvInactive = readFlashCached16((flashAddr_t) &appNV.pmData[pmx].adcInactive);
    pm->adcThreshold = (nvActive + nvInactive) / 2;
    pm->flags.stallMode = (nvInactive > nvActive) ? 1 : 0;

    pm->state = pmStateActivated;
    pm->moveStarted = getMillisShort();
}

/**
 * Sets PM driver off.
 * 
 * @pre pmx
 */
static void driverOff() {

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
}

/**
 * Sets PM driver to 'A'.
 * 
 * @pre pmx
 */
static void driveToA() {

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
 * Sets PM driver to 'B'.
 * 
 * @pre pmx
 */
static void driveToB() {

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
 * Sets PM driver brake.
 * 
 * @pre pmx
 */
static void driverBrake() {

    switch (pmx) {
        case 0: {
            EnA1_SetHigh();
            EnB1_SetHigh();
        } break;
        case 1: {
            EnA2_SetHigh();
            EnB2_SetHigh();
        } break;
        case 2: {
            EnA3_SetHigh();
            EnB3_SetHigh();
        } break;
        case 3: {
            EnA4_SetHigh();
            EnB4_SetHigh();
        } break;
    }
}

/**
 * Initiates a PM move to 'A'.
 * 
 * @pre pmx and pm
 */
static void moveToA() {

    setActivated(pmStateA);

    // Turn on driver
    driveToA();
}

/**
 * Initiates a PM move to 'B'.
 * 
 * @pre pmx and pm
 */
static void moveToB() {

    setActivated(pmStateB);

    // Turn on driver
    driveToB();
}

/**
 * Stops PM movement.
 * 
 * @pre pmx and pm
 * @param atTarget True if at, or assumed to be at, target
 * @param events True to allow produce events
 * 
 * @note Final state is written to EEPROM.
 */
static void stopMove(bool atTarget, bool events) {

    pmState_t finalState;
    if (atTarget) {

        finalState = pm->targetState;

        nvPMStatic_t nvStaticMode = readFlashCached8((flashAddr_t) &appNV.pmData[pmx].staticMode);
        if (nvStaticMode == nvPMStaticOff) {
            // Turn off driver
            driverOff();
        } else if (nvStaticMode == nvPMStaticBrake) {
            // Set driver brake
            driverBrake();
        } else {
            // Do nothing; leave driver running (stalled)
        }

    } else {

        finalState = pm->startingState;

        // Turn off driver
        driverOff();
    }
    pm->state = finalState;

    // Save this as the last known state
    pmState_t s = readEeprom8(pmx);
    if (s != finalState) writeEeprom8(pmx, finalState);

    // Generate ON events, having arrived
    if (events && pm->flags.events) {
        uint8_t eventIndex = (finalState == pmStateA) ? atAHappenings[pmx]
                : (finalState == pmStateB) ? atBHappenings[pmx]
                : NO_INDEX;
        if (eventIndex != NO_INDEX) produceEvent(true, eventIndex);
    }
}

/**
 * Reads the ADC data for a PM.  Stops the PM if inactive, when target reached,
 * or when timed out.
 * 
 * @pre pmx and pm
 */
static void process() {

    // Done if PM static
    if (pm->state < pmStateActivated) return;

    // Get channel ADC reading
    pm->adcReading = adccRead(pm->adcChan);

    // Get time since started moving
    uint16_t m = getMillisShort() - pm->moveStarted;

    if (pm->state == pmStateActivated) {

        // Check for active current
        if (pm->adcReading >= PM_DETECT_THRESHOLD) {

            // Detected
            pm->state = pmStateDetected;
            pm->shift = 0;

        // Give up waiting?
        } else if (m > PM_DETECTED_MS) {

            stopMove(false, false);             // Never got started
            return;
        }

    } else if (pm->state == pmStateDetected) {

        // Debounce threshold pass
        pm->shift <<= 1;
        if (pm->flags.stallMode) {
            if (pm->adcReading < pm->adcThreshold) pm->shift |= 1;
        } else {
            if (pm->adcReading >= pm->adcThreshold) pm->shift |= 1;
        }
        if (pm->shift == 0xFF) {

            // Moving
            pm->state = pmStateMoving;
            pm->shift = 0;

            // Generate OFF events
            if (pm->flags.events) {
                uint8_t eventIndex = (pm->startingState == pmStateA) ? atAHappenings[pmx]
                        : (pm->startingState == pmStateB) ? atBHappenings[pmx]
                        : NO_INDEX;
                if (eventIndex != NO_INDEX) produceEvent(false, eventIndex);
            }

        // Give up waiting?
        } else if (m > PM_MOVING_MS) {

            stopMove(true, false);              // Probably already at target
            return;
        }

    } else if (pm->state == pmStateMoving) {

        // Debounce threshold pass
        pm->shift <<= 1;
        if (pm->flags.stallMode) {
            if (pm->adcReading >= pm->adcThreshold) pm->shift |= 1;
        } else {
            if (pm->adcReading < pm->adcThreshold) pm->shift |= 1;
        }
        if (pm->shift == 0xFF) {

            stopMove(true, true);               // Confirmed reached target
            return;

        // Give up waiting?
        } else if (m > PM_TIMEOUT_MS) {

            stopMove(true, false);              // Hopefully reached target
            return;
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
    nvFlags_t nvFlags = {.byte = readFlashCached8((flashAddr_t) &appNV.flags)};
    if (nvFlags.activateToA) {
        if (nvFlags.toAWhenUnknown) {
            sDef = pmStateA;    // To A when unknown
        } else {
            s = pmStateA;       // To A regardless
        }
    }

    if (s == pmStateUnknown) s = readEeprom8(pmx);  // ...or use last known position
    if (s == pmStateUnknown) s = sDef;              // ...or use default position
    pm->state = s;

    if (s == pmStateA) {
        moveToA();
    } else if (s == pmStateB) {
        moveToB();
    }
    pm->flags.events = 0;   // No events
}

/**
 * Main application processing.
 */
void appProcess() {

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
 * Simple action event.
 * 
 * @param eventIndex Index of event.
 * @param onEvent True if ON; false if OFF
 * 
 * EV1  EV1_SIMPLE_ACTION
 * EV2  Action/Happening number [1..8]
 * EV3  Ignored
 */
static void simpleActionEvent(uint8_t eventIndex, bool onEvent) {

    // EV2 (varIndex 1) is Action/Happening number [1..8]
    uint8_t ah = getEV(eventIndex, 1) - 1;  // 0..7

    // PM index
    pmx = ah >> 1;
    if (pmx >= NUM_PM) return;
    pm = &pointMotor[pmx];

    bool toB = (ah & 0b00000001);
    if (!onEvent) toB = !toB;   // OFF event reverses target
    if (toB) {
        moveToB();
    } else {
        moveToA();
    }
    pm->flags.events = 1;       // Generate events on completion
}

/**
 * Multi-action event.
 * 
 * @param eventIndex Index of event.
 * @param onEvent True if ON; false if OFF
 * 
 * EV1  EV1_MULTI_ACTION
 * EV2  ON actions
 * EV3  OFF actions
 * 
 * Actions
 *      bit 0..3    0: indexed PM no movement
 *                  1: indexed PM move
 *      bit 4..7    0: default direction (ON action to A; OFF action to B)
 *                  1: reverse direction (ON action to B; OFF action to A)
 */
static void multiActionEvent(uint8_t eventIndex, bool onEvent) {

    uint8_t varIndex;
    uint8_t invert;

    if (onEvent) {
        // ON event
        varIndex = 1;       // EV2
        invert = 0b0000;
    } else {
        // OFF event
        varIndex = 2;       // EV3
        invert = 0b1111;    // Invert reverse bits to mean the same as ON event
    }

    // Get move bits & reverse bits; initialise bit mask
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
 * 
 * @note ON events only
 * 
 * EV1  EV1_SOD
 * EV2  Ignored
 * EV3  Ignored
 */
static void sodEvent() {

    for (pmx = 0; pmx < NUM_PM; ++pmx) {
        pm = &pointMotor[pmx];

        uint8_t eventIndex = (pm->state == pmStateA) ?  atAHappenings[pmx]
                : (pm->state == pmStateB) ?  atBHappenings[pmx]
                : NO_INDEX;
        if (eventIndex != NO_INDEX) produceEvent(true, eventIndex);
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

    // Determine if ON or OFF event
    bool onEvent = !(cbusMsg[0] & 0b00000001);

    if (ev1 == EV1_SIMPLE_ACTION) {
        simpleActionEvent(eventIndex, onEvent);
    } else if (ev1 == EV1_MULTI_ACTION) {
        multiActionEvent(eventIndex, onEvent);
    } else if (ev1 == EV1_SOD) {
        if (onEvent) sodEvent();
    }
}

/**
 * Sends periodic debug messages if required.
 * 
 * @return 0 no response; >0 send response; <0 cmderr.
 * @post cbusMsg[] response.
 */
int8_t appGenerateCbusMessage() {

    // Debug...
    uint8_t nvDebugPM = readFlashCached8((flashAddr_t) &appNV.debugPM);
    if (nvDebugPM > 0) {

        uint16_t t = getMillisShort();
        if (t - lastDebug >= DEBUG_MS) {
            lastDebug = t;

            cbusMsg[0] = OPC_ACDAT;

            if (nvDebugPM <= NUM_PM) {
                pm_t* p = &pointMotor[nvDebugPM - 1];

                cbusMsg[3] = p->state;
                cbusMsg[4] = ((bytes16_t) p->adcThreshold).valueH;
                cbusMsg[5] = ((bytes16_t) p->adcThreshold).valueL;
                cbusMsg[6] = ((bytes16_t) p->adcReading).valueH;
                cbusMsg[7] = ((bytes16_t) p->adcReading).valueL;

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
 * Called when entering FLiM mode.
 */
//void appEnterFlimMode() {
//}

/**
 * Called when leaving FLiM mode.
 */
//void appLeaveFlimMode() {
//}

/**
 * Called when a node variable change is requested.
 * 
 * @param varIndex Index of node variable.
 * @param curValue Current NV value.
 * @param newValue New NV value.
 * @return false to reject change (e.g invalid value).
 */
//bool appValidateNodeVar(uint8_t varIndex, uint8_t curValue, uint8_t newValue) {
//
//    return true;
//}

/**
 * Called when a node variable change has been made.
 * 
 * @param varIndex Index of node variable.
 * @param oldValue Old NV value.
 * @param curValue Current NV value.
 */
//void appNodeVarChanged(uint8_t varIndex, uint8_t oldValue, uint8_t curValue) {
//}

/**
 * Called when an event variable change is requested.
 * 
 * @param eventIndex Index of event.
 * @param varIndex Index of event variable.
 * @param curValue Current EV value.
 * @param newValue New EV value.
 * @return false to reject change (e.g invalid value).
 */
//bool appValidateEventVar(uint8_t eventIndex, uint8_t varIndex, uint8_t curValue, uint8_t newValue) {
//
//    return true;
//}

/**
 * Called when an event variable change has been made.
 * 
 * @param eventIndex Index of event.
 * @param varIndex Index of event variable.
 * @param oldValue Old EV value.
 * @param curValue Current EV value.
 */
void appEventVarChanged(uint8_t eventIndex, uint8_t varIndex, uint8_t oldValue, uint8_t curValue) {

    // Only interested in EV2 (varIndex 1),
    // and then only when EV1 (varIndex 0) is EV1_HAPPENING
    if (varIndex != 1 || getEV(eventIndex, 0) != EV1_HAPPENING) return;

    // EV2 curValue is Action/Happening number [1..8]
    uint8_t ah = curValue - 1;  // [0..7]

    // PM index
    pmx = ah >> 1;
    if (pmx >= NUM_PM) return;

    // Update happenings
    if (ah & 0b00000001) {
        atBHappenings[pmx] = eventIndex;
    } else {
        atAHappenings[pmx] = eventIndex;
    }
}

/**
 * Called when an event is removed.
 * 
 * @param eventIndex Index of event, or NO_INDEX for all events.
 */
void appEventRemoved(uint8_t eventIndex) {

    if (eventIndex  == NO_INDEX) {
        for (pmx = 0; pmx < NUM_PM; ++pmx) {
            atAHappenings[pmx] = NO_INDEX;
            atBHappenings[pmx] = NO_INDEX;
        }
    } else {
        for (pmx = 0; pmx < NUM_PM; ++pmx) {
            if (atAHappenings[pmx] == eventIndex) atAHappenings[pmx] = NO_INDEX;
            if (atBHappenings[pmx] == eventIndex) atBHappenings[pmx] = NO_INDEX;
        }
    }
}


// <editor-fold defaultstate="expanded" desc="Interrupt service routines">


/**
 * Performs regular application operations (called every millisecond).
 */
void appTimerIsr() {

    static uint8_t count = 0;

    // Set flag every UPDATE_MS.
    if (++count == PM_UPDATE_MS) {
        count = 0;
        updateAdccValues = true;
    }
}


// </editor-fold>

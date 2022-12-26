# CanPMSense Firmware
**CBUS CAN point motor driver with current sense**

This project provides the firmware for a Microchip PIC18F based CBUS module that drives up to four
slow-motion (motor, rather than solenoid) model railway point motors.

CanPMSense is based on version 2c of the [CbusLibXC8](https://github.com/Syspixie/CbusLibXC8) library.
It is written for the Microchip XC8 compiler, and targets PIC18F25K83 processors.

*As a design exercise, CanPMSense shows how a project can be built on the CbusLibXC8 library.
The project provides four additional files:* canpmsense.c *and* canpmsense.h *for the majority of the
application code, plus* adcc.c *and* adcc.h *for the ADC peripheral interface.  Only two of the library
files require modification:* module.c *to uncomment calls to the application routines, and*
moduledefs.h *to set the major application parameters.*

**********

# Revision History

### Version 1a
Initial version.

**********

# Hardware Description

[Circuit diagram](Module/CanPMSense.pdf).

The module follows [MERG CBUS](https://merg.org.uk/resources/cbus) CAN module convention, and is
powered by 12V DC from the CBUS connector. The point motors have a separate 12V DC supply.

The module has been tested with DCCconcepts Cobalt Classic Ω Analog point motors.  These work best
on about 9V, so the 12V motor supply is stepped down with 9V regulators for each motor.  At 9V the
point motors draw about 30mA whilst moving, and 60mA when stalled at the end position.

Each motor has an H-bridge driver, and a current sense amplifier.

For each point motor, the processor has:
- Two outputs, each driving the motor in a different direction; when both outputs are off,
the motor stops.
- An analog input connected to the current sense amplifier, which provides an ADC reading
proportional to the current drawn by the motor.

**********

# Basic Firmware Functionality

- A state model monitors the position and operation of each point motor.
- Current sense feedback is used to determine whether the point motor is moving,
has stopped moving, or is even connected at all.
- When a point motor stops, the program cuts off the drive current. For point motors that
stall with a higher current than when moving, this significantly reduces the power
requirements.
- On start-up, each point motor, in sequence or in parallel, can be driven to its
last known position (as stored in EEPROM).
- Point motors can be instructed to move individually, by either a single event
with ON and OFF variants, or by paired events (one for each position).  They can also
be instructed in combination with a single event - e.g. PM1 to A, PM2 to B and PM4 to A.

### Notes

- The current sense ADC inputs are filtered with an exponential (low-pass) filter,
followed by a glitch filter, to minimise the effects of variations in motor current.
- Each point motor is defined with 'active' and 'stall' ADC readings.  These are
used to calculate the thresholds used to determine the motor's movements.  They
allow the use of point motors that, upon reaching the target position, stall with
a high current, as well as those that turn themselves off.

**********

# CBUS Functionality
 
- Standard interface with SLiM and FLiM LEDs and a mode button.
- Designed for FLiM mode, configured using the FLiM Configuration Utility (FCU).
- Responds to single and paired events for each point motor, and also to
events controlling multiple point motors.
- Produces single and paired events for each point motor when the target position
is reached.
- Responds to Start of Day (SoD) event.
- Node variables control:
  - ADC settings for each point motor.
  - Start-up behaviour.
  - Generation of debug data messages.
- Bootloader support.

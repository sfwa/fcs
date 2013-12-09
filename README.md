# fcs

Flight control system firmware for the SFWA CPU board DSP (TMS320C6657)

## Building

Requires `cmake` version 2.8.7 or higher.

Create a build directory outside the source tree, then use cmake to generate
the makefile.

**Note: currently only tests can be built**

`mkdir fcs_build`

`cd fcs_build`

`cmake /path/to/fcs/test`

Now, build the library using the `make` command.


## Testing

The `googletest` library is used for unit testing.

After creating the build directory, `make check` will automatically download
and build googletest, build the unit tests and then run them.

To build the unit tests without running them, use `make unittest`. The unit
tests can then manually be run (with more detailed reporting) by running
`./unittest` in the build directory.


## Overview

Main bootloader executes AHRS on core 0, and NMPC on core 1.

AHRS task (boots first, only executes on core 0):

* SPI bootloader finishes -- BOOTCOMPLETE asserted, which switches CPLD to
  DSP booted state (no DSP code needed)
* Read calibration/configuration data from I2C flash
* Start NMPC task on core 1
* Initialize UKF with default values from I2C flash/L2 SRAM (depending on
  startup mode)
* Load sensor calibration data from I2C flash/L2 SRAM (depending on startup
  mode)
* Initialize I/O board comms -- 2x internal UARTs with EDMA to/from L2 then a
  copy from L2 to L1 each frame.
* Initialize CPU logging, update and control comms -- external UART with EDMA
  to/from L2 only, logged packets transferred from L1 to L2 at end of each
  frame
* Initialize IPC structures -- single mutex-protected state structure, plus
  mutex-protected message queue (?) for control packets to and from NMPC
  system.
* Enter infinite loop:
    * Store loop start TSCH/TSCL
    * DMA last I/O board packets from UARTs
    * Parse and validate packets
    * Apply sensor calibration
    * Convert sensor readings to metric units
    * Transform and sum redundant sensor readings into a single virtual sensor
      reading
    * Exclude/increase covariance for very implausible sensor readings
    * Set GPS position/velocity covariance based on fix type
    * Set UKF sensor values
    * Run UKF iteration
    * Get UKF output state
    * Acquire global state mutex, copy from L1 state to global state, release
      global state mutex
    * Copy current state and raw I/O board packets to L2 CPU UART buffer,
      initiate DMA to external UART
    * Acquire global control value mutex, copy from global control value to L1
      control value, release global control value mutex
    * Copy control values to L2 I/O board UART buffers, initiate DMA to
      internal UARTs
    * Acquire global I/O board command mutex, copy from global I/O board
      command to L2 I/O board UART buffers, release global I/O board command
      mutex, initiate DMA transfer to internal UARTs
    * Acquire global DSP command mutex, copy from L2 DSP command buffer to
      global DSP command buffer, release global DSP command mutex
    * Check difference between current TSCH/TSCL and loop start TSCH/TSCL;
      if > 1M enter reset

IPC globals:

* AHRS state structure
* DSP command buffer (CPU -> DSP, read by AHRS, handled by AHRS or NMPC)
* NMPC state structure
* System state structure

Communications:

* CPU->DSP: waypoint update, AHRS or NMPC parameter update
* DSP->CPU: I/O board packet mirror, nav state estimate, system state
* I/O board->DSP: sensor update
* DSP->I/O board: control update

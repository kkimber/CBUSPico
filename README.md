# CBUS library for RaspberryPi PICO

A CBUS Library for the Raspberry PICO, based on the PICO-SDK.

Based on existing Arduino libraries with significant refactoring and clean-up.

   * https://github.com/MERG-DEV/CBUSLED
   * https://github.com/MERG-DEV/CBUSSWITCH
   * https://github.com/MERG-DEV/CBUSConfig
   * https://github.com/MERG-DEV/CBUS

and the PICO CAN2040 libraries:

   * https://github.com/obdevel/ACAN2040
   * https://github.com/obdevel/CBUSACAN2040

## Project Status

* Current status has the example module compiling and it appears functional.
* Use of the external I2C FRAM or EEPROM is not yet implemented, and is largely untested.
* Module based on using the PICO QPSI flash should be functional, but has had very limited testing.

**NOTE:** This is a work in progress and all API's and interfaces are therefore subject to change.

**Build status**

Github is configured to perform continuous integration builds on every check-in, the latest binaries can be downloaded directly from "Github Actions" without needing to download and build the sources. 

<img alt="git build action status"
   src="https://github.com/kkimber/CBUSPico/actions/workflows/cmake_rpi_pico.yml/badge.svg"/>

**Static code analysis status**

The code has been analyzed with Coverity for errors.

There are currently zero defects identified in the CBUS library code itself, however there are 22 defects identified in the PICO-SDK code that the CBUS library uses.

<a href="https://scan.coverity.com/projects/kkimber-cbuspico">
  <img alt="Coverity Scan Build Status"
       src="https://scan.coverity.com/projects/29566/badge.svg"/>
</a>

---

## Building

CANPico requires the RPi Pico-SDK to be installed and configuration files are provided to build with CMake using the standard setup for a pico-sdk based application.

\todo add more build details

## Documentation 

Full source documentation is provided to be generated with Doxygen:

https://www.doxygen.nl/

To build the documentation make sure you have Doxygen and GraphViz installed and then build the CMake target "doxygen".

The documentation will be created in the build/html folder.  Open the file index.html in a browser to view the docs.

## Hardware setup

The CBUSPico library and examples can be run on any Pico board as long as it is connected to a suitable CAN transceiver.  The code uses the PIO based CAN2040 soft CAN controller, so no external CAN controller is needed. Simply connect the transceiver to the CAN Tx /Rx pins as indicated below.

The default pin mapping used by CANPico is follows.

| Pico Pin | Function      |/| Pico Pin | Function      |
|----------|---------------|-|----------|---------------|
| 1        | GP0           | | 40       | VBUS          |
| 2        | GP0           | | 39       | VSYS          |
| 3        | GND           | | 38       | GND           |
| 4        | GP2           | | 37       | 3V3_EN        |
| 5        | GP3           | | 36       | 3V3           |
| 6        | GP4           | | 35       | ADC_VREF      |
| 7        | GP5           | | 34       | GP28          |
| 8        | GND           | | 33       | GND           |
| 9        | GP6           | | 32       | GP27          |
| 10       | GP7           | | 31       | GP26          |
| 11       | Red LED       | | 30       | RUN           |
| 12       | Green LED     | | 29       | GP22          |
| 13       | GND           | | 28       | GND           |
| 14       | GP10          | | 27       | GP21          |
| 15       | GP11          | | 26       | GP20          |
| 16       | GP12          | | 25       | GP19          |
| 17       | CAN Tx        | | 24       | GP18          |
| 18       | GND           | | 23       | GND           |
| 19       | CAN Rx        | | 22       | GP17          |
| 20       | Yellow LED    | | 21       | GP16          |


## Storage memory layout

Storage for module global configuration variables and node variables can optionally be in an external I2C FRAM or EEPROM, or can be located in a the external QSPI flash on the PICO board.

### External FRAM / EEPROM or Flash

If the external FRAM or EEPROM is used the variables are located offset into the FRAM or EEPROM as per "Address" below.

If QSPI flash is used, the last sector of flash is used.  The size of flash on the PICO is defined by PICO_FLASH_SIZE_BYTES, nominally 2MiB in size for current models.  Flash sector size is defined as FLASH_SECTOR_SIZE, which is the minimum size that can be erased, so with a flash size of 2MiB the data will be located at 0x1FF000, with variables located within that sector as per "Address" below.

| Address #           | Length                           | Usage           | EEPROM/FRAM   |
|---------------------|----------------------------------|-----------------|---------------|
| 0x00                | 1                                | FLiM Mode       | Optional      |
| 0x01                | 1                                | CANID           | Optional      |
| 0x02 - 0x03         | 2                                | Node Number     | Optional      |
| 0x04                | 1                                | [spare]         | Optional      |
| 0x05                | 1                                | Reset flag      | Optional      |
| 0x06 - 0x0A         | 4                                | [spare]         | Optional      |
| EE_NVS_START        | EE_NUM_NVS                       | Node Variables  | Optional      |
| EE_EVENTS_START     | EE_MAX_EVENTS * (EE_NUM_EVS + 4) | Events          | No            |

### Flash

| Address #           | Length                           | Usage                      |
|---------------------|----------------------------------|----------------------------|
| 0x000000 - 0x1FEFFF | 0x1FF000                         | Application Image          |
| 0x1FF000 - 0x1FFFFF | 0x1000                           | Flash Storage              |

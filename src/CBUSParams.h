/*
   MERG CBUS Module Library - RasberryPi Pico SDK port
   Copyright (c) Kevin Kimber 2023

   Based on work by Duncan Greenwood
   Copyright (C) Duncan Greenwood 2017 (duncan_greenwood@hotmail.com)

   This work is licensed under the:
      Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
   To view a copy of this license, visit:
      http://creativecommons.org/licenses/by-nc-sa/4.0/
   or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

   License summary:
    You are free to:
      Share, copy and redistribute the material in any medium or format
      Adapt, remix, transform, and build upon the material

    The licensor cannot revoke these freedoms as long as you follow the license terms.

    Attribution : You must give appropriate credit, provide a link to the license,
                  and indicate if changes were made. You may do so in any reasonable manner,
                  but not in any way that suggests the licensor endorses you or your use.

    NonCommercial : You may not use the material for commercial purposes. **(see note below)

    ShareAlike : If you remix, transform, or build upon the material, you must distribute
                 your contributions under the same license as the original.

    No additional restrictions : You may not apply legal terms or technological measures that
                                 legally restrict others from doing anything the license permits.

   ** For commercial use, please contact the original copyright holder(s) to agree licensing terms

    This software is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE

*/

#pragma once

#include "CBUSConfig.h"

#include <cstdint>

constexpr uint8_t NUM_PARAMS = 20; ///< Number of parameters in the CBUS parameter block

constexpr uint8_t NO_PARAMS = 0;  ///< byte 0 Number of params - should be 20 to exclude
constexpr uint8_t MANUFR_ID = 1;  ///< byte 1 manufacturer
constexpr uint8_t MINOR_VER = 2;  ///< byte 2 module minor version (character)
constexpr uint8_t MODULE_ID = 3;  ///< byte 3 module ID
constexpr uint8_t NO_EVENTS = 4;  ///< byte 4 maximum number of events
constexpr uint8_t EVS_PR_EV = 5;  ///< byte 5 number of event variables per event
constexpr uint8_t MXNUM_NVS = 6;  ///< byte 6 number of node variables
constexpr uint8_t MAJOR_VER = 7;  ///< byte 7 module major version
constexpr uint8_t MOD_FLAGS = 8;  ///< byte 8 Flags - this identifies the module class in the ls 2 bits - encoded,
                                  ///  No Events = 0, Consumer = 1, Producer = 2, Combi = 3.
                                  ///  It also includes the FLiM bit (bit 2) and the Bootable bit (bit 3)
constexpr uint8_t PROCSR_ID = 9;  ///< byte 9 Processor Id - defines the processor, e.g. 2480, 25K80 the firmware was built for.
                                  ///  Set to zero for non-PIC processors
constexpr uint8_t IF_PROTOC = 10; ///< byte 10 Interface protocol - the network type that the module uses,
                                  ///  currently either CAN (1) or Ethernet (2)
constexpr uint8_t LOAD_ADDR = 11; ///< bytes 11-14 The load address for the new code, not used for non PIC processor, set to zero
constexpr uint8_t MANU_PROC = 15; ///< bytes 15-18 Processor manufacturer code, not used for non PIC processor, set to zero
constexpr uint8_t MANU_CODE = 19; ///< byte 19 Manufacturer code – this parameter identifies the manufacturer
constexpr uint8_t BETA_FLAG = 20; ///< byte 20 Beta release code – a non-zero value specifies the beta release version, zero indicates a normal release

//
/// A class to manage setting and storage of CBUS module parameters
//

class CBUSParams
{
public:
   explicit CBUSParams(CBUSConfig const &config);
   void setVersion(char major, char minor, char beta);
   void setModuleId(uint8_t id);
   void setFlags(uint8_t flags);
   void setProcessor(uint8_t manufacturer, uint8_t id, char const *name);
   uint8_t *getParams(void);

private:
   // Initializes processor specific parameters
   static void initProcessorParams(void);

   // Memory for the params is allocated on global memory and handed over to CBUS.setParams().
   static uint8_t m_params[NUM_PARAMS + 1];
};
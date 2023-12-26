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

#include "CBUSParams.h"
#include "cbusdefs.h"

#include <string.h>

/// Initialize global CBUSParams
uint8_t CBUSParams::m_params[21] = {};

///
/// @brief Construct a new CBUSParams::CBUSParams object
/// 
/// @param config Reference to a CBUSConfig object, from which some configuration parameters will be extracted
///
CBUSParams::CBUSParams(CBUSConfig const &config)
{
   m_params[0] = 20;                   // byte 0 num params = 20
   m_params[1] = MANU_MERG;            // byte 1 manf = MERG, 165
   m_params[4] = config.EE_MAX_EVENTS; // byte 4 num events
   m_params[5] = config.EE_NUM_EVS;    // byte 5 num evs per event
   m_params[6] = config.EE_NUM_NVS;    // byte 6 num NVs
   m_params[10] = PB_CAN;              // byte 10 CAN implementation of CBUS
   m_params[11] = 0x00;
   m_params[12] = 0x00;
   m_params[13] = 0x00;
   m_params[14] = 0x00;
   initProcessorParams();
}

///
/// @brief Set the version information of the module
/// 
/// @param major Major version
/// @param minor Minor version
/// @param beta  Beta version
///
void CBUSParams::setVersion(char major, char minor, char beta)
{
   m_params[7] = major; // byte  7 code major version
   m_params[2] = minor; // byte  2 code minor version
   m_params[20] = beta; // byte 20 code beta version
}

///
/// @brief Set the module ID
/// 
/// @param id ID to set for the module
///
void CBUSParams::setModuleId(uint8_t id)
{
   m_params[3] = id; // byte 3 module id
}

///
/// @brief Set module flags
/// 
/// @param flags Flags to set on the module
///
void CBUSParams::setFlags(uint8_t flags)
{
   m_params[8] = flags; // byte 8 flags - FLiM, consumer/producer
}

///
/// @brief Set the processor information for the module,
///        Optional: use this to override processor info that is set by default.
/// 
/// @param manufacturer Processor Manufacturer ID
/// @param id Processor ID
/// @param name Processor name - expected to be four bytes!!
///
void CBUSParams::setProcessor(uint8_t manufacturer, uint8_t id, char const *name)
{
   m_params[9] = id;               // byte  9 processor id
   m_params[19] = manufacturer;    // byte 19 processor manufacturer
   memcpy(m_params + 15, name, 4); // byte 15-18 processor version
}

///
/// @brief Get pointer to the CBUSParam array
/// 
/// @return uint8_t* Pointer to the CBUSParam array
///
uint8_t *CBUSParams::getParams()
{
   return m_params;
}

///
/// @brief Initialize processor information for the module
/// 
///
void CBUSParams::initProcessorParams()
{
   m_params[9] = 50;        // byte  9 processor id
   m_params[19] = CPUM_ARM; // byte 19 processor manufacturer

   m_params[15] = '?'; // byte 15-18 processor version
   m_params[16] = '?';
   m_params[17] = '?';
   m_params[18] = '?';
}
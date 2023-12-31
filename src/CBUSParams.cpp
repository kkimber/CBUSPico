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

#include <cstring>

/// Initialize global CBUSParams
uint8_t CBUSParams::m_params[NUM_PARAMS + 1] = {}; ///< Initialize parameter block

///
/// @brief Construct a new CBUSParams::CBUSParams object
///
/// @param config Reference to a CBUSConfig object, from which some configuration parameters will be extracted
///
CBUSParams::CBUSParams(CBUSConfig const &config)
{
   m_params[NO_PARAMS] = NUM_PARAMS;
   m_params[MANUFR_ID] = MANU_MERG;
   m_params[NO_EVENTS] = config.EE_MAX_EVENTS;
   m_params[EVS_PR_EV] = config.EE_NUM_EVS;
   m_params[MXNUM_NVS] = config.EE_NUM_NVS;
   m_params[IF_PROTOC] = PB_CAN;
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
   m_params[MAJOR_VER] = major;
   m_params[MINOR_VER] = minor;
   m_params[BETA_FLAG] = beta;
}

///
/// @brief Set the module ID
///
/// @param id ID to set for the module
///
void CBUSParams::setModuleId(uint8_t id)
{
   m_params[MODULE_ID] = id;
}

///
/// @brief Set module flags
///
/// @param flags Flags to set on the module
///
void CBUSParams::setFlags(uint8_t flags)
{
   m_params[MOD_FLAGS] = flags;
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
   m_params[PROCSR_ID] = id;
   m_params[MANU_CODE] = manufacturer;
   memcpy(&m_params[MANU_PROC], name, 4);
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
   /// @todo there is no defined processor ID code for the RPxxxx mircocontrollers
   m_params[PROCSR_ID] = 50;       // Processor ID
   m_params[MANU_CODE] = CPUM_ARM; // Processor manufacturer

   m_params[MANU_PROC + 0] = '2'; // Set processor version to 2040
   m_params[MANU_PROC + 1] = '0';
   m_params[MANU_PROC + 2] = '4';
   m_params[MANU_PROC + 3] = '0';
}
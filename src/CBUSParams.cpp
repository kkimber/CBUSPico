/*
   CBUS Module Library - RasberryPi Pico SDK port
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
#include "CBUSConfig.h"
#include "cbusdefs.h"

#include <cstring>

/// Initialize global CBUSParams
cbusparam_t CBUSParams::m_params = {}; ///< Initialize parameter block

///
/// @brief Construct a new CBUSParams::CBUSParams object
///
/// @param config Reference to a CBUSConfig object, from which some configuration parameters will be extracted
///
CBUSParams::CBUSParams(CBUSConfig const &config)
{
   m_params.param[PAR_NPARAMS] = NUM_PARAMS;
   m_params.param[PAR_MANU] = MANU_MERG;
   m_params.param[PAR_EVTNUM] = config.EE_MAX_EVENTS;
   m_params.param[PAR_EVNUM] = config.EE_NUM_EVS;
   m_params.param[PAR_NVNUM] = config.EE_NUM_NVS;
   m_params.param[PAR_BUSTYPE] = PB_CAN;
   initProcessorParams();
}

///
/// @brief Set the version information of the module
///
/// @param major Major version
/// @param minor Minor version
/// @param beta  Beta version
///
void CBUSParams::setVersion(const uint8_t major, const char minor, const uint8_t beta)
{
   m_params.param[PAR_MAJVER] = major;
   m_params.param[PAR_MINVER] = minor;
   m_params.param[PAR_BETA] = beta;
}

///
/// @brief Set the module ID
///
/// @param id ID to set for the module
///
void CBUSParams::setModuleId(const uint8_t id)
{
   m_params.param[PAR_MTYP] = id;
}

///
/// @brief Set module flags
///
/// @param flags Flags to set on the module
///
void CBUSParams::setFlags(const uint8_t flags)
{
   m_params.param[PAR_FLAGS] = flags;
}

///
/// @brief Get pointer to the CBUS Params
///
/// @return Pointer to the CBUS Params
///
cbusparam_t* CBUSParams::getParams() const
{
   return &m_params;
}

///
/// @brief Initialize processor information for the module
///
///
void CBUSParams::initProcessorParams()
{
   /// @todo there is no defined processor ID code for the RPxxxx mircocontrollers
   m_params.param[PAR_CPUID] = 50;       // Processor ID
   m_params.param[PAR_CPUMAN] = CPUM_ARM; // Processor manufacturer

   m_params.param[PAR_CPUMID + 0] = '2'; // Set processor version to 2040
   m_params.param[PAR_CPUMID + 1] = '0';
   m_params.param[PAR_CPUMID + 2] = '4';
   m_params.param[PAR_CPUMID + 3] = '0';
}
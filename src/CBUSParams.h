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

#pragma once

#include <cstdint>

// Forward declarations
class CBUSConfig;

constexpr uint8_t NUM_PARAMS = 20; ///< Number of parameters in the CBUS parameter blockconstexpr uint8_t IDX_NO_PARAMS = 0;  ///< byte 0 Number of params - should be 20 to exclude
constexpr uint8_t PAR_NPARAMS = 0;  ///< byte 0 Number of params - should be 20 to exclude
/// Type for holding configuration parameters, size is plus one to include param zero, which is number of avaialble parameters
typedef struct cbusparam_t { uint8_t param[NUM_PARAMS + 1]; } cbusparam_t;

///
/// @brief A class to manage setting and storage of CBUS module parameters
///
/// @details Each CBUS Module maintains a number of parameters that are used primarily by CBUS configuration tools
///          to obtain information on the firmware running in the module.  The parameters define things like software
///          versions, capabilies of the module, i.e. if it is a Producer, Consumer or "Combi" module.  There are nominally
///          twenty defined parameters, although CBUS allows more to be defined in future.  During initial configuration of a
///          module, the first seven parameters can be read via Request Node Parameters (RQNP).  After initial configuration
///          any node parameter can be read via Request read of a node parameter by index (RQNPN).  Node parameters are 
///          numbered from one to twenty, however performing a RQNPN for parameter zero will return the number of available
///          parameters in the module.  Indexes for the parameters are in cbusdefs.h as PAR_xxx, e.g. PAR_MTYP.
///          The CBUS definition header, cbusdefs.h, includes defines values for a number of the parameters, e.g. manufacturer
///          codes, module ID's etc. Module developers should ensure they do not conflict or reuse ID's already defined by CBUS.

class CBUSParams
{
public:
   explicit CBUSParams(CBUSConfig const &config);
   void setVersion(const uint8_t major, const char minor, const uint8_t beta);
   void setModuleId(const uint8_t id);
   void setFlags(const uint8_t flags);
   cbusparam_t* getParams(void) const;

private:
   // Initializes processor specific parameters
   static void initProcessorParams(void);

   // Memory for the params is allocated on global memory and handed over to CBUS.setParams().
   static cbusparam_t m_params;
};
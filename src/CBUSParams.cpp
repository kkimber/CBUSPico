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

uint8_t CBUSParams::params[21] = {};

CBUSParams::CBUSParams(CBUSConfig const &config)
{
   params[0] = 20;                   // byte 0 num params = 20
   params[1] = MANU_MERG;            // byte 1 manf = MERG, 165
   params[4] = config.EE_MAX_EVENTS; // byte 4 num events
   params[5] = config.EE_NUM_EVS;    // byte 5 num evs per event
   params[6] = config.EE_NUM_NVS;    // byte 6 num NVs
   params[10] = PB_CAN;              // byte 10 CAN implementation of CBUS
   params[11] = 0x00;
   params[12] = 0x00;
   params[13] = 0x00;
   params[14] = 0x00;
   initProcessorParams();
}

void CBUSParams::setVersion(char major, char minor, char beta)
{
   params[7] = major; // byte  7 code major version
   params[2] = minor; // byte  2 code minor version
   params[20] = beta; // byte 20 code beta version
}

void CBUSParams::setModuleId(uint8_t id)
{
   params[3] = id; // byte 3 module id
}

void CBUSParams::setFlags(uint8_t flags)
{
   params[8] = flags; // byte 8 flags - FLiM, consumer/producer
}

// Optional: use this to override processor info that is set by default.
void CBUSParams::setProcessor(uint8_t manufacturer, uint8_t id, char const *name)
{
   params[9] = id;               // byte  9 processor id
   params[19] = manufacturer;    // byte 19 processor manufacturer
   memcpy(params + 15, name, 4); // byte 15-18 processor version
}

uint8_t *CBUSParams::getParams()
{
   return params;
}

void CBUSParams::initProcessorParams()
{
   params[9] = 50;        // byte  9 processor id
   params[19] = CPUM_ARM; // byte 19 processor manufacturer

   params[15] = '?'; // byte 15-18 processor version
   params[16] = '?';
   params[17] = '?';
   params[18] = '?';
}
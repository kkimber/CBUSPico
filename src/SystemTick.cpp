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

#include "SystemTick.h"

#include <RP2040.h>
#include <pico/stdlib.h>
#include <hardware/irq.h>

// Initialize milliseconds since boot counter
uint32_t SystemTick::m_nMilliTicks = 0x0UL;

#define MHZ_TO_MS 1000

SystemTick::SystemTick()
{

}

bool SystemTick::Init(void)
{
   // Configure SysTick for 1ms tick rate
   return SysTick_Config(SystemCoreClock / MHZ_TO_MS) == 0;
}

void SystemTick::IncMilli(void)
{
   // Increment milliseconds since boot counter
   SystemTick::m_nMilliTicks++;
}

uint32_t SystemTick::GetMilli(void)
{
   // Get milliseconds since boot counter value
   return m_nMilliTicks;
}

extern "C" void SysTick_Handler(void)
{
   // Increment millisecond since boot tick counter on every SysTick
   SystemTick::IncMilli();
}

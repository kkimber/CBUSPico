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

#include <cstdint>

// non-blocking switch class

class CBUSSwitch
{

public:
   CBUSSwitch();
   void setPin(uint8_t pin, bool pressedState);
   void run(void);
   void reset(void);
   bool stateChanged(void);
   bool getState(void);
   bool isPressed(void);
   uint32_t getCurrentStateDuration(void);
   uint32_t getLastStateDuration(void);
   uint32_t getLastStateChangeTime(void);
   void resetCurrentDuration(void);

private:
   bool _readPin(uint8_t pin);
   bool m_configured;
   uint8_t m_pin;
   uint8_t m_pressedState;
   uint8_t m_currentState;
   uint8_t m_lastState;
   uint8_t m_stateChanged;
   uint32_t m_lastStateChangeTime;
   uint32_t m_lastStateDuration;
   uint32_t m_prevReleaseTime;
   uint32_t m_prevStateDuration;
};
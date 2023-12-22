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

#include "CBUSSwitch.h"
#include "SystemTick.h"

#include <pico/stdlib.h>

//
/// a class to encapsulate a physical pushbutton switch, with non-blocking processing
//
CBUSSwitch::CBUSSwitch() : m_pin{0x0U},
                           m_pressedState{0x0U},
                           m_currentState{0x0U},
                           m_lastState{0x0U},
                           m_stateChanged{0x0U},
                           m_lastStateChangeTime{0x0UL},
                           m_lastStateDuration{0x0UL},
                           m_prevReleaseTime{0x0UL},
                           m_prevStateDuration{0x0UL}

{
}

void CBUSSwitch::setPin(uint8_t pin, bool pressedState = false)
{
   m_pin = pin;
   m_pressedState = pressedState;

   // Set pin as GPIO Input
   gpio_init(pin);
   gpio_set_dir(pin, GPIO_IN);

   // Setup appropriate pull-up or pull-down on pin
   if (m_pressedState == false)
   {
      // Active low, set pull UP
      gpio_set_pulls(m_pin, true, false);
   }
   else
   {
      // Active high, set pull DOWN
      gpio_set_pulls(m_pin, false, true);
   }

   // Reset internal states to match new pin definition and pin state
   reset();
   m_currentState = _readPin(m_pin);
}

void CBUSSwitch::run(void)
{
   // check for state change

   // read the pin
   m_currentState = _readPin(m_pin);

   // has state changed ?
   if (m_currentState != m_lastState)
   {
      // yes - state has changed since last call to this method
      m_lastState = m_currentState;
      m_prevStateDuration = m_lastStateDuration;
      m_lastStateDuration = SystemTick::GetMilli() - m_lastStateChangeTime;
      m_lastStateChangeTime = SystemTick::GetMilli();
      m_stateChanged = true;

      // has key been released
      if (m_currentState != m_pressedState)
      {
         // save release time
         m_prevReleaseTime = m_lastStateChangeTime;
      }
   }
   else
   {
      // no -- state has not changed
      m_stateChanged = false;
   }
}

void CBUSSwitch::reset(void)
{
   // Initialize internal states
   m_lastState = !m_pressedState;
   m_stateChanged = false;
   m_lastStateChangeTime = 0x0UL;
   m_lastStateDuration = 0x0UL;
   m_prevReleaseTime = 0x0UL;
   m_prevStateDuration = 0x0UL;
}

bool CBUSSwitch::stateChanged(void)
{
   // has switch state changed ?
   return m_stateChanged;
}

bool CBUSSwitch::getState(void)
{
   // return the current switch state read
   return m_currentState;
}

bool CBUSSwitch::isPressed(void)
{
   // is the switch pressed ?
   return (m_currentState == m_pressedState);
}

uint32_t CBUSSwitch::getCurrentStateDuration(void)
{
   // how long has the switch been in its current state ?
   return (SystemTick::GetMilli() - m_lastStateChangeTime);
}

uint32_t CBUSSwitch::getLastStateDuration(void)
{
   // how long was the last state active for ?
   return m_lastStateDuration;
}

uint32_t CBUSSwitch::getLastStateChangeTime(void)
{
   // when was the last state change ?
   return m_lastStateChangeTime;
}

/// reset the state duration counter
void CBUSSwitch::resetCurrentDuration(void)
{
   m_lastStateChangeTime = SystemTick::GetMilli();
}

/// Read the GPIO pin level
bool CBUSSwitch::_readPin(uint8_t pin)
{
   return gpio_get(pin);
}

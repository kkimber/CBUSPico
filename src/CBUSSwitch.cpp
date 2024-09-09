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

#include "CBUSSwitch.h"
#include "SystemTick.h"

#include <pico/stdlib.h>

/// Debounce delay of 20ms for all switches
constexpr uint32_t DEBOUNCE_DELAY {20UL};

///
/// A class to encapsulate a physical pushbutton switch, with non-blocking processing
///

CBUSSwitch::CBUSSwitch() : m_configured{false},
                           m_pin{0x0U},
                           m_pressedState{false},
                           m_currentState{true},
                           m_lastState{false},
                           m_activeState{false},
                           m_stateChanged{false},
                           m_debounceStartTime{0x0UL},
                           m_lastStateChangeTime{0x0UL},
                           m_lastStateDuration{0x0UL},
                           m_prevReleaseTime{0x0UL},
                           m_prevStateDuration{0x0UL}
{
}

///
/// @brief Set the pin for this Switch, defines if the button is active HIGH or LOW
///        configures the pin as a GPIO input and sets a suitable pull (UP or DOWN)
///
/// @param pin Pin number of the pin to assign to this Switch
/// @param pressedState active state of the pin, true = press = HIGH, false for LOW, default is LOW
///
void CBUSSwitch::setPin(const uint8_t pin, const bool pressedState = false)
{
   // Set the pin and active state
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

   // Switch has now been configured
   m_configured = true;

   // Reset internal states to match new pin definition and read current pin state
   reset();
   m_currentState = _readPin();
}

///
/// @brief Process the Switch, must be called frequently
///
void CBUSSwitch::run()
{
   // read the pin
   m_currentState = _readPin();

   // has state changed?
   if (m_currentState != m_lastState)
   {
      // record time of change of state
      m_debounceStartTime = SystemTick::GetMilli();
   }

   // check for persistent state exceeding debounce time
   if ((SystemTick::GetMilli() - m_debounceStartTime) > DEBOUNCE_DELAY)
   {
      // has state changed?
      if (m_currentState != m_activeState)
      {
         // yes - state has changed since last call to this method
         m_activeState = m_currentState;
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

  m_lastState = m_currentState;
}

///
/// @brief Reset the internal states of the Switch
///
void CBUSSwitch::reset()
{
   // Initialize internal states
   m_lastState = !m_pressedState;
   m_stateChanged = false;
   m_lastStateChangeTime = 0x0UL;
   m_lastStateDuration = 0x0UL;
   m_prevReleaseTime = 0x0UL;
   m_prevStateDuration = 0x0UL;
}

///
/// @brief Determine if the state of the switch has changed
///
/// @return true The state of the switch has changed
/// @return false The state of the switch has no changed
///
bool CBUSSwitch::stateChanged() const
{
   // has switch state changed ?
   return m_stateChanged;
}

///
/// @brief Get the current state of the switch pin
///
/// @return true The pin is logical HIGH
/// @return false The pin is logical LOW
///
bool CBUSSwitch::getState() const
{
   // return the current switch state read (after debounce)
   return m_activeState;
}

///
/// @brief Determine if the Switch has been detected as Pressed
///
/// @return true The Switch is Pressed
/// @return false The Switch is not Pressed
///
bool CBUSSwitch::isPressed() const
{
   // is the switch pressed ?
   return (m_activeState == m_pressedState);
}

///
/// @brief Determine how long the Switch has been in its current state
///
/// @return uint32_t Duration in the current state, in milliseconds
///
uint32_t CBUSSwitch::getCurrentStateDuration()
{
   // how long has the switch been in its current state ?
   return (SystemTick::GetMilli() - m_lastStateChangeTime);
}

///
/// @brief Determine how long the Switch was in its previous state
///
/// @return uint32_t Duration in previous state, in milliseconds
///
uint32_t CBUSSwitch::getLastStateDuration()
{
   // how long was the last state active for ?
   return m_lastStateDuration;
}

///
/// @brief Determine when the Switch last changed state
///
/// @return uint32_t Timestamp of the last state change, in milliseconds since boot
///
uint32_t CBUSSwitch::getLastStateChangeTime()
{
   // when was the last state change ?
   return m_lastStateChangeTime;
}

///
/// @brief Reset the timestamp of the last state change
///
///
void CBUSSwitch::resetCurrentDuration()
{
   m_lastStateChangeTime = SystemTick::GetMilli();
}

///
/// @brief Read the phyical pin state
///        If the pin is not configured it will return true
///
/// @return true The pin is logical HIGH
/// @return false The pin is logical LOW
///
bool CBUSSwitch::_readPin()
{
   // If configured, read the physical pin state
   if (m_configured)
   {
      return gpio_get(m_pin);
   }

   // Unconfigured, return true, as default state is active LOW
   return true;
}

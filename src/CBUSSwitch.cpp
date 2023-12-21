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

   reset();
   m_currentState = _readPin(m_pin);
}

void CBUSSwitch::reset(void)
{
   m_lastState = !m_pressedState;
   m_stateChanged = false;
   m_lastStateChangeTime = 0x0UL;
   m_lastStateDuration = 0x0UL;
   m_prevReleaseTime = 0x0UL;
   m_prevStateDuration = 0x0UL;
}

bool CBUSSwitch::_readPin(uint8_t pin)
{
   return gpio_get(pin);
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

      if (m_currentState == m_pressedState)
      {
         // DEBUG_SERIAL << F("  -- switch has been pressed") << endl;
      }
      else
      {
         // DEBUG_SERIAL << F("  -- switch has been released") << endl;

         // double-click detection
         // two clicks of less than 250ms, less than 500ms apart

         // DEBUG_SERIAL << F("  -- last state duration = ") << m_lastStateDuration << endl;
         // DEBUG_SERIAL << F("  -- this release = ") << m_lastStateChangeTime << F(", last release = ") << m_prevReleaseTime << endl;

         // save release time
         m_prevReleaseTime = m_lastStateChangeTime;
      }
   }
   else
   {
      // no -- state has not changed
      m_stateChanged = false;
   }

   return;
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

unsigned long CBUSSwitch::getCurrentStateDuration(void)
{
   // how long has the switch been in its current state ?
   return (SystemTick::GetMilli() - m_lastStateChangeTime);
}

unsigned long CBUSSwitch::getLastStateDuration(void)
{
   // how long was the last state active for ?
   return m_lastStateDuration;
}

unsigned long CBUSSwitch::getLastStateChangeTime(void)
{
   // when was the last state change ?
   return m_lastStateChangeTime;
}

void CBUSSwitch::resetCurrentDuration(void)
{
   // reset the state duration counter
   m_lastStateChangeTime = SystemTick::GetMilli();
   return;
}
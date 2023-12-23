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

#include "CBUSLED.h"
#include "SystemTick.h"

#include <pico/stdlib.h>

//
/// class for individual LED with non-blocking control
//

CBUSLED::CBUSLED() : m_configured{false},
                     m_pin{0x0U},
                     m_state{false},
                     m_blink{false},
                     m_pulse{false},
                     m_lastTime{0x0UL},
                     m_pulseStart{0x0UL}
{
}

//  set the pin for this LED

void CBUSLED::setPin(uint8_t pin)
{
   m_pin = pin;

   gpio_init(pin);
   gpio_set_dir(pin, GPIO_OUT);
   gpio_put(pin, false);

   m_configured = true;
}

// return the current state, on or off

bool CBUSLED::getState()
{
   return m_state;
}

// turn LED state on

void CBUSLED::on(void)
{
   m_state = true;
   m_blink = false;
}

// turn LED state off

void CBUSLED::off(void)
{
   m_state = false;
   m_blink = false;
}

// toggle LED state from on to off or vv

void CBUSLED::toggle(void)
{
   m_state = !m_state;
}

// blink LED

void CBUSLED::blink()
{
   m_blink = true;
}

// pulse the LED

void CBUSLED::pulse()
{
   m_pulse = true;
   m_state = true;
   m_pulseStart = SystemTick::GetMilli();
   run();
}

// actually operate the LED dependent upon its current state
// must be called frequently from loop() if the LED is set to blink or pulse

void CBUSLED::run()
{
   if (m_blink)
   {
      // blinking
      if ((SystemTick::GetMilli() - m_lastTime) >= BLINK_RATE)
      {
         toggle();
         m_lastTime = SystemTick::GetMilli();
      }
   }

   // single pulse
   if (m_pulse)
   {
      if ((SystemTick::GetMilli() - m_pulseStart) >= PULSE_ON_TIME)
      {
         m_pulse = false;
         m_state = false;
      }
   }

   _write(m_pin, m_state);
}

// write to the physical pin

void CBUSLED::_write(uint8_t pin, bool state)
{
   if (m_configured)
   {
      gpio_put(pin, state);
   }
}

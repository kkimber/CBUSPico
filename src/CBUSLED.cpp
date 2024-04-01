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

#include "CBUSLED.h"
#include "SystemTick.h"

#include <pico/stdlib.h>

constexpr uint16_t BLINK_RATE = 500;         ///< flash at 1Hz, 500mS on, 500mS off
constexpr uint16_t SHORT_FILCKER_TIME = 100; ///< short flicker duration 100mS - Non consumed CAN event
constexpr uint16_t LONG_FLICKER_TIME = 500;  ///< long flicker duration 500mS - Consumed CAN event

///
/// Class to control an individual LED, with non-blocking control
///

CBUSLED::CBUSLED() : m_configured{false},
                     m_pin{0x0U},
                     m_state{false},
                     m_blink{false},
                     m_pulse{false},
                     m_lastTime{0x0UL},
                     m_pulseStart{0x0UL},
                     m_pulseDuration{0x0U}
{
}

///
/// @brief Set the pin for this LED
///        configures the pin as a GPIO ouput and sets the output LOW
///
/// @param pin Pin number of the pin to assign to this LED
///
void CBUSLED::setPin(const uint8_t pin)
{
   // Assign the pin
   m_pin = pin;

   // Initialize as GPIO output and drive LOW
   gpio_init(pin);
   gpio_set_dir(pin, GPIO_OUT);
   gpio_put(pin, false);

   // This class instance is now configured
   m_configured = true;
}

///
/// @brief Get the current state of the LED output pin
///
/// @return true The output pin is HIGH
/// @return false The output pin is LOW
///
bool CBUSLED::getState() const
{
   return m_state;
}

///
/// @brief Turns the LED on
///
void CBUSLED::on()
{
   m_state = true;
   m_blink = false;
}

///
/// @brief Turns the LED off
///
void CBUSLED::off()
{
   m_state = false;
   m_blink = false;
}

///
/// @brief Toggles the current state of the LED from On to Off or vice versa
///
void CBUSLED::toggle()
{
   m_state = !m_state;
}

///
/// @brief Sets the LED into blinking mode
///
void CBUSLED::blink()
{
   m_blink = true;
}

///
/// @brief Sets the LED to pulse (on) once
///
void CBUSLED::pulse(bool bShort)
{
   if (bShort)
   {
      m_pulseDuration = SHORT_FILCKER_TIME;
   }
   else
   {
      m_pulseDuration = LONG_FLICKER_TIME;
   }

   m_pulse = true;
   m_state = true;
   m_pulseStart = SystemTick::GetMilli();

   // Run now to turn on immediately
   run();
}

///
/// @brief Process the LED based on its configured state,
///        must be called frequently if the LED is set to blink or pulse
///
void CBUSLED::run()
{
   // blinking
   if (m_blink)
   {
      if ((SystemTick::GetMilli() - m_lastTime) >= BLINK_RATE)
      {
         toggle();
         m_lastTime = SystemTick::GetMilli();
      }
   }

   // single pulse
   if (m_pulse)
   {
      if ((SystemTick::GetMilli() - m_pulseStart) >= m_pulseDuration)
      {
         m_pulse = false;
         m_state = false;
      }
   }

   // Update the pin state
   _write();
}

///
/// @brief Output the current state to the physical pin
///
void CBUSLED::_write()
{
   if (m_configured)
   {
      gpio_put(m_pin, m_state);
   }
}

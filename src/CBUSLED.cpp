#include "CBUSLED.h"
#include "SystemTick.h"

#include <pico/stdlib.h>

//
/// class for individual LED with non-blocking control
//

CBUSLED::CBUSLED() : m_state{false},
                     m_blink{false},
                     m_pulse{false},
                     m_lastTime{0UL}
{
}

CBUSLED::~CBUSLED()
{
}

//  set the pin for this LED

void CBUSLED::setPin(uint8_t pin)
{
   m_pin = pin;

   gpio_init(pin);
   gpio_set_dir(pin, GPIO_OUT);
   gpio_put(pin, false);
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
      if (SystemTick::GetMilli() - m_pulseStart >= PULSE_ON_TIME)
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
   gpio_put(pin, state);
}

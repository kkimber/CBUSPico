#pragma once

#include <cstdint>

#define BLINK_RATE 500 // flash at 1Hz, 500mS on, 500mS off
#define PULSE_ON_TIME 5

//
/// class to encapsulate a non-blocking LED
//

class CBUSLED
{

public:
   /// @brief Construct CBUS LED Class Instance
   CBUSLED();
   virtual ~CBUSLED();
   void virtual setPin(uint8_t pin);
   bool getState();
   void on();
   void off();
   void toggle();
   void blink();
   virtual void run();
   void pulse();

protected:
   uint8_t m_pin;
   bool m_state;
   bool m_blink;
   bool m_pulse;
   uint32_t m_lastTime;
   uint32_t m_pulseStart;
   virtual void _write(uint8_t pin, bool state);
};
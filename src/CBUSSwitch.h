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
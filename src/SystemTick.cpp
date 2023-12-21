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

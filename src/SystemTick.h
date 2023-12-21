#include <cstdint>

class SystemTick
{
public:   
   SystemTick();

   bool Init(void);
   static void IncMilli(void);
   static uint32_t GetMilli(void);

private:
   static uint32_t m_nMilliTicks;
};
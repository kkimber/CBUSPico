#include <CBUSParams.h>

uint8_t CBUSParams::params[21] = {};

void CBUSParams::initProcessorParams()
{
  params[9] = 50;
  params[19] = CPUM_ARM;

  params[15] = '?';
  params[16] = '?';
  params[17] = '?';
  params[18] = '?';
}
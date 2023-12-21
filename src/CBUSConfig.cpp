//
/// CBUS module configuration
/// manages the storage of events and node variables in on-chip or external EEPROM
//

#include "CBUSConfig.h"
#include "SystemTick.h"

#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <pico/binary_info.h>
#include <hardware/gpio.h>
#include <hardware/i2c.h>
#include <hardware/flash.h>
#include <hardware/sync.h>

// WHY
#include <hardware/watchdog.h>
extern "C" char *sbrk(int incr);

// Replace with C++ constructs
#include <stdlib.h>
#include <string.h>

#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))

//
/// ctor
//

CBUSConfig::CBUSConfig()
{
   eeprom_type = EEPROM_INTERNAL;
   // I2Cbus = &Wire;
}

//
/// initialise and set default values
//

void CBUSConfig::begin(void)
{
   EE_BYTES_PER_EVENT = EE_NUM_EVS + 4;

   if (eeprom_type == EEPROM_INTERNAL)
   {
      // these devices require an explicit begin with the desired emulated size

#if defined ESP32 || defined ESP8266
      EEPROM.begin(EE_EVENTS_START + (EE_MAX_EVENTS * EE_BYTES_PER_EVENT));
#endif

#ifdef ARDUINO_ARCH_RP2040
      EEPROM.begin(4096);
#endif
   }

   if (eeprom_type == EEPROM_USES_FLASH)
   {

// #ifdef __AVR_XMEGA__
#if defined(DXCORE)
      // check flash is writable
      uint8_t check = Flash.checkWritable();

      if (check == FLASHWRITE_OK)
      {
         // DEBUG_SERIAL << F("> flash is writable, PROGMEM_SIZE = ") << PROGMEM_SIZE << endl;
      }
      else
      {
         // DEBUG_SERIAL << F("> flash is not writable, ret = ") << check << endl;
      }

      // cache the first page into memory
      flash_cache_page(0);
#endif
   }

   makeEvHashTable();
   loadNVs();
}

//
/// set the EEPROM type for event storage - on-chip or external I2C bus device
/// NVs are always stored in the on-chip EEPROM
/// external EEPROM must use 16-bit addresses !!
//

bool CBUSConfig::setEEPROMtype(uint8_t type)
{
   bool ret = true;
   uint8_t result;
   eeprom_type = EEPROM_INTERNAL;

   switch (type)
   {
   case EEPROM_EXTERNAL:
      // test accessibility of external EEPROM chip
//      I2Cbus->begin();
//      I2Cbus->beginTransmission(external_address);
//      result = I2Cbus->endTransmission();

      if (result == 0)
      {
         eeprom_type = type;
      }
      else
      {
         eeprom_type = EEPROM_INTERNAL;
         ret = false;
      }
      break;

   case EEPROM_USES_FLASH:

// #ifdef __AVR_XMEGA__
#if defined(DXCORE)
      eeprom_type = EEPROM_USES_FLASH;
#else
      eeprom_type = EEPROM_INTERNAL;
#endif
      break;
   }

   return ret;
}

//
/// set the bus address of an external EEPROM chip
//
#if 0
void CBUSConfig::setExtEEPROMAddress(uint8_t address, TwoWire *bus)
{
   external_address = address;
   I2Cbus = bus;
}
#endif

//
/// store the FLiM mode
//

void CBUSConfig::setFLiM(bool f)
{

   FLiM = f;
   writeEEPROM(0, f);
   return;
}

//
/// store the CANID
//

void CBUSConfig::setCANID(uint8_t canid)
{

   CANID = canid;
   writeEEPROM(1, canid);
   return;
}

//
/// store the node number
//

void CBUSConfig::setNodeNum(uint32_t nn)
{
   nodeNum = nn;
   writeEEPROM(2, highByte(nodeNum));
   writeEEPROM(3, lowByte(nodeNum));
   return;
}

//
/// lookup an event by node number and event number, using the hash table
//

uint8_t CBUSConfig::findExistingEvent(uint32_t nn, uint32_t en)
{
   uint8_t tarray[4];
   uint8_t tmphash, i, j, matches;
   bool confirmed = false;

   tarray[0] = highByte(nn);
   tarray[1] = lowByte(nn);
   tarray[2] = highByte(en);
   tarray[3] = lowByte(en);

   // calc the hash of the incoming event to match
   tmphash = makeHash(tarray);
   // DEBUG_SERIAL << F("> event hash = ") << tmphash << endl;

   for (i = 0; i < EE_MAX_EVENTS; i++)
   {

      if (evhashtbl[i] == tmphash)
      {
         if (!hash_collision)
         {
            // NN + EN hash matches and there are no hash collisions in the hash table
         }
         else
         {
            // there is a potential hash collision, so we have to check the slower way
            // first, check if this hash appears in the table more than once

            // DEBUG_SERIAL << F("> there are hash collisions") << endl;

            for (j = 0, matches = 0; j < EE_MAX_EVENTS; j++)
            {
               if (evhashtbl[j] == tmphash)
               {
                  ++matches;
               }
            }

            if (matches > 1)
            {
               // one or more collisions for this hash exist, so check the very slow way
               // DEBUG_SERIAL << F("> this hash matches = ") << matches << endl;

               for (i = 0; i < EE_MAX_EVENTS; i++)
               {
                  if (evhashtbl[i] == tmphash)
                  {
                     // check the EEPROM for a match with the incoming NN and EN
                     readEvent(i, tarray);
                     if ((uint32_t)((tarray[0] << 8) + tarray[1]) == nn && (uint32_t)((tarray[2] << 8) + tarray[3]) == en)
                     {
                        // the stored NN and EN match this event, so no further checking is required
                        confirmed = true;
                        break;
                     }
                  }
               }
            }
            else
            {
               // no collisions for this specific hash, so no further checking is required
               break;
            }
         }

         break;
      }
   }

   // finally, there may be a collision with an event that we haven't seen before
   // so we still need to check the candidate match to be certain, if we haven't done so already

   if (i < EE_MAX_EVENTS && !confirmed)
   {
      readEvent(i, tarray);
      if (!((uint32_t)((tarray[0] << 8) + tarray[1]) == nn && (uint32_t)((tarray[2] << 8) + tarray[3]) == en))
      {
         // the stored NN and EN do not match this event
         // DEBUG_SERIAL << F("> hash result does not match stored event") << endl;
         i = EE_MAX_EVENTS;
      }
   }

   // if (i >= EE_MAX_EVENTS) DEBUG_SERIAL << F("> unable to find matching event") << endl;
   return i;
}

//
/// find the first empty EEPROM event slot - the hash table entry == 0
//

uint8_t CBUSConfig::findEventSpace(void)
{

   uint8_t evidx;

   for (evidx = 0; evidx < EE_MAX_EVENTS; evidx++)
   {
      if (evhashtbl[evidx] == 0)
      {
         // DEBUG_SERIAL << F("> found unused location at index = ") << evidx << endl;
         break;
      }
   }

   return evidx;
}

//
/// create a hash from a 4-uint8_t event entry array -- NN + EN
//

uint8_t CBUSConfig::makeHash(uint8_t tarr[4])
{

   uint8_t hash = 0;
   uint32_t nn, en;

   // make a hash from a 4-uint8_t NN + EN event

   nn = (tarr[0] << 8) + tarr[1];
   en = (tarr[2] << 8) + tarr[3];

   // need to hash the NN and EN to a uniform distribution across HASH_LENGTH
   hash = nn ^ (nn >> 8);
   hash = 7 * hash + (en ^ (en >> 8));

   // ensure it is within bounds and non-zero
   hash %= HASH_LENGTH;
   hash = (hash == 0) ? 255 : hash;

   // DEBUG_SERIAL << F("> makeHash - hash of nn = ") << nn << F(", en = ") << en << F(", = ") << hash << endl;
   return hash;
}

//
/// return an existing EEPROM event as a 4-uint8_t array -- NN + EN
//

void CBUSConfig::readEvent(uint8_t idx, uint8_t tarr[])
{

   // populate the array with the first 4 bytes (NN + EN) of the event entry from the EEPROM
   for (uint8_t i = 0; i < EE_HASH_BYTES; i++)
   {
      tarr[i] = readEEPROM(EE_EVENTS_START + (idx * EE_BYTES_PER_EVENT) + i);
   }

   // DEBUG_SERIAL << F("> readEvent - idx = ") << idx << F(", nn = ") << (tarr[0] << 8) + tarr[1] << F(", en = ") << (tarr[2] << 8) + tarr[3] << endl;
   return;
}

//
/// return an event variable (EV) value given the event table index and EV number
//

uint8_t CBUSConfig::getEventEVval(uint8_t idx, uint8_t evnum)
{

   return readEEPROM(EE_EVENTS_START + (idx * EE_BYTES_PER_EVENT) + 3 + evnum);
}

//
/// write an event variable
//

void CBUSConfig::writeEventEV(uint8_t idx, uint8_t evnum, uint8_t evval)
{

   writeEEPROM(EE_EVENTS_START + (idx * EE_BYTES_PER_EVENT) + 3 + evnum, evval);
}

//
/// re/create the event hash table
//

void CBUSConfig::makeEvHashTable(void)
{

   uint8_t evarray[4];
   const uint8_t unused_entry[4] = {0xff, 0xff, 0xff, 0xff};

   // DEBUG_SERIAL << F("> creating event hash table") << endl;

   evhashtbl = (uint8_t *)malloc(EE_MAX_EVENTS * sizeof(uint8_t));

   for (uint8_t idx = 0; idx < EE_MAX_EVENTS; idx++)
   {

      readEvent(idx, evarray);

      // empty slots have all four bytes set to 0xff
      if (memcmp(evarray, unused_entry, 4) == 0)
      {
         evhashtbl[idx] = 0;
      }
      else
      {
         evhashtbl[idx] = makeHash(evarray);
      }
   }

   hash_collision = check_hash_collisions();

   return;
}

//
/// update a single hash table entry -- after a learn or unlearn
//

void CBUSConfig::updateEvHashEntry(uint8_t idx)
{

   uint8_t evarray[4];
   const uint8_t unused_entry[4] = {0xff, 0xff, 0xff, 0xff};

   // read the first four bytes from EEPROM - NN + EN
   readEvent(idx, evarray);

   // empty slots have all four bytes set to 0xff
   if (memcmp(evarray, unused_entry, 4) == 0)
   {
      evhashtbl[idx] = 0;
   }
   else
   {
      evhashtbl[idx] = makeHash(evarray);
   }

   hash_collision = check_hash_collisions();

   // DEBUG_SERIAL << F("> updateEvHashEntry for idx = ") << idx << F(", hash = ") << hash << endl;
   return;
}

//
/// clear the hash table
//

void CBUSConfig::clearEvHashTable(void)
{

   // zero in the hash table indicates that the corresponding event slot is free
   // DEBUG_SERIAL << F("> clearEvHashTable - clearing hash table") << endl;

   for (uint8_t i = 0; i < EE_MAX_EVENTS; i++)
   {
      evhashtbl[i] = 0;
   }

   hash_collision = false;
   return;
}

//
/// print the event hash table
//

void CBUSConfig::printEvHashTable(bool raw)
{

   // removed so that no libraries produce serial output
   // can be implemented in user's sketch
   (void)raw;

   /*
     DEBUG_SERIAL << F("> Event hash table - ") << hash_collision << endl;

     for (uint8_t i = 0; i < EE_MAX_EVENTS; i++) {
       if (evhashtbl[i] > 0) {
         if (raw)
           DEBUG_SERIAL << evhashtbl[i] << endl;
         else
           DEBUG_SERIAL << i << " - " << evhashtbl[i] << endl;
       }
     }

     DEBUG_SERIAL << endl;
   */

   return;
}

//
/// return the number of stored events
//

uint8_t CBUSConfig::numEvents(void)
{

   uint8_t numevents = 0;

   for (uint8_t i = 0; i < EE_MAX_EVENTS; i++)
   {
      if (evhashtbl[i] != 0)
      {
         ++numevents;
      }
   }

   return numevents;
}

//
/// return a single hash table entry by index
//

uint8_t CBUSConfig::getEvTableEntry(uint8_t tindex)
{

   if (tindex < EE_MAX_EVENTS)
   {
      return evhashtbl[tindex];
   }
   else
   {
      return 0;
   }
}

//
/// read an NV value from EEPROM
/// note that NVs number from 1, not 0
//

uint8_t CBUSConfig::readNV(uint8_t idx)
{

   return (readEEPROM(EE_NVS_START + (idx - 1)));
}

//
/// write an NV value to EEPROM
/// note that NVs number from 1, not 0
//

void CBUSConfig::writeNV(uint8_t idx, uint8_t val)
{

   writeEEPROM(EE_NVS_START + (idx - 1), val);
   return;
}

//
/// generic EEPROM access methods
//

//
/// read a single uint8_t from EEPROM
//

uint8_t CBUSConfig::readEEPROM(uint32_t eeaddress)
{

   uint8_t rdata = 0;
   int r = 0;

   // DEBUG_SERIAL << F("> readEEPROM, addr = ") << eeaddress << endl;

   switch (eeprom_type)
   {

   case EEPROM_EXTERNAL:
#if 0
      I2Cbus->beginTransmission(external_address);
      I2Cbus->write((int)(eeaddress >> 8));   // MSB
      I2Cbus->write((int)(eeaddress & 0xFF)); // LSB
      r = I2Cbus->endTransmission();

      if (r < 0)
      {
         // DEBUG_SERIAL << F("> readEEPROM: I2C write error = ") << r << endl;
      }

      I2Cbus->requestFrom((int)external_address, (int)1);

      if (I2Cbus->available())
         rdata = I2Cbus->read();
#endif         
      break;

   case EEPROM_INTERNAL:
      rdata = getChipEEPROMVal(eeaddress);
      break;

   case EEPROM_USES_FLASH:
// #ifdef __AVR_XMEGA__
#if defined(DXCORE)
      rdata = Flash.readByte(FLASH_AREA_BASE_ADDRESS + eeaddress);
      // DEBUG_SERIAL << F("> read uint8_t = ") << rdata << F(" from address = ") << eeaddress << endl;
#endif
      break;
   }

   return rdata;
}

//
/// read a number of bytes from EEPROM
/// external EEPROM must use 16-bit addresses !!
//

uint8_t CBUSConfig::readBytesEEPROM(uint32_t eeaddress, uint8_t nbytes, uint8_t dest[])
{

   int r = 0;
   uint8_t count = 0;

   switch (eeprom_type)
   {

   case EEPROM_EXTERNAL:
#if 0   
      I2Cbus->beginTransmission(external_address);
      I2Cbus->write((int)(eeaddress >> 8));   // MSB
      I2Cbus->write((int)(eeaddress & 0xFF)); // LSB
      r = I2Cbus->endTransmission();

      if (r < 0)
      {
         // DEBUG_SERIAL << F("> readBytesEEPROM: I2C write error = ") << r << endl;
      }

      I2Cbus->requestFrom((int)external_address, (int)nbytes);

      while (I2Cbus->available() && count < nbytes)
      {
         dest[count++] = I2Cbus->read();
      }
#endif
      // DEBUG_SERIAL << F("> readBytesEEPROM: read ") << count << F(" bytes from EEPROM in ") << micros() - t1 << F("us") << endl;
      break;

   case EEPROM_INTERNAL:
      for (count = 0; count < nbytes; count++)
      {
         dest[count] = getChipEEPROMVal(eeaddress + count);
      }
      break;

   case EEPROM_USES_FLASH:
// #ifdef __AVR_XMEGA__
#if defined(DXCORE)
      for (count = 0; count < nbytes; count++)
      {
         dest[count] = Flash.readByte(FLASH_AREA_BASE_ADDRESS + eeaddress + count);
      }
#endif
      break;
   }

   return count;
}

//
/// write a uint8_t
//

void CBUSConfig::writeEEPROM(uint32_t eeaddress, uint8_t data)
{

   int r = 0;

   // DEBUG_SERIAL << F("> writeEEPROM, addr = ") << eeaddress << F(", data = ") << data << endl;

   switch (eeprom_type)
   {

   case EEPROM_EXTERNAL:
#if 0   
      I2Cbus->beginTransmission(external_address);
      I2Cbus->write((int)(eeaddress >> 8));   // MSB
      I2Cbus->write((int)(eeaddress & 0xFF)); // LSB
      I2Cbus->write(data);
      r = I2Cbus->endTransmission();
      delay(5);

      if (r < 0)
      {
         // DEBUG_SERIAL << F("> writeEEPROM: I2C write error = ") << r << endl;
      }
#endif      
      break;

   case EEPROM_INTERNAL:
      setChipEEPROMVal(eeaddress, data);
      break;

   case EEPROM_USES_FLASH:
// #ifdef __AVR_XMEGA__
#if defined(DXCORE)
      flash_write_bytes(eeaddress, &data, 1);
#endif
      break;
   }

   return;
}

//
/// write a number of bytes to EEPROM
/// external EEPROM must use 16-bit addresses !!
//

void CBUSConfig::writeBytesEEPROM(uint32_t eeaddress, uint8_t src[], uint8_t numbytes)
{

   // *** TODO *** handle greater than 32 bytes -> the Arduino I2C write buffer size
   // max write = EEPROM pagesize - 64 bytes

   int r = 0;

   switch (eeprom_type)
   {
   case EEPROM_EXTERNAL:
#if 0   
      I2Cbus->beginTransmission(external_address);
      I2Cbus->write((int)(eeaddress >> 8));   // MSB
      I2Cbus->write((int)(eeaddress & 0xFF)); // LSB

      for (uint8_t i = 0; i < numbytes; i++)
      {
         I2Cbus->write(src[i]);
      }

      r = I2Cbus->endTransmission();
      delay(5);

      if (r < 0)
      {
         // DEBUG_SERIAL << F("> writeBytesEEPROM: I2C write error = ") << r << endl;
      }
#endif      
      break;

   case EEPROM_INTERNAL:
      for (uint8_t i = 0; i < numbytes; i++)
      {
         setChipEEPROMVal(eeaddress + i, src[i]);
      }
      break;

   case EEPROM_USES_FLASH:
// #ifdef __AVR_XMEGA__
#if defined(DXCORE)
      flash_write_bytes(eeaddress, src, numbytes);
#endif
      break;
   }

   return;
}

//
/// write (or clear) an event to EEPROM
/// just the first four bytes -- NN and EN
//

void CBUSConfig::writeEvent(uint8_t index, uint8_t data[])
{

   int eeaddress = EE_EVENTS_START + (index * EE_BYTES_PER_EVENT);

   // DEBUG_SERIAL << F("> writeEvent, index = ") << index << F(", addr = ") << eeaddress << endl;
   writeBytesEEPROM(eeaddress, data, 4);

   return;
}

//
/// clear an event from the table
//

void CBUSConfig::cleareventEEPROM(uint8_t index)
{

   uint8_t unused_entry[4] = {0xff, 0xff, 0xff, 0xff};

   // DEBUG_SERIAL << F("> clearing event at index = ") << index << endl;
   writeEvent(index, unused_entry);

   return;
}

//
/// clear all event data in external EEPROM chip
//

void CBUSConfig::resetEEPROM(void)
{

   if (eeprom_type == EEPROM_EXTERNAL)
   {

      // DEBUG_SERIAL << F("> clearing data from external EEPROM ...") << endl;

      for (uint32_t addr = 10; addr < 4096; addr++)
      {
         writeEEPROM(addr, 0xff);
      }
   }
   else if (eeprom_type == EEPROM_USES_FLASH)
   {
// #ifdef __AVR_XMEGA__
#if defined(DXCORE)
      for (uint8_t i = 0; i < 4; i++)
      {
         memset(cache_page.data, 0xff, FLASH_PAGE_SIZE);
         cache_page.dirty = true;
         flash_writeback_page(i);
      }
#endif
   }

   return;
}

//
/// reboot the processor
//

#ifdef __AVR__
#include <avr/wdt.h>
#endif

#ifndef AVR_RESET_METHOD
#define AVR_RESET_METHOD 3 // don't use watchdog timer method as it's unreliable on some boards
#endif

void (*rebootFunc)(void) = 0; // just causes a jump to address zero - not a full chip reset

void CBUSConfig::reboot(void)
{

#ifdef __AVR__

// for newer AVR Xmega, e.g. AVR-DA
// #ifdef __AVR_XMEGA__
#if defined(DXCORE)
   _PROTECTED_WRITE(RSTCTRL.SWRR, 1);
   // CCP = (uint8_t)CCP_IOREG_gc;
   // RSTCTRL.SWRR = 1;
#else

// for older AVR Mega, e.g. Uno, Nano, Mega
#if AVR_RESET_METHOD == 1 // 1. the preferred way
   // #warning "Using reset method 1"
   wdt_disable();
   wdt_enable(WDTO_15MS);
   while (1)
   {
   }
#endif

#if AVR_RESET_METHOD == 2 // 2. a variation on the watchdog method
   // #warning "Using reset method 2"
   wdt_enable(WDTO_15MS);
   while (1)
   {
   }
#endif

#if AVR_RESET_METHOD == 3 // 3. a dirty way
   // #warning "Using reset method 3"
   asm volatile("jmp 0");
#endif

#if AVR_RESET_METHOD == 4 // 4. another dirty way
   // #warning "Using reset method 4"
   rebootFunc();
#endif

#endif // which AVR
#endif // AVR

// for ESP32
#if defined ESP32 || defined ESP8266
   ESP.restart();
#endif

// for Arduino Due
#ifdef __SAM3X8E__
   rstc_start_software_reset(RSTC);
#endif

// for Raspberry Pi Pico using arduino-pico core
#ifdef ARDUINO_ARCH_RP2040
   watchdog_enable(100, 1); // set watchdog timeout to 100ms and allow to expire
   while (1)
      ;
#endif
}

//
/// get free RAM
//

uint32_t CBUSConfig::freeSRAM(void)
{
#ifdef __AVR__
   extern int __heap_start, *__brkval;
   int v;
   return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
#endif

#if defined ESP32 || defined ESP8266
   return ESP.getFreeHeap();
#endif

#ifdef __SAM3X8E__
   char top;
   return &top - reinterpret_cast<char *>(sbrk(0));
#endif

#ifdef ARDUINO_ARCH_RP2040
   char top;
   return &top - reinterpret_cast<char *>(sbrk(0));
#endif
}

//
/// manually reset the module to factory defaults
//

void CBUSConfig::resetModule(CBUSLED ledGrn, CBUSLED ledYlw, CBUSSwitch pbSwitch)
{
   /// standard implementation of resetModule()

   bool bDone;
   unsigned long waittime;

   // start timeout timer
   waittime = SystemTick::GetMilli();
   bDone = false;

   // DEBUG_SERIAL << F("> waiting for a further 5 sec button push, as a safety measure") << endl;

   pbSwitch.reset();
   ledGrn.blink();
   ledYlw.blink();

   // wait for a further (5 sec) button press -- as a 'safety' mechanism
   while (!bDone)
   {

      // 30 sec timeout
      if ((SystemTick::GetMilli() - waittime) > 30000)
      {
         // DEBUG_SERIAL << F("> timeout expired, reset not performed") << endl;
         return;
      }

      pbSwitch.run();
      ledGrn.run();
      ledYlw.run();

      // wait until switch held for a further 5 secs
      if (pbSwitch.isPressed() && pbSwitch.getCurrentStateDuration() > 5000)
      {
         bDone = true;
      }
   }

   // do the reset
   // DEBUG_SERIAL << F("> performing module reset ...") <<  endl;

   ledGrn.off();
   ledYlw.off();
   ledGrn.run();
   ledYlw.run();

   resetModule();
}

void CBUSConfig::resetModule(void)
{

   /// implementation of resetModule() without CBUSswitch or CBUSLEDs
   // uint32_t t = millis();
   // DEBUG_SERIAL << F("> resetting EEPROM") << endl;

   if (eeprom_type == EEPROM_INTERNAL)
   {

      // clear the entire on-chip EEPROM
      // !! note we don't clear the first ten locations (0-9), so that they can be used across resets
      // DEBUG_SERIAL << F("> clearing on-chip EEPROM ...") << endl;
#ifdef __SAM3X8E__
#else
      for (uint32_t j = 10; j < 20/*TODO EEPROM.length()*/; j++)
      {
         // if (j % 250 == 0) DEBUG_SERIAL << j << endl;
         setChipEEPROMVal(j, 0xff);
      }
#endif
   }
   else
   {
      // clear the external I2C EEPROM of learned events
      resetEEPROM();
   }

   // DEBUG_SERIAL << F("> setting SLiM config") << endl;

   // set the node identity defaults
   // we set a NN and CANID of zero in SLiM as we're now a consumer-only node

   writeEEPROM(0, 0); // SLiM
   writeEEPROM(1, 0); // CANID
   writeEEPROM(2, 0); // NN hi
   writeEEPROM(3, 0); // NN lo
   setResetFlag();    // set reset indicator

   // zero NVs (NVs number from one, not zero)
   for (uint8_t i = 0; i < EE_NUM_NVS; i++)
   {
      writeNV(i + 1, 0);
   }

   // DEBUG_SERIAL << F("> complete in ") << (millis() - t) << F(", rebooting ... ") << endl;

   // reset complete
   reboot();
}

//
//
/// load node identity from EEPROM
//

void CBUSConfig::loadNVs(void)
{

   FLiM = readEEPROM(0);
   CANID = readEEPROM(1);
   nodeNum = (readEEPROM(2) << 8) + readEEPROM(3);
   return;
}

//
/// check whether there is a collision for any hash in the event hash table
//

bool CBUSConfig::check_hash_collisions(void)
{

   for (uint8_t i = 0; i < EE_MAX_EVENTS - 1; i++)
   {
      for (uint8_t j = i + 1; j < EE_MAX_EVENTS; j++)
      {
         if (evhashtbl[i] == evhashtbl[j] && evhashtbl[i] != 0)
         {
            // DEBUG_SERIAL << F("> hash collision detected, val = ") << evhashtbl[i] << endl;
            // return when first collision detected
            return true;
         }
      }
   }

   return false;
}

//
/// architecture-neutral methods to read and write the microcontroller's on-chip EEPROM (or emulation)
/// as EEPROM.h is not available for all, and a post-write commit may or may not be required
//

void CBUSConfig::setChipEEPROMVal(uint32_t eeaddress, uint8_t val)
{
#ifdef __SAM3X8E__
   dueFlashStorage.write(eeaddress, val);
#else
   // TODO EEPROM.write(eeaddress, val);
#endif

#if defined ESP32 || defined ESP8266 || defined ARDUINO_ARCH_RP2040
   EEPROM.commit();
#endif
}

///

uint8_t CBUSConfig::getChipEEPROMVal(uint32_t eeaddress)
{

#ifdef __SAM3X8E__
   return dueFlashStorage.read(eeaddress);
#else
   return 0;//EEPROM.read(eeaddress);
#endif
}

//
/// a group of methods to get and set the reset flag
/// the resetModule method writes the value 99 to EEPROM address 5 when a module reset has been performed
/// this can be tested at module startup for e.g. setting default NVs or creating producer events
// ƒ

void CBUSConfig::setResetFlag(void)
{

   writeEEPROM(5, 99);
}

void CBUSConfig::clearResetFlag(void)
{

   writeEEPROM(5, 0);
}

bool CBUSConfig::isResetFlagSet(void)
{

   return (readEEPROM(5) == 99);
}

//
/// flash routines for AVR-Dx devices
/// we allocate 2048 bytes at the far end of flash, and cache the data of one of four 512 uint8_t pages (0-3)
/// dirty pages must be erased and written back before moving on
//

// #ifdef __AVR_XMEGA__
#if defined(DXCORE)
// cache a page of flash into memory

void flash_cache_page(const uint8_t page)
{

   const uint32_t address_base = FLASH_AREA_BASE_ADDRESS + (page * FLASH_PAGE_SIZE);

   // DEBUG_SERIAL << F("> flash_cache_page, page = ") << page << endl;

   for (uint32_t a = 0; a < FLASH_PAGE_SIZE; a++)
   {
      cache_page.data[a] = Flash.readByte(address_base + a);
   }

   cache_page.dirty = false;
   return;
}

// write out a cached page to flash

bool flash_writeback_page(const uint8_t page)
{

   bool ret = true;
   uint32_t address;

   // DEBUG_SERIAL << F("> flash_writeback_page, page = ") << curr_page_num << F(", dirty = ") << cache_page.dirty << endl;

   if (cache_page.dirty)
   {
      address = FLASH_AREA_BASE_ADDRESS + (FLASH_PAGE_SIZE * page);

      // erase the existing page of flash memory
      ret = Flash.erasePage(address, 1);

      if (ret != FLASHWRITE_OK)
      {
         // DEBUG_SERIAL.printf(F("error erasing flash page\r\n"));
      }

      // write the nexw data
      ret = Flash.writeBytes(address, cache_page.data, FLASH_PAGE_SIZE);

      if (ret != FLASHWRITE_OK)
      {
         // DEBUG_SERIAL.printf(F("error writing flash data\r\n"));
      }

      cache_page.dirty = false;
   }

   return ret;
}

// write one or more bytes into the page cache, handling crossing a page boundary
// address is the index into the flash area (0-2047), not the absolute memory address

bool flash_write_bytes(const uint16_t address, const uint8_t *data, const uint16_t number)
{

   bool ret = true;

   // DEBUG_SERIAL << F("> flash_write_bytes: address = ") << address << F(", data = ") << *data << F(", length = ") << length << endl;

   if (address >= (FLASH_PAGE_SIZE * NUM_FLASH_PAGES))
   {
      // DEBUG_SERIAL.printf(F("cache page address = %u is out of bounds\r\n"), address);
   }

   // calculate page number, 0-3
   uint8_t new_page_num = address / FLASH_PAGE_SIZE;

   // DEBUG_SERIAL << F("> curr page = ") << curr_page_num << F(", new = ") << new_page_num << endl;

   // erase and write back current page if cache page number is changing
   if (new_page_num != curr_page_num)
   {
      flash_writeback_page(curr_page_num); // write back current cache page
      curr_page_num = new_page_num;        // set new cache page num
      flash_cache_page(curr_page_num);     // cache the new page data from flash
      cache_page.dirty = false;            // set flag
   }

   // calculate the address offset into the page cache buffer
   uint16_t buffer_index = address % FLASH_PAGE_SIZE;

   // write data into cached page buffer
   for (uint16_t a = 0; a < number; a++)
   {

      // we are crossing a page boundary ...
      if (buffer_index >= FLASH_PAGE_SIZE)
      {
         // DEBUG_SERIAL.printf(F("crossing page boundary\r\n"));
         cache_page.dirty = true;
         flash_writeback_page(curr_page_num); // write back current cache page
         ++curr_page_num;                     // increment the page number -- should really handle end of flash area
         flash_cache_page(curr_page_num);     // cache the new page data from flash
         buffer_index = 0;                    // set cache buffer index to zero
      }

      cache_page.data[buffer_index + a] = data[a];
   }

   // set page dirty flag
   cache_page.dirty = true;

   // write the cache page to flash
   flash_writeback_page(curr_page_num);

   return ret;
}

// read one uint8_t from flash
// not via cache

uint8_t flash_read_byte(uint32_t address)
{

   return Flash.readByte(address);
}

// read multiple bytes from flash
// address is internal address map offset
// not through cache

void flash_read_bytes(const uint16_t address, const uint16_t number, uint8_t *dest)
{

   uint32_t base_address = FLASH_AREA_BASE_ADDRESS + address;

   for (uint32_t a = 0; a < number; a++)
   {
      dest[a] = flash_read_byte(base_address + a);
   }

   return;
}

#endif
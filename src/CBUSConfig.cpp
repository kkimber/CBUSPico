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

//
/// CBUS module configuration
/// manages the storage of events and node variables in on-chip or external EEPROM
//

#include "CBUSConfig.h"
#include "CBUSLED.h"
#include "CBUSSwitch.h"
#include "SystemTick.h"
#include "CBUSUtil.h"

#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <pico/binary_info.h>
#include <hardware/gpio.h>
#include <hardware/i2c.h>
#include <hardware/flash.h>
#include <hardware/sync.h>
#include <hardware/watchdog.h>

#include <stdlib.h>
#include <string.h>

// For detecting free memory
extern "C" char *sbrk(int incr);

//
/// ctor
//

CBUSConfig::CBUSConfig()
{
   eeprom_type = EEPROM_INTERNAL;
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
      //EEPROM.begin(4096);
   }

   if (eeprom_type == EEPROM_USES_FLASH)
   {
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

      eeprom_type = EEPROM_INTERNAL;
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
}

//
/// store the CANID
//

void CBUSConfig::setCANID(uint8_t canid)
{
   CANID = canid;
   writeEEPROM(1, canid);
}

//
/// store the node number
//

void CBUSConfig::setNodeNum(uint32_t nn)
{
   nodeNum = nn;
   writeEEPROM(2, highByte(nodeNum));
   writeEEPROM(3, lowByte(nodeNum));
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
         i = EE_MAX_EVENTS;
      }
   }

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
}

//
/// clear the hash table
//

void CBUSConfig::clearEvHashTable(void)
{
   // zero in the hash table indicates that the corresponding event slot is free
   for (uint8_t i = 0; i < EE_MAX_EVENTS; i++)
   {
      evhashtbl[i] = 0;
   }

   hash_collision = false;
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
      }

      I2Cbus->requestFrom((int)external_address, (int)nbytes);

      while (I2Cbus->available() && count < nbytes)
      {
         dest[count++] = I2Cbus->read();
      }
#endif
      break;

   case EEPROM_INTERNAL:
      for (count = 0; count < nbytes; count++)
      {
         dest[count] = getChipEEPROMVal(eeaddress + count);
      }
      break;

   case EEPROM_USES_FLASH:
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
      }
#endif      
      break;

   case EEPROM_INTERNAL:
      setChipEEPROMVal(eeaddress, data);
      break;

   case EEPROM_USES_FLASH:
      break;
   }
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
      break;
   }
}

//
/// write (or clear) an event to EEPROM
/// just the first four bytes -- NN and EN
//

void CBUSConfig::writeEvent(uint8_t index, uint8_t data[])
{
   int eeaddress = EE_EVENTS_START + (index * EE_BYTES_PER_EVENT);

   writeBytesEEPROM(eeaddress, data, 4);
}

//
/// clear an event from the table
//

void CBUSConfig::cleareventEEPROM(uint8_t index)
{
   uint8_t unused_entry[4] = {0xff, 0xff, 0xff, 0xff};

   writeEvent(index, unused_entry);
}

//
/// clear all event data in external EEPROM chip
//

void CBUSConfig::resetEEPROM(void)
{
   if (eeprom_type == EEPROM_EXTERNAL)
   {
      for (uint32_t addr = 10; addr < 4096; addr++)
      {
         writeEEPROM(addr, 0xff);
      }
   }
   else if (eeprom_type == EEPROM_USES_FLASH)
   {
   }
}

//
/// reboot the processor
//

void CBUSConfig::reboot(void)
{
   // Set watchdog timeout to 100ms and allow to expire
   watchdog_enable(100, 1); 
   while (1)
      ;
}

//
/// get free RAM
//

uint32_t CBUSConfig::freeSRAM(void)
{
   char top;
   return &top - reinterpret_cast<char *>(sbrk(0));
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

   pbSwitch.reset();
   ledGrn.blink();
   ledYlw.blink();

   // wait for a further (5 sec) button press -- as a 'safety' mechanism
   while (!bDone)
   {
      // 30 sec timeout
      if ((SystemTick::GetMilli() - waittime) > 30000)
      {
         // Timeout expired, reset not performed
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

   ledGrn.off();
   ledYlw.off();
   ledGrn.run();
   ledYlw.run();

   resetModule();
}

void CBUSConfig::resetModule(void)
{
   /// implementation of resetModule() without CBUSswitch or CBUSLEDs

   if (eeprom_type == EEPROM_INTERNAL)
   {
      // clear the entire on-chip EEPROM
      // !! note we don't clear the first ten locations (0-9), so that they can be used across resets
      for (uint32_t j = 10; j < 20/*TODO EEPROM.length()*/; j++)
      {
         setChipEEPROMVal(j, 0xff);
      }
   }
   else
   {
      // clear the external I2C EEPROM of learned events
      resetEEPROM();
   }

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
   // TODO EEPROM.write(eeaddress, val);

   //EEPROM.commit();
}

///

uint8_t CBUSConfig::getChipEEPROMVal(uint32_t eeaddress)
{
   return 0;//EEPROM.read(eeaddress);
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
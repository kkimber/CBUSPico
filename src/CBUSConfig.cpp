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
#include <hardware/sync.h>
#include <hardware/watchdog.h>

#include <new>
#include <cstdlib>
#include <cstring>

// For detecting free memory
extern "C" char *sbrk(int incr);

constexpr uint8_t RESET_FLAG = 99U; ///< Byte value of reset flag, set into EEPROM to indicate module has been reset

constexpr uint8_t OFS_FLIM_MODE = 0U;   ///< Offset of FLiM mode variable in storage
constexpr uint8_t OFS_CAN_ID = 1U;      ///< Offset of CAN ID variable in storage
constexpr uint8_t OFS_NODE_NUM_HB = 2U; ///< Offset of High Byte of Node Number in storage
constexpr uint8_t OFS_NODE_NUM_LB = 3U; ///< Offset of Low Byte of Node Number in storage
constexpr uint8_t OFS_RESET_FLAG = 5U;  ///< Offset of Reset flag variable in storage

constexpr uint8_t DEFAULT_CANID = 1U; ///< Default CAN ID (for SLiM), likely to be modified by conflict resolution
constexpr uint8_t DEFAULT_NN = 0U;    ///< Default Node Number, modules should start with a node number of zero

/// Memory offset of flash in global memory map (flash is memory mapped)
constexpr uint32_t FLASH_BASE = (XIP_BASE + PICO_FLASH_SIZE_BYTES) - FLASH_SECTOR_SIZE;

/// Offset into flash where our data is located (for write)
constexpr uint32_t FLASH_OFFSET = PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE;

/// Size of our image data is 4KiB or one sectors
constexpr uint32_t FLASH_SIZE = FLASH_SECTOR_SIZE;

///
/// @brief Construct a new CBUSConfig::CBUSConfig object
///
CBUSConfig::CBUSConfig() : EE_EVENTS_START{0x0UL},
                           EE_MAX_EVENTS{0x0U},
                           EE_NUM_EVS{0x0U},
                           EE_BYTES_PER_EVENT{0x0U},
                           EE_NVS_START{0x0UL},
                           EE_NUM_NVS{0x0U},
                           m_canId{0x0U},
                           m_bFLiM{false},
                           m_nodeNum{0x0UL},
                           m_intrStatus{0x0UL},
                           m_eepromType{EEPROM_TYPE::EEPROM_USES_FLASH},
                           m_externalAddress{EEPROM_I2C_ADDR},
                           m_i2cBus{i2c_default},
                           m_evhashtbl{nullptr},
                           m_bHashCollisions{false},
                           m_bFlashModified{false},
                           m_bFlashZeroToOne{false},
                           m_flashBuf{}
{
}

///
/// @brief Destroy the CBUSConfig::CBUSConfig object
///        Cleans-up the hash table
///
CBUSConfig::~CBUSConfig()
{
   // Delete any allocated event hash table
   if (m_evhashtbl)
   {
      delete[] m_evhashtbl;
      m_evhashtbl = nullptr;
   }
}

///
/// @brief Disable interrupts and pause the second core
///
void CBUSConfig::disableIRQs()
{
   /// Pause the second core - @todo need flag if single core
   // multicore_lockout_start_blocking();

   // Disable IRQs
   m_intrStatus = save_and_disable_interrupts();
}

///
/// @brief Enable interrups and re-enable the second core
///
void CBUSConfig::enableIRQs()
{
   // Enable IRQs
   restore_interrupts(m_intrStatus);

   /// Resume the second core - @todo need flag if single core
   // multicore_lockout_end_blocking();
}

///
/// @brief initialise and set default values
///
void CBUSConfig::begin()
{
   EE_BYTES_PER_EVENT = EE_NUM_EVS + 4;

   if (m_eepromType == EEPROM_TYPE::EEPROM_USES_FLASH)
   {
      // Read flash into memory cache - flash is memory mapped
      // load the page into the flash buffer - flash is memory mapped
      memcpy(m_flashBuf, reinterpret_cast<void *>(FLASH_BASE), FLASH_SECTOR_SIZE);
   }

   if (m_eepromType == EEPROM_TYPE::EEPROM_EXTERNAL_I2C)
   {
      // Init i2c0 at 100kHz
      i2c_init(m_i2cBus, 100 * 1000);
      gpio_set_function(0, GPIO_FUNC_I2C); // GP0
      gpio_set_function(1, GPIO_FUNC_I2C); // GP1

      // Make the I2C pins available to picotool
      bi_decl(bi_2pins_with_func(0, 1, GPIO_FUNC_I2C));
   }

   // read events and create an event hash table
   makeEvHashTable();

   // read global configuration variables
   loadNVs();
}

///
/// @brief Set the type of storage in use, either QSPI flash or an external I2C EEPROM
///
/// @param type Type of storage to use
/// @return true The storage type was set as requested
/// @return false The storage type could not be set, i.e. valiation of external I2C device failed
///
bool CBUSConfig::setEEPROMtype(EEPROM_TYPE type)
{
   bool ret = true;
   uint8_t tmpByte = 0x0U;

   disableIRQs();

   switch (type)
   {
   case EEPROM_TYPE::EEPROM_EXTERNAL_I2C:

      // Attempt to read offset 0
      if ((i2c_write_blocking(m_i2cBus, m_externalAddress, &tmpByte, 1, true) == 0x01) &&
          (i2c_read_blocking(m_i2cBus, m_externalAddress, &tmpByte, 1, false) == 0x01))
      {
         // Read was OK, so we can use external EEPROM
         m_eepromType = type;
      }
      else
      {
         // Read failed, default to using Flash
         m_eepromType = EEPROM_TYPE::EEPROM_USES_FLASH;
         ret = false;
      }
      break;

   case EEPROM_TYPE::EEPROM_USES_FLASH:

      m_eepromType = EEPROM_TYPE::EEPROM_USES_FLASH;
      break;
   }

   enableIRQs();

   return ret;
}

///
/// @brief Set the address of the external EEPROM devices on the I2C bus
///
/// @param address I2C address of the EEPROM
///
void CBUSConfig::setExtEEPROMAddress(uint8_t address)
{
   m_externalAddress = address;
}

///
/// @brief Store the FLiM mode and cache the value
///
/// @param flim New FLiM mode, true = FLiM, false = SLiM
///
void CBUSConfig::setFLiM(bool flim)
{
   m_bFLiM = flim;
   writeEEPROM(OFS_FLIM_MODE, flim);
}

///
/// @brief Store the CANID and cache the valie
///
/// @param canid New CAN ID for the module
///
void CBUSConfig::setCANID(uint8_t canid)
{
   m_canId = canid;
   writeEEPROM(OFS_CAN_ID, canid);
}

///
/// @brief Store the Node Number and cache the value
///
/// @param nn New Node Number for the module
///
void CBUSConfig::setNodeNum(uint32_t nn)
{
   m_nodeNum = nn;
   writeEEPROM(OFS_NODE_NUM_HB, highByte(nn));
   writeEEPROM(OFS_NODE_NUM_LB, lowByte(nn));
}

///
/// @brief Lookup an event by node number and event number, using the hash table
///
/// @param nn Node Number
/// @param en Event Number
/// @return uint8_t Index of the event, EE_MAX_EVENTS if the event is not found
///
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
      if (m_evhashtbl[i] == tmphash)
      {
         if (!m_bHashCollisions)
         {
            // NN + EN hash matches and there are no hash collisions in the hash table
         }
         else
         {
            // there is a potential hash collision, so we have to check the slower way
            // first, check if this hash appears in the table more than once
            for (j = 0, matches = 0; j < EE_MAX_EVENTS; j++)
            {
               if (m_evhashtbl[j] == tmphash)
               {
                  ++matches;
               }
            }

            if (matches > 1)
            {
               // one or more collisions for this hash exist, so check the very slow way
               for (i = 0; i < EE_MAX_EVENTS; i++)
               {
                  if (m_evhashtbl[i] == tmphash)
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

///
/// @brief Find first empty slot in the Event Table
///
/// @return uint8_t index of the event slot, or EE_MAX_EVENTS if no free slot found
///
uint8_t CBUSConfig::findEventSpace(void)
{
   uint8_t evidx;

   for (evidx = 0; evidx < EE_MAX_EVENTS; evidx++)
   {
      if (m_evhashtbl[evidx] == 0)
      {
         break;
      }
   }

   return evidx;
}

///
/// @brief Create a 8-bit hash from the combination of Node Number and Event Number
///
/// @param tarr Node number in lower two bytes, Event number in upper two bytes
/// @return uint8_t 8-bit has of Node Number and Event Number
///
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

///
/// @brief Read a event from the EEPROM
///
/// @param idx Index of the event to read
/// @param tarr four byte array containing event info (NN + EN)
///
void CBUSConfig::readEvent(uint8_t idx, uint8_t tarr[])
{
   // populate the array with the first 4 bytes (NN + EN) of the event entry from the EEPROM
   for (uint8_t i = 0; i < EE_HASH_BYTES; i++)
   {
      tarr[i] = readEEPROM(EE_EVENTS_START + (idx * EE_BYTES_PER_EVENT) + i);
   }
}

///
/// @brief Read an event variable from the EEPROM
///
/// @param idx Index of the event to read
/// @param evnum Index of the event variable to read
/// @return uint8_t Value of the event variable
///
uint8_t CBUSConfig::getEventEVval(uint8_t idx, uint8_t evnum)
{
   return readEEPROM(EE_EVENTS_START + (idx * EE_BYTES_PER_EVENT) + 3 + evnum);
}

///
/// @brief Write an event variable to the EEPROM
///
/// @param idx Index of the event to write
/// @param evnum Index of the event variable to write
/// @param evval Value of the event variable to write
///
void CBUSConfig::writeEventEV(uint8_t idx, uint8_t evnum, uint8_t evval)
{
   writeEEPROM(EE_EVENTS_START + (idx * EE_BYTES_PER_EVENT) + 3 + evnum, evval);
}

///
/// @brief Rebuild the event hash table
///
///
void CBUSConfig::makeEvHashTable(void)
{
   uint8_t evarray[4];
   const uint8_t unused_entry[4] = {0xff, 0xff, 0xff, 0xff};

   // Delete any previously allocated hash table
   if (m_evhashtbl != nullptr)
   {
      delete[] m_evhashtbl;
   }

   // Allocate the hash table
   m_evhashtbl = new (std::nothrow) uint8_t[EE_MAX_EVENTS];

   if (!m_evhashtbl)
   {
      while (1)
      {
         /// @todo need debug trap for out of memory
      };
   }

   for (uint8_t idx = 0; idx < EE_MAX_EVENTS; idx++)
   {
      readEvent(idx, evarray);

      // empty slots have all four bytes set to 0xff
      if (memcmp(evarray, unused_entry, 4) == 0)
      {
         m_evhashtbl[idx] = 0;
      }
      else
      {
         m_evhashtbl[idx] = makeHash(evarray);
      }
   }

   m_bHashCollisions = check_hash_collisions();
}

///
/// @brief Update an event hash table entry
///
/// @param idx Index of the event to update
///
void CBUSConfig::updateEvHashEntry(uint8_t idx)
{
   uint8_t evarray[4];
   const uint8_t unused_entry[4] = {0xff, 0xff, 0xff, 0xff};

   // read the first four bytes from EEPROM - NN + EN
   readEvent(idx, evarray);

   // empty slots have all four bytes set to 0xff
   if (memcmp(evarray, unused_entry, 4) == 0)
   {
      m_evhashtbl[idx] = 0;
   }
   else
   {
      m_evhashtbl[idx] = makeHash(evarray);
   }

   m_bHashCollisions = check_hash_collisions();
}

////
/// @brief Clear the event hash table
///
void CBUSConfig::clearEvHashTable(void)
{
   // zero in the hash table indicates that the corresponding event slot is free
   for (uint8_t i = 0; i < EE_MAX_EVENTS; i++)
   {
      m_evhashtbl[i] = 0;
   }

   m_bHashCollisions = false;
}

///
/// @brief Retrieve the number of currently configure / stored events
///
/// @return uint8_t Number of stored events
///
uint8_t CBUSConfig::numEvents(void)
{
   uint8_t numevents = 0;

   for (uint8_t i = 0; i < EE_MAX_EVENTS; i++)
   {
      if (m_evhashtbl[i] != 0)
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
      return m_evhashtbl[tindex];
   }
   else
   {
      return 0;
   }
}

///
/// @brief Read a Node Variable from the EEPROM
///
/// @param idx Index of the node variable to read (one based)
/// @return uint8_t value of the node variable
///
uint8_t CBUSConfig::readNV(uint8_t idx)
{
   return (readEEPROM(EE_NVS_START + (idx - 1)));
}

///
/// @brief Write a Node Variable to the EEPROM
///
/// @param idx Index of the node variable to write (one based)
/// @param val Value of the node variable to write
///
void CBUSConfig::writeNV(uint8_t idx, uint8_t val)
{
   writeEEPROM(EE_NVS_START + (idx - 1), val);
}

///
/// @brief Read a single byte from the EEPROM
///
/// @param eeaddress Byte offset address to read
/// @return uint8_t value read from the EEPROM
///
uint8_t CBUSConfig::readEEPROM(uint32_t eeaddress)
{
   uint8_t addr = static_cast<uint8_t>(eeaddress);
   uint8_t rdata = 0U;

   disableIRQs();

   switch (m_eepromType)
   {

   case EEPROM_TYPE::EEPROM_EXTERNAL_I2C:
      i2c_write_blocking(m_i2cBus, m_externalAddress, &addr, 1, true);
      i2c_read_blocking(m_i2cBus, m_externalAddress, &rdata, 1, false);
      break;

   case EEPROM_TYPE::EEPROM_USES_FLASH:
      rdata = getChipEEPROMVal(eeaddress);
      break;
   }

   enableIRQs();

   return rdata;
}

///
/// @brief Read a number of bytes of data from the EEPROM
///
/// @param eeaddress Byte offset address to read
/// @param nbytes Number of bytes to read
/// @param dest Buffer where read data will be placed
/// @return uint8_t Number of bytes read
///
uint8_t CBUSConfig::readBytesEEPROM(uint32_t eeaddress, uint8_t nbytes, uint8_t dest[])
{
   uint8_t addr = static_cast<uint8_t>(eeaddress);
   int r = 0;
   uint8_t count = 0;

   disableIRQs();

   switch (m_eepromType)
   {

   case EEPROM_TYPE::EEPROM_EXTERNAL_I2C:
      i2c_write_blocking(m_i2cBus, m_externalAddress, &addr, 1, true);

      while (count < nbytes)
      {
         i2c_read_blocking(m_i2cBus, m_externalAddress, &dest[count++], 1, false);
         i2c_read_blocking_until(m_i2cBus, m_externalAddress, dest, nbytes, false, make_timeout_time_ms(10));
      }
      break;

   case EEPROM_TYPE::EEPROM_USES_FLASH:
      for (count = 0; count < nbytes; count++)
      {
         dest[count] = getChipEEPROMVal(eeaddress + count);
      }
      break;
   }

   enableIRQs();

   return count;
}

///
/// @brief Write a byte to the EEPROM
///
/// @param eeaddress Byte address of the offset to write
/// @param data Value to write to the EEPROM
///
void CBUSConfig::writeEEPROM(uint32_t eeaddress, uint8_t data)
{
   int r = 0;
   uint8_t txdata[2] = {static_cast<uint8_t>(eeaddress), data};

   disableIRQs();

   switch (m_eepromType)
   {

   case EEPROM_TYPE::EEPROM_EXTERNAL_I2C:
      // Write address and byte - STOP
      i2c_write_blocking(m_i2cBus, m_externalAddress, txdata, 2, false);

      // Wait for write to complete
      sleep_ms(4);

      break;

   case EEPROM_TYPE::EEPROM_USES_FLASH:
      setChipEEPROMVal(eeaddress, data);
      flushToFlash();
      break;
   }

   enableIRQs();
}

///
/// @brief Write a number of bytes to the EEPROM
///
/// @param eeaddress Byte offset of the address to write
/// @param src Bytes to write
/// @param numbytes Number of bytes in src to write
///
void CBUSConfig::writeBytesEEPROM(uint32_t eeaddress, uint8_t src[], uint8_t numbytes)
{
   // *** TODO *** handle greater than 32 bytes -> the Arduino I2C write buffer size
   // max write = EEPROM pagesize - 64 bytes

   int r = 0;

   disableIRQs();

   switch (m_eepromType)
   {
   case EEPROM_TYPE::EEPROM_EXTERNAL_I2C:
#if 0   
      I2Cbus->beginTransmission(m_externalAddress);
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

   case EEPROM_TYPE::EEPROM_USES_FLASH:
      // Update RAM Flash cache
      for (uint8_t i = 0; i < numbytes; i++)
      {
         setChipEEPROMVal(eeaddress + i, src[i]);
      }

      // Flush to flash
      flushToFlash();
      break;
   }

   enableIRQs();
}

///
/// @brief Write an event to the EEPROM
///
/// @param index Index of the event to write
/// @param data event data
///
void CBUSConfig::writeEvent(uint8_t index, uint8_t data[])
{
   int eeaddress = EE_EVENTS_START + (index * EE_BYTES_PER_EVENT);

   writeBytesEEPROM(eeaddress, data, 4);
}

///
/// @brief Clear and event from the EEPROM
///
/// @param index Index of the event to clear
///
void CBUSConfig::cleareventEEPROM(uint8_t index)
{
   uint8_t unused_entry[4] = {0xff, 0xff, 0xff, 0xff};

   writeEvent(index, unused_entry);
}

///
/// @brief  Clear all event data in external EEPROM chip
///
void CBUSConfig::resetEEPROM(void)
{
   if (m_eepromType == EEPROM_TYPE::EEPROM_EXTERNAL_I2C)
   {
      /// @todo need user define for size of EEPROM
      /// currently have 24LC00 fitted, 16 bytes only
      for (uint32_t addr = 10; addr < 16; addr++)
      {
         writeEEPROM(addr, 0xff);
      }
   }
}

///
/// @brief Initiate a reboot of the PICO
///
void CBUSConfig::reboot(void)
{
   // Reset now via the watchdog
   watchdog_reboot(0x0UL, 0x0UL, 0x0UL);
}

///
/// @brief Determine the amount of free memory
///
/// @return uint32_t Free memory
///
uint32_t CBUSConfig::freeSRAM(void)
{
   char top;
   return &top - reinterpret_cast<char *>(sbrk(0));
}

///
/// @brief Reset the module to factory defaults
///
/// @param ledGrn Reference to CBUS Green LED for indicating reset
/// @param ledYlw Reference to CBUS Yellow LED for indicating reset
/// @param pbSwitch Reference to CBUS Switch for monitoring reset request
///
void CBUSConfig::resetModule(CBUSLED &ledGrn, CBUSLED &ledYlw, CBUSSwitch &pbSwitch)
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

///
/// @brief Perform a factory reset of the module
///
void CBUSConfig::resetModule(void)
{
   /// implementation of resetModule() without CBUSswitch or CBUSLEDs

   if (m_eepromType == EEPROM_TYPE::EEPROM_USES_FLASH)
   {
      // Erase all of Flash
      flash_range_erase(FLASH_OFFSET, FLASH_SECTOR_SIZE);
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

   // reset complete, now reboot the module
   reboot();
}

///
/// @brief Load the global node variables from EEPROM
///        Will set defaults if the EEPROM is unitialized
///
void CBUSConfig::loadNVs(void)
{
   /// Detect valid EEPROM data
   uint8_t resetFlag = readEEPROM(OFS_RESET_FLAG);

   if (resetFlag == 0xFF)
   {
      // EEPROM is initialized, default to SLiM with default CAN ID and Node Number
      setFLiM(false);
      setCANID(DEFAULT_CANID);
      setNodeNum(DEFAULT_NN);
      clearResetFlag(); // Clear the reset flag so next boot we use EEPROM values
   }
   else
   {
      // EEPROM previously initialized, read defaults from EEPROM
      m_bFLiM = readEEPROM(OFS_FLIM_MODE);
      m_canId = readEEPROM(OFS_CAN_ID);
      m_nodeNum = (readEEPROM(OFS_NODE_NUM_HB) << 8) + readEEPROM(OFS_NODE_NUM_LB);
   }
}

///
/// @brief Check for any existing hash clashes between different configured events
///
/// @return true There are hash clashes in the hash table
/// @return false There are no hash clashes in the hash table
///
bool CBUSConfig::check_hash_collisions(void)
{
   for (uint8_t i = 0; i < EE_MAX_EVENTS - 1; i++)
   {
      for (uint8_t j = i + 1; j < EE_MAX_EVENTS; j++)
      {
         if (m_evhashtbl[i] == m_evhashtbl[j] && m_evhashtbl[i] != 0)
         {
            // return when first collision detected
            return true;
         }
      }
   }

   return false;
}

///
/// @brief Write a byte value to the RAM flash cache,
///        must be followed up with a call to flushToFlash
///
/// @param eeaddress Address to write to (byte offset) in the flash cache
/// @param val Value to write to the flash cache
///
void CBUSConfig::setChipEEPROMVal(uint32_t eeaddress, uint8_t val)
{
   // Check address is within flash bounds
   if (eeaddress < sizeof(m_flashBuf))
   {
      // Get current value from flash buffer cache
      uint8_t curVal = m_flashBuf[eeaddress];

      // Check if we're changing data
      if (val != curVal)
      {
         m_bFlashModified = true;
      }

      // Check if we're modifying any bits from zero to one (i.e. we need to erase flash)
      if (val & ~curVal)
      {
         m_bFlashZeroToOne = true;
      }

      // Update cache
      m_flashBuf[eeaddress] = val;
   }
}

///
/// @brief Flush RAM cache of Flash to Flash
///
void CBUSConfig::flushToFlash()
{
   // Has flash actually been modified?
   if (m_bFlashModified)
   {
      // Does the modification change bits from zero to one?
      if (m_bFlashZeroToOne)
      {
         // Yes, so we must erase first
         flash_range_erase(FLASH_OFFSET, FLASH_SECTOR_SIZE);
      }

      /// @todo Optimize to only flash required page

      // (Re)program flash
      flash_range_program(FLASH_OFFSET, m_flashBuf, FLASH_SECTOR_SIZE);
   }

   // Reset flags
   m_bFlashModified = false;
   m_bFlashZeroToOne = false;
}

///
/// @brief Read a byte value from the RAM cache of flash data
///
/// @param eeaddress Byte offset into flash data image
/// @return uint8_t value of byte in the flash cache, or 0xFF if eeaddress exceeds flash bounds
///
uint8_t CBUSConfig::getChipEEPROMVal(uint32_t eeaddress)
{
   // Check address is with flash bounds
   if (eeaddress < sizeof(m_flashBuf))
   {
      return m_flashBuf[eeaddress];
   }

   // Read beyond flash boundary
   return 0xFF;
}

//
// A group of methods to get and set the reset flag
// the resetModule method writes a magic RESET_FLAG value to EEPROM address OFS_RESET_FLAG when
// a module reset has been performed, this can be tested at module startup for
// e.g. setting default NVs or creating producer events
//

///
/// @brief Set the reset flag in the EEPROM
///
void CBUSConfig::setResetFlag(void)
{
   writeEEPROM(OFS_RESET_FLAG, RESET_FLAG);
}

///
/// @brief Clear the reset flag in the EEPROM
///
void CBUSConfig::clearResetFlag(void)
{
   writeEEPROM(OFS_RESET_FLAG, 0U);
}

///
/// @brief Determine if the Reset flag is set in the EEPROM
///
/// @return true Reset flag is set
/// @return false Reset flag is not set
///
bool CBUSConfig::isResetFlagSet(void)
{
   return (readEEPROM(OFS_RESET_FLAG) == RESET_FLAG);
}
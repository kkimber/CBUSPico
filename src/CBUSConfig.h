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

#pragma once

#include <cstdint>

#include <hardware/i2c.h>
#include <hardware/flash.h>

// Forward declations
class CBUSLED;
class CBUSSwitch;

// in-memory hash table
constexpr uint8_t EE_HASH_BYTES = 4;
constexpr uint8_t HASH_LENGTH = 128;

/// Default I2C address of the external EEPROM
constexpr uint8_t EEPROM_I2C_ADDR = 0x50;

enum class EEPROM_TYPE
{
   EEPROM_USES_FLASH,  ///< Use Pico QPSI flash as a pseudo EEPROM
   EEPROM_EXTERNAL_I2C ///< Use an external I2C EEPROM
};

//
/// a class to encapsulate CBUS module configuration, events, NVs, EEPROM, etc
//

class CBUSConfig
{

public:
   CBUSConfig();
   ~CBUSConfig();

   // Initialization
   void begin(void);

   // Concurrency support
   void disableIRQs(void);
   void enableIRQs(void);

   // Event management
   uint8_t findExistingEvent(uint32_t nn, uint32_t en);
   uint8_t findEventSpace(void);

   // Event table and hash table management
   uint8_t getEvTableEntry(uint8_t tindex);
   uint8_t numEvents(void);
   uint8_t makeHash(uint8_t tarr[]);
   void getEvArray(uint8_t idx);
   void makeEvHashTable(void);
   void updateEvHashEntry(uint8_t idx);
   void clearEvHashTable(void);
   bool check_hash_collisions(void);
   uint8_t getEventEVval(uint8_t idx, uint8_t evnum);
   void writeEventEV(uint8_t idx, uint8_t evnum, uint8_t evval);

   // Node Variable management
   uint8_t readNV(uint8_t idx);
   void writeNV(uint8_t idx, uint8_t val);
   void loadNVs(void);

   // Event management
   void readEvent(uint8_t idx, uint8_t tarr[]);
   void writeEvent(uint8_t index, uint8_t data[]);
   void cleareventEEPROM(uint8_t index);
   void resetModule(CBUSLED& green, CBUSLED& yellow, CBUSSwitch& sw);
   void resetModule(void);

   // EEPROM support
   uint8_t readEEPROM(uint32_t eeaddress);
   void writeEEPROM(uint32_t eeaddress, uint8_t data);
   uint8_t readBytesEEPROM(uint32_t eeaddress, uint8_t nbytes, uint8_t dest[]);
   void writeBytesEEPROM(uint32_t eeaddress, uint8_t src[], uint8_t numbytes);
   void resetEEPROM(void);

   // CBUS Addressing
   void setCANID(uint8_t canid);
   void setFLiM(bool f);
   void setNodeNum(uint32_t nn);

   // Module reset management
   void setResetFlag(void);
   void clearResetFlag(void);
   bool isResetFlagSet(void);

   // Flash EEPROM emulation
   uint8_t getChipEEPROMVal(uint32_t eeaddress);
   void setChipEEPROMVal(uint32_t eeaddress, uint8_t val);
   void flushToFlash(void);

   // EEPROM addressing
   bool setEEPROMtype(EEPROM_TYPE type);
   void setExtEEPROMAddress(uint8_t address);
   uint32_t freeSRAM(void);
   void reboot(void);

   /// Externally accessed variables @todo should be private with accessors !
   uint32_t EE_EVENTS_START;
   uint8_t EE_MAX_EVENTS;
   uint8_t EE_NUM_EVS;
   uint8_t EE_BYTES_PER_EVENT;
   uint32_t EE_NVS_START;
   uint8_t EE_NUM_NVS;

   uint8_t CANID;
   bool FLiM;
   uint32_t NODE_NUM;

private:
   uint32_t m_intrStatus;
   EEPROM_TYPE m_eepromType;
   uint8_t m_externalAddress;
   i2c_inst_t *m_i2cBus;
   uint8_t *m_evhashtbl;
   bool m_bHashCollisions;
   bool m_bFlashModified;
   bool m_bFlashZeroToOne;
   uint8_t m_flashBuf[FLASH_SECTOR_SIZE];
};
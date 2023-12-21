#pragma once

#include <cstdint>

#include "CBUSLED.h"
#include "CBUSSwitch.h"

// in-memory hash table
static const uint8_t EE_HASH_BYTES = 4;
static const uint8_t HASH_LENGTH = 128;

static const uint8_t EEPROM_I2C_ADDR = 0x50;

enum
{
   EEPROM_INTERNAL = 0,
   EEPROM_EXTERNAL = 1,
   EEPROM_USES_FLASH
};

//
/// a class to encapsulate CBUS module configuration, events, NVs, EEPROM, etc
//

class CBUSConfig
{

public:
   CBUSConfig();
   void begin(void);

   uint8_t findExistingEvent(uint32_t nn, uint32_t en);
   uint8_t findEventSpace(void);

   void printEvHashTable(bool raw);
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

   uint8_t readNV(uint8_t idx);
   void writeNV(uint8_t idx, uint8_t val);
   void loadNVs(void);

   void readEvent(uint8_t idx, uint8_t tarr[]);
   void writeEvent(uint8_t index, uint8_t data[]);
   void cleareventEEPROM(uint8_t index);
   void resetModule(CBUSLED green, CBUSLED yellow, CBUSSwitch sw);
   void resetModule(void);

   uint8_t readEEPROM(uint32_t eeaddress);
   void writeEEPROM(uint32_t eeaddress, uint8_t data);
   uint8_t readBytesEEPROM(uint32_t eeaddress, uint8_t nbytes, uint8_t dest[]);
   void writeBytesEEPROM(uint32_t eeaddress, uint8_t src[], uint8_t numbytes);
   void resetEEPROM(void);

   void setCANID(uint8_t canid);
   void setFLiM(bool f);
   void setNodeNum(uint32_t nn);

   void setResetFlag(void);
   void clearResetFlag(void);
   bool isResetFlagSet(void);

   uint8_t getChipEEPROMVal(uint32_t eeaddress);
   void setChipEEPROMVal(uint32_t eeaddress, uint8_t val);

   bool setEEPROMtype(uint8_t type);
  // void setExtEEPROMAddress(uint8_t address, TwoWire *bus = &Wire);
   uint32_t freeSRAM(void);
   void reboot(void);

   uint32_t EE_EVENTS_START;
   uint8_t EE_MAX_EVENTS;
   uint8_t EE_NUM_EVS;
   uint8_t EE_BYTES_PER_EVENT;
   uint32_t EE_NVS_START;
   uint8_t EE_NUM_NVS;

   uint8_t CANID;
   bool FLiM;
   uint32_t nodeNum;
   uint8_t eeprom_type;
   uint8_t external_address;
  // TwoWire *I2Cbus;
   uint8_t *evhashtbl;
   bool hash_collision;
};
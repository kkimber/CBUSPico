/*
   CBUS Module Library - RasberryPi Pico SDK port
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

//
/// Class to hold a CAN/CBUS frame
//

class CANFrame
{

public:
   /// CAN Frame ID
   uint32_t id = {};
   /// CAN Frame EXT flag
   bool ext = {};
   /// CAN Frame RTR flag
   bool rtr = {};
   /// CAN Frame length
   uint8_t len = {};
   /// CAN Frame raw data
   uint8_t data[8] = {};
};

/// A buffer item type for holding CAN/CBUS frames in the circular buffer

typedef struct
{
   uint32_t _item_insert_time;
   CANFrame _item;
} cbus_frame_buffer_t;

//
/// A circular buffer class for holding CAN/CBUS Messages
//

class CBUSCircularBuffer
{

public:
   explicit CBUSCircularBuffer(uint8_t num_items);
   ~CBUSCircularBuffer();
   CBUSCircularBuffer &operator=(const CBUSCircularBuffer &) = delete; // Delete assignment operator to prevent possible memleak
   CBUSCircularBuffer &operator=(CBUSCircularBuffer &) = delete;
   CBUSCircularBuffer(const CBUSCircularBuffer &) = delete; // Do the same for the default copy constructor
   CBUSCircularBuffer(CBUSCircularBuffer &) = delete;

   bool available(void);
   void put(const CANFrame &cf);
   CANFrame *peek(void);
   CANFrame *get(void);
   uint32_t insert_time(void);
   bool full(void);
   void clear(void);
   bool empty(void);
   uint8_t size(void);
   uint8_t free_slots(void);
   uint32_t puts(void);
   uint32_t gets(void);
   uint8_t hwm(void);
   uint32_t overflows(void);

private:
   bool m_full;
   uint8_t m_head;
   uint8_t m_tail;
   uint8_t m_capacity;
   uint8_t m_size;
   uint8_t m_hwm;
   uint32_t m_puts;
   uint32_t m_gets;
   uint32_t m_overflows;
   cbus_frame_buffer_t *m_buffer;
};

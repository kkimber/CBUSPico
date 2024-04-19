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

// header files

#include "CBUS.h"               // abstract base class
#include "ACAN2040.h"           // header for CAN driver
#include "CBUSCircularBuffer.h" // header for circular buffer of CBUS Frames

// constants

static const uint8_t tx_qsize = 8;           ///< Transmit queue size
static const uint8_t rx_qsize = 32;          ///< Receive queue size
static const uint8_t tx_pin = 12;            ///< Default CAN Tx pin number
static const uint8_t rx_pin = 11;            ///< Default CAN Rx pin number
static const uint32_t CANBITRATE = 125000UL; ///< 125Kb/s - fixed for CBUS

//
// class definitions
//

//
/// @brief An implementation of the abstract base CBUSbased class
/// using the ACAN2040 wrapper of the RP2040 PIO CAN controller
//

class CBUSACAN2040 : public CBUSbase
{

public:
   explicit CBUSACAN2040(CBUSConfig &config);
   virtual ~CBUSACAN2040();

   // these methods are declared virtual in the base class and must be implemented by the derived class
   bool begin(void) override;
   bool available(void) override;
   CANFrame getNextMessage(void) override;
   bool sendMessage(CANFrame &msg, bool rtr = false, bool ext = false, uint8_t priority = DEFAULT_PRIORITY) override; // note default arguments
   void reset(void) override;

   // these methods are specific to this implementation
   // they are not declared or implemented by the base CBUS class
   void setNumBuffers(uint8_t num_rx_buffers, uint8_t _num_tx_buffers = 2);
   void setPins(uint8_t tx_pin, uint8_t rx_pin);
   void notify_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *amsg);

   // Override base class implementation
   bool validateNV(const uint8_t NVindex, const uint8_t oldValue, const uint8_t NVvalue) override;

   static bool sendCANMessage(CANFrame &msg);

   /// Static pointer to ACAN2040 class to manage CAN connection
   inline static ACAN2040 *acan2040 = nullptr;
   
   CBUSCircularBuffer *tx_buffer;
   CBUSCircularBuffer *rx_buffer;

private:
   void initMembers(void);
   uint8_t _gpio_tx;
   uint8_t _gpio_rx;
   uint8_t _num_tx_buffers;
   uint8_t _num_rx_buffers;
};

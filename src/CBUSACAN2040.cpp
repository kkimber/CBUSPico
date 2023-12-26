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

// CBUS device-specific library
#include "CBUSACAN2040.h"
#include "SystemTick.h"

#include <new>
#include <cstdlib>
#include <cstring>

#include <RP2040.h>

// static pointer to object
CBUSACAN2040 *acan2040p;

// static callback function
static void cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
{
   acan2040p->notify_cb(cd, notify, msg);
}

//
/// constructor and destructor
//

CBUSACAN2040::CBUSACAN2040(CBUSConfig &config) : CBUSbase(config),
                                                 acan2040{nullptr},
                                                 tx_buffer{nullptr},
                                                 rx_buffer{nullptr},
                                                 _gpio_tx{0x0U},
                                                 _gpio_rx{0x0U},
                                                 _num_tx_buffers{tx_qsize},
                                                 _num_rx_buffers{rx_qsize}
{
   initMembers();
}

void CBUSACAN2040::initMembers(void)
{
   acan2040p = this;
}

CBUSACAN2040::~CBUSACAN2040()
{
   if (rx_buffer)
   {
      delete rx_buffer;
      rx_buffer = nullptr;
   }

   if (tx_buffer)
   {
      delete tx_buffer;
      tx_buffer = nullptr;
   }

   if (acan2040)
   {
      delete acan2040;
      acan2040 = nullptr;
   }
}

//
/// initialise the CAN controller and buffers, and attach the ISR
/// default poll arg is set to false, so as not to break existing code
//

bool CBUSACAN2040::begin()
{
   // allocate tx and tx buffers -- tx is currently unused
   rx_buffer = new (std::nothrow) CBUSCircularBuffer(_num_rx_buffers);
   tx_buffer = new (std::nothrow) CBUSCircularBuffer(_num_tx_buffers);

   acan2040 = new ACAN2040(0, _gpio_tx, _gpio_rx, CANBITRATE, SystemCoreClock, cb);

   if (!rx_buffer || !tx_buffer || !acan2040)
   {
      return false;
   }

   acan2040->begin();

   return true;
}

//
/// callback
//

//
/// check for one or more messages in the receive buffer
//

bool CBUSACAN2040::available(void)
{
   if (!rx_buffer)
   {
      return false;
   }

   return rx_buffer->available();
}

//
/// must call available() first to ensure a message is available in the buffer
//

CANFrame CBUSACAN2040::getNextMessage(void)
{
   CANFrame cf;
   
   if (!rx_buffer)
   {
      return cf;
   }
   
   cf = *rx_buffer->get();

   ++m_numMsgsRcvd;

   return cf;
}

//
/// callback
//

void CBUSACAN2040::notify_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *amsg)
{
   (void)(cd);

   switch (notify)
   {
   case CAN2040_NOTIFY_RX:
      // Serial.printf("acan2040 cb: message received\n");

      m_msg.id = amsg->id;
      m_msg.len = amsg->dlc;

      for (uint8_t i = 0; i < m_msg.len && i < 8; i++)
      {
         m_msg.data[i] = amsg->data[i];
      }

      m_msg.rtr = amsg->id & CAN2040_ID_RTR;
      m_msg.ext = amsg->id & CAN2040_ID_EFF;

      if (rx_buffer)
      {
         rx_buffer->put(&m_msg);
      }
      break;

   case CAN2040_NOTIFY_TX:
      // Serial.printf("acan2040 cb: message sent ok\n");
      break;
   case CAN2040_NOTIFY_ERROR:
      // Serial.printf("acan2040 cb: an error occurred\n");
      break;
   default:
      // Serial.printf("acan2040 cb: unknown event type\n");
      break;
   }
}

//
/// send a CBUS message
//

bool CBUSACAN2040::sendMessage(CANFrame *msg, bool rtr, bool ext, uint8_t priority)
{
   struct can2040_msg tx_msg;

   // caller must populate the message data
   // this method will create the correct frame header (CAN ID and priority bits)
   // rtr and ext default to false unless arguments are supplied - see method definition in .h
   // priority defaults to 1011 low/medium

   if (!acan2040->ok_to_send())
   {
      // Serial.print("no space available to send message");
      return false;
   }

   makeHeader(msg, priority); // default priority unless user overrides

   if (rtr)
      msg->id |= 0x40000000;

   if (ext)
      msg->id |= 0x80000000;

   tx_msg.id = msg->id;
   tx_msg.dlc = msg->len;

   for (uint8_t i = 0; i < msg->len && i < 8; i++)
   {
      tx_msg.data[i] = msg->data[i];
   }

   if (acan2040->send_message(&tx_msg))
   {
      return true;
   }
   else
   {
      return false;
   }
}

//
/// reset the CAN transceiver
//

void CBUSACAN2040::reset(void)
{
   if (rx_buffer)
   {
      delete rx_buffer;
      rx_buffer = nullptr;
   }

   if (rx_buffer)
   {
      delete tx_buffer;
      tx_buffer = nullptr;
   }
   
   if (acan2040)
   {
      delete acan2040;
      acan2040 = nullptr;
   }

   begin();
}

//
/// set the CS and interrupt pins - option to override defaults
/// used as CANL and CANH in this library
//

void CBUSACAN2040::setPins(uint8_t gpio_tx, uint8_t gpio_rx)
{
   _gpio_tx = gpio_tx;
   _gpio_rx = gpio_rx;
}

//
/// set the number of CAN frame receive buffers
/// this can be tuned according to bus load and available memory
//

void CBUSACAN2040::setNumBuffers(uint8_t num_rx_buffers, uint8_t num_tx_buffers)
{
   _num_rx_buffers = num_rx_buffers;
   _num_tx_buffers = num_tx_buffers;
}
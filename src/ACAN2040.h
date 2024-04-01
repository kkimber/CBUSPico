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

// Pico SDK headers - supplied with arduino-pico core
#include <RP2040.h>                     // hw_set_bits
#include <hardware/regs/dreq.h>         // DREQ_PIO0_RX1
#include <hardware/structs/dma.h>       // dma_hw
#include <hardware/structs/iobank0.h>   // iobank0_hw
#include <hardware/structs/padsbank0.h> // padsbank0_hw
#include <hardware/structs/pio.h>       // pio0_hw
#include <hardware/structs/resets.h>    // RESETS_RESET_PIO0_BITS

// additional SDK header required for successful compilation
#include <hardware/irq.h>

// can2040 header
extern "C"
{
#include "can2040.h"
}

/// Library class that wraps the CAN2040 code

class ACAN2040
{

public:
   ACAN2040(uint32_t pio_num, uint32_t gpio_tx, uint32_t gpio_rx, uint32_t bitrate, uint32_t sys_clock, can2040_rx_cb callback);
   void begin();
   void stop();
   bool send_message(struct can2040_msg *msg);
   bool ok_to_send(void);
   void get_statistics(struct can2040_stats *can_stats);

private:
   uint32_t m_pio_num;       ///< PIO instance to use for the CAN2040
   uint32_t m_bitrate;       ///< CAN bitrate - fixed at 125kbps for CBUS
   uint32_t m_gpio_tx;       ///< PICO pin number for CAN Tx
   uint32_t m_gpio_rx;       ///< PICO pin number for CAN Rx
   uint32_t m_sys_clock;     ///< Clock rate of the PICO
   struct can2040 m_cbus;    ///< Struct to manage the CAN2040 instance
   can2040_rx_cb m_callback; ///< Callback for CAN2040 to notify us of frame receipt, transmit complete, errors etc.
};
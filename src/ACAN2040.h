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

/// Arduino library class that wraps the can2040 Kevin's code

class ACAN2040
{

public:
   ACAN2040(uint32_t pio_num, uint32_t gpio_tx, uint32_t gpio_rx, uint32_t bitrate, uint32_t sys_clock, void (*callback)(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg));
   void begin();
   void stop();
   bool send_message(struct can2040_msg *msg);
   bool ok_to_send(void);
   void get_statistics(struct can2040_stats *can_stats);

private:
   uint32_t m_pio_num;
   uint32_t m_bitrate;
   uint32_t m_gpio_tx;
   uint32_t m_gpio_rx;
   uint32_t m_sys_clock;
   struct can2040 m_cbus;
   void (*m_callback)(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg);
};
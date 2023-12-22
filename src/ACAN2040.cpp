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

#include <ACAN2040.h>

struct can2040 *_cbusp;

static void PIOx_IRQHandler(void)
{
   can2040_pio_irq_handler(_cbusp);
}

ACAN2040::ACAN2040(uint32_t pio_num,
                   uint32_t gpio_tx,
                   uint32_t gpio_rx,
                   uint32_t bitrate,
                   uint32_t sys_clock,
                   void (*callback)(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)) : m_pio_num{pio_num},
                                                                                                     m_bitrate{bitrate},
                                                                                                     m_gpio_tx{gpio_tx},
                                                                                                     m_gpio_rx{gpio_rx},
                                                                                                     m_sys_clock{sys_clock},
                                                                                                     m_cbus{},
                                                                                                     m_callback(callback)
{
   _cbusp = &m_cbus;
}

/// implementation of ACAN2040 class

void ACAN2040::begin()
{
   // setup canbus
   can2040_setup(&m_cbus, m_pio_num);
   can2040_callback_config(&m_cbus, m_callback);

   // enable irqs
   irq_set_exclusive_handler(PIO0_IRQ_0_IRQn, PIOx_IRQHandler);
   NVIC_SetPriority(PIO0_IRQ_0_IRQn, 1);
   NVIC_EnableIRQ(PIO0_IRQ_0_IRQn);

   // start canbus
   can2040_start(&m_cbus, m_sys_clock, m_bitrate, m_gpio_rx, m_gpio_tx);
}

bool ACAN2040::send_message(struct can2040_msg *msg)
{
   int ret = can2040_transmit(&m_cbus, msg);
   return (ret == 0);
}

bool ACAN2040::ok_to_send(void)
{
   return can2040_check_transmit(&m_cbus);
}

void ACAN2040::stop(void)
{
   can2040_stop(_cbusp);
}

void ACAN2040::get_statistics(struct can2040_stats *can_stats)
{
   can2040_get_statistics(_cbusp, can_stats);
}
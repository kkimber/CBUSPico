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

#include <ACAN2040.h>

struct can2040 *_cbusp;

///
/// @brief PIO IRQ ISR - locate in RAM
///        Notify CAN2040 of the interrupt
///
static void __attribute__((section(".RAM"))) PIOx_IRQHandler(void)
{
   can2040_pio_irq_handler(_cbusp);
}


///
/// @brief Construct a new ACAN2040::ACAN2040 object
/// 
/// @param pio_num PIO instance to use [0 or 1]
/// @param gpio_tx PICO pin number for the tx line
/// @param gpio_rx PICO pin number for the rx line
/// @param bitrate bit rate to use for CAN (in bits per second)
/// @param sys_clock Clock frequency of the PICO
/// @param callback Callback for CAN2040 to use to notify us of CAN Rx, Tx complete and errors
///
ACAN2040::ACAN2040(uint32_t pio_num,
                   uint32_t gpio_tx,
                   uint32_t gpio_rx,
                   uint32_t bitrate,
                   uint32_t sys_clock,
                   can2040_rx_cb callback) : m_pio_num{pio_num},
                                             m_bitrate{bitrate},
                                             m_gpio_tx{gpio_tx},
                                             m_gpio_rx{gpio_rx},
                                             m_sys_clock{sys_clock},
                                             m_cbus{},
                                             m_callback(callback)
{
   _cbusp = &m_cbus;
}

///
/// @brief Initialize and start the CAN2040 instance
/// 
///
void ACAN2040::begin()
{
   // setup canbus
   can2040_setup(&m_cbus, m_pio_num);
   can2040_callback_config(&m_cbus, m_callback);

   // enable irqs based on selected PIO
   if (m_pio_num == 0)
   {
      irq_set_exclusive_handler(PIO0_IRQ_0_IRQn, PIOx_IRQHandler);
      NVIC_SetPriority(PIO0_IRQ_0_IRQn, 1);
      NVIC_EnableIRQ(PIO0_IRQ_0_IRQn);
   }
   else
   {
      irq_set_exclusive_handler(PIO1_IRQ_0_IRQn, PIOx_IRQHandler);
      NVIC_SetPriority(PIO1_IRQ_0_IRQn, 1);
      NVIC_EnableIRQ(PIO1_IRQ_0_IRQn);
   }

   // start canbus
   can2040_start(&m_cbus, m_sys_clock, m_bitrate, m_gpio_rx, m_gpio_tx);
}

///
/// @brief Transmit a frame through the CAN2040 instance
/// 
/// @param msg Pointer to CAN frame to transmit
/// @return true Successful initiation of frame transmit
/// @return false Error sending frame, e.g. Tx FIFO full
///
bool ACAN2040::send_message(struct can2040_msg *msg)
{
   int ret = can2040_transmit(&m_cbus, msg);
   return (ret == 0);
}

///
/// @brief Check if its OK to initiate a frame transmission
/// 
/// @return true CAN2040 is ready to accept a frame for transmission
/// @return false CAN2040 is not ready, e.g. Tx FIFO is full
///
bool ACAN2040::ok_to_send(void)
{
   return can2040_check_transmit(&m_cbus);
}

///
/// @brief Stop CAN2040 from processing
/// 
void ACAN2040::stop(void)
{
   can2040_stop(&m_cbus);
}

///
/// @brief Get frame statistics from the CAN2040 controller instance
/// 
/// @param can_stats Pointer to a can2040_stats struct where data will be returned
///
void ACAN2040::get_statistics(struct can2040_stats *can_stats)
{
   can2040_get_statistics(&m_cbus, can_stats);
}
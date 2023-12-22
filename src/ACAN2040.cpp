// this library header
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
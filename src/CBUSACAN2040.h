#pragma once

// header files

#include <CBUS.h>               // abstract base class
#include <ACAN2040.h>           // header for CAN driver

// constants

static const uint8_t tx_qsize = 8;
static const uint8_t rx_qsize = 32;
static const uint8_t txpin = 1;
static const uint8_t rx_pin = 2;
static const uint32_t CANBITRATE = 125000UL;                // 125Kb/s - fixed for CBUS

/// class definitions

// buffer item type

typedef struct _buffer_entry {
  unsigned long _item_insert_time;
  CANFrame _item;
} buffer_entry_t;

//
/// a circular buffer class
//

class circular_buffer {

public:
  circular_buffer(uint8_t num_items);
  ~circular_buffer();
  bool available(void);
  void put(const CANFrame *cf);
  CANFrame *peek(void);
  CANFrame *get(void);
  unsigned long insert_time(void);
  bool full(void);
  void clear(void);
  bool empty(void);
  uint8_t size(void);
  uint8_t free_slots(void);
  unsigned int puts();
  unsigned int gets();
  uint8_t hwm(void);
  unsigned int overflows(void);

private:
  bool _full;
  uint8_t _head, _tail, _capacity, _size, _hwm;
  unsigned int _puts, _gets, _overflows;
  buffer_entry_t *_buffer;
};

//
/// an implementation of the abstract base CBUS class
/// using the ACAN2040 wrapper of the PIO CAN controller
//

class CBUSACAN2040 : public CBUSbase {

public:

  CBUSACAN2040();
  CBUSACAN2040(CBUSConfig *the_config);
  ~CBUSACAN2040();

  // these methods are declared virtual in the base class and must be implemented by the derived class
  bool begin();
  bool available(void);
  CANFrame getNextMessage(void);
  bool sendMessage(CANFrame * msg, bool rtr = false, bool ext = false, uint8_t priority = DEFAULT_PRIORITY);   // note default arguments
  void reset(void);

  // these methods are specific to this implementation
  // they are not declared or implemented by the base CBUS class
  void setNumBuffers(uint8_t num_rx_buffers, uint8_t _num_tx_buffers = 2);
  void setPins(uint8_t tx_pin, uint8_t rx_pin);
  void printStatus(void);
  void notify_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *amsg);

  ACAN2040 *acan2040;
  circular_buffer *tx_buffer, *rx_buffer;

private:
  void initMembers(void);
  uint8_t _gpio_tx, _gpio_rx;
  uint8_t _num_tx_buffers, _num_rx_buffers;
};

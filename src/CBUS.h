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

#pragma once

#include <cstddef>
#include <cstdint>

#include "CBUSLED.h"
#include "CBUSSwitch.h"
#include "CBUSConfig.h"

#include <cbusdefs.h>

#define SW_TR_HOLD 6000U                  // CBUS push button hold time for SLiM/FLiM transition in millis = 6 seconds
#define DEFAULT_PRIORITY 0xB              // default CBUS messages priority. 1011 = 2|3 = normal/low
#define LONG_MESSAGE_DEFAULT_DELAY 20     // delay in milliseconds between sending successive long message fragments
#define LONG_MESSAGE_RECEIVE_TIMEOUT 5000 // timeout waiting for next long message packet
#define NUM_EX_CONTEXTS 4                 // number of send and receive contexts for extended implementation = number of concurrent messages
#define EX_BUFFER_LEN 64                  // size of extended send and receive buffers

//
/// CBUS modes
//

enum
{
   MODE_SLIM = 0,
   MODE_FLIM = 1,
   MODE_CHANGING = 2
};

//
/// CBUS long message status codes
//

enum
{
   CBUS_LONG_MESSAGE_INCOMPLETE = 0,
   CBUS_LONG_MESSAGE_COMPLETE,
   CBUS_LONG_MESSAGE_SEQUENCE_ERROR,
   CBUS_LONG_MESSAGE_TIMEOUT_ERROR,
   CBUS_LONG_MESSAGE_CRC_ERROR,
   CBUS_LONG_MESSAGE_TRUNCATED
};

//
/// CAN/CBUS message type
//

class CANFrame
{

public:
   uint32_t id;
   bool ext;
   bool rtr;
   uint8_t len;
   uint8_t data[8] = {};
};

//
/// an abstract class to encapsulate CAN bus and CBUS processing
/// it must be implemented by a derived subclass
//

// forward references
class CBUSLongMessage;
class CBUScoe;

class CBUSbase
{

public:
   CBUSbase(CBUSConfig &config);

   // these methods are pure virtual and must be implemented by the derived class
   // as a consequence, it is not possible to create an instance of this class

   virtual bool begin(void) = 0;
   virtual bool available(void) = 0;
   virtual CANFrame getNextMessage(void) = 0;
   virtual bool sendMessage(CANFrame *msg, bool rtr = false, bool ext = false, uint8_t priority = DEFAULT_PRIORITY) = 0;
   virtual void reset(void) = 0;

   // implementations of these methods are provided in the base class

   bool sendWRACK(void);
   bool sendCMDERR(uint8_t cerrno);
   void CANenumeration(void);
   uint8_t getCANID(uint32_t header);
   bool isExt(CANFrame *msg);
   bool isRTR(CANFrame *msg);
   void process(uint8_t num_messages = 3);
   void initFLiM(void);
   void revertSLiM(void);
   void setSLiM(void);
   void renegotiate(void);
   void setParams(unsigned char *mparams);
   void setName(unsigned char *mname);
   void checkCANenum(void);
   void indicateMode(uint8_t mode);
   void setEventHandler(void (*fptr)(uint8_t index, CANFrame *msg));
   void setEventHandler(void (*fptr)(uint8_t index, CANFrame *msg, bool ison, uint8_t evval));
   void setFrameHandler(void (*fptr)(CANFrame *msg), uint8_t *opcodes = nullptr, uint8_t num_opcodes = 0);
   void makeHeader(CANFrame *msg, uint8_t priority = DEFAULT_PRIORITY);
   void processAccessoryEvent(uint32_t nn, uint32_t en, bool is_on_event);

   void setLongMessageHandler(CBUSLongMessage *handler);
   void consumeOwnEvents(CBUScoe *coe);

   void getCBUSUIObjects(CBUSSwitch& sw, CBUSLED& ledGrn, CBUSLED& ledYlw);

   uint32_t _numMsgsSent, _numMsgsRcvd;

protected: // protected members become private in derived classes
   CANFrame _msg;
   CBUSLED _ledGrn;
   CBUSLED _ledYlw;
   CBUSSwitch _sw;
   CBUSConfig &module_config;
   unsigned char *_mparams;
   unsigned char *_mname;
   void (*eventhandler)(uint8_t index, CANFrame *msg);
   void (*eventhandlerex)(uint8_t index, CANFrame *msg, bool evOn, uint8_t evVal);
   void (*framehandler)(CANFrame *msg);
   uint8_t *_opcodes;
   uint8_t _num_opcodes;
   uint8_t enum_responses[16]; // 128 bits for storing CAN ID enumeration results
   bool bModeChanging, bCANenum, bLearn;
   uint32_t timeOutTimer, CANenumTime;
   bool enumeration_required;

   CBUSLongMessage *longMessageHandler; // CBUS long message object to receive relevant frames
   CBUScoe *coe_obj;                    // consume-own-events
};

//
/// a basic class to send and receive CBUS long messages per MERG RFC 0005
/// handles a single message, sending and receiving
/// suitable for small microcontrollers with limited memory
//

class CBUSLongMessage
{

public:
   explicit CBUSLongMessage(CBUSbase *cbus_object_ptr);
   bool sendLongMessage(const void *msg, const uint32_t msg_len, const uint8_t stream_id, const uint8_t priority = DEFAULT_PRIORITY);
   void subscribe(uint8_t *stream_ids, const uint8_t num_stream_ids, void *receive_buffer, const uint32_t receive_buffer_len, void (*messagehandler)(void *fragment, const uint32_t fragment_len, const uint8_t stream_id, const uint8_t status));
   bool process(void);
   virtual void processReceivedMessageFragment(const CANFrame *frame);
   bool is_sending(void);
   void setDelay(uint8_t delay_in_millis);
   void setTimeout(uint32_t timeout_in_millis);

protected:
   bool sendMessageFragment(CANFrame *frame, const uint8_t priority);

   bool _is_receiving = false;
   uint8_t *_send_buffer, *_receive_buffer;
   uint8_t _send_stream_id = 0, _receive_stream_id = 0, *_stream_ids = nullptr, _num_stream_ids = 0, _send_priority = DEFAULT_PRIORITY, _msg_delay = LONG_MESSAGE_DEFAULT_DELAY, _sender_canid = 0;
   uint32_t _send_buffer_len = 0, _incoming_message_length = 0, _receive_buffer_len = 0, _receive_buffer_index = 0, _send_buffer_index = 0, _incoming_message_crc = 0,
            _incoming_bytes_received = 0, _receive_timeout = LONG_MESSAGE_RECEIVE_TIMEOUT, _send_sequence_num = 0, _expected_next_receive_sequence_num = 0;
   uint32_t _last_fragment_sent = 0UL, _last_fragment_received = 0UL;

   void (*_messagehandler)(void *fragment, const uint32_t fragment_len, const uint8_t stream_id, const uint8_t status); // user callback function to receive long message fragments
   CBUSbase *_cbus_object_ptr;
};

//// extended support for multiple concurrent long messages

// send and receive contexts

typedef struct _receive_context_t
{
   bool in_use;
   uint8_t receive_stream_id, sender_canid;
   uint8_t *buffer;
   uint32_t receive_buffer_index, incoming_bytes_received, incoming_message_length, expected_next_receive_sequence_num, incoming_message_crc;
   uint32_t last_fragment_received;
} receive_context_t;

typedef struct _send_context_t
{
   bool in_use;
   uint8_t send_stream_id, send_priority, msg_delay;
   uint8_t *buffer;
   uint32_t send_buffer_len, send_buffer_index, send_sequence_num;
   uint32_t last_fragment_sent;
} send_context_t;

//
/// a derived class to extend the base long message class to handle multiple concurrent messages, sending and receiving
//

class CBUSLongMessageEx : public CBUSLongMessage
{

public:
   explicit CBUSLongMessageEx(CBUSbase *cbus_object_ptr)
       : CBUSLongMessage(cbus_object_ptr) {} // derived class constructor calls the base class constructor

   bool allocateContexts(uint8_t num_receive_contexts = NUM_EX_CONTEXTS, uint32_t receive_buffer_len = EX_BUFFER_LEN, uint8_t num_send_contexts = NUM_EX_CONTEXTS);
   bool sendLongMessage(const void *msg, const uint32_t msg_len, const uint8_t stream_id, const uint8_t priority = DEFAULT_PRIORITY);
   bool process(void);
   void subscribe(uint8_t *stream_ids, const uint8_t num_stream_ids, void (*messagehandler)(void *msg, uint32_t msg_len, uint8_t stream_id, uint8_t status));
   void processReceivedMessageFragment(const CANFrame *frame) override;
   uint8_t is_sending(void);
   void use_crc(bool use_crc);

private:
   bool _use_crc = false;
   uint8_t _num_receive_contexts = NUM_EX_CONTEXTS, _num_send_contexts = NUM_EX_CONTEXTS;
   receive_context_t **_receive_context = nullptr;
   send_context_t **_send_context = nullptr;
};

//
/// a circular buffer class
//

// buffer item type

typedef struct _buffer_entry2
{
   uint32_t _item_insert_time;
   CANFrame _item;
} buffer_entry2_t;

//

class circular_buffer2
{

public:
   explicit circular_buffer2(uint8_t num_items);
   ~circular_buffer2();
   bool available(void);
   void put(const CANFrame *cf);
   CANFrame *peek(void);
   CANFrame *get(void);
   uint32_t insert_time(void);
   bool full(void);
   void clear(void);
   bool empty(void);
   uint8_t size(void);
   uint8_t free_slots(void);
   uint32_t puts();
   uint32_t gets();
   uint8_t hwm(void);
   uint32_t overflows(void);

private:
   bool _full;
   uint8_t _head, _tail, _capacity, _size, _hwm;
   uint32_t _puts, _gets, _overflows;
   buffer_entry2_t *_buffer;
};

// consume-own-events class

class CBUScoe
{

public:
   CBUScoe(const uint8_t num_items = 4);
   ~CBUScoe();
   CBUScoe &operator=(const CBUScoe &) = delete; // Delete assignment operator to prevent possible memleak
   CBUScoe(const CBUScoe &) = delete;            // Do the same for the default copy constructor
   void put(const CANFrame *msg);
   CANFrame get(void);
   bool available(void);

private:
   circular_buffer2 *coe_buff;
};

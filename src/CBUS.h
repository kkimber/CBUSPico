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

#include <cstddef>
#include <cstdint>

#include "CBUSLED.h"
#include "CBUSSwitch.h"
#include "CBUSConfig.h"
#include "CBUSParams.h"
#include "CBUSCircularBuffer.h"

#include <cbusdefs.h>

#define SW_TR_HOLD 6000U                  ///< CBUS push button hold time for SLiM/FLiM transition in millis = 6 seconds
#define DEFAULT_PRIORITY 0xB              ///< default CBUS messages priority. 1011 = 2|3 = normal/low
#define LONG_MESSAGE_DEFAULT_DELAY 20     ///< delay in milliseconds between sending successive long message fragments
#define LONG_MESSAGE_RECEIVE_TIMEOUT 5000 ///< timeout waiting for next long message packet
#define NUM_EX_CONTEXTS 4                 ///< number of send and receive contexts for extended implementation = number of concurrent messages
#define EX_BUFFER_LEN 64                  ///< size of extended send and receive buffers

//
/// Enumeration of CBUS modes
//

enum
{
   MODE_SLIM = 0,    ///< Module is in SLiM mode
   MODE_FLIM = 1,    ///< Module is in FLiM mode
   MODE_CHANGING = 2 ///< Module mode is in the process of being changed
};

//
/// Enumeration CBUS long message status codes
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

// forward declations
class CBUSLongMessage;
class CBUScoe;

/// Length of the module name
constexpr uint8_t MODULE_NAME_LEN = 7;

/// Type for holding the Module Name
typedef struct module_name_t
{
   uint8_t byte[MODULE_NAME_LEN];
} module_name_t;

// Callback function definitions

/// Standard event callback type
using eventCallback_t = void (*)(uint8_t index, const CANFrame &msg);

/// Extended event callback type
using eventExCallback_t = void (*)(uint8_t index, const CANFrame &msg, bool ison, uint8_t evval);

/// Frame callback type
using frameCallback_t = void (*)(CANFrame &msg);

/// Long Message callback type
using longMessageCallback_t = void (*)(void *fragment, const uint32_t fragment_len, const uint8_t stream_id, const uint8_t status);

//
/// @brief An abstract class to encapsulate CAN bus and CBUS processing for a CBUS module,
/// it must be implemented by a derived subclass
//

class CBUSbase
{

public:
   CBUSbase(CBUSConfig &config);
   virtual ~CBUSbase() {}; // explict virtual destructor for proper cleanup

   // these methods are pure virtual and must be implemented by the derived class
   // as a consequence, it is not possible to create an instance of this class

   virtual bool begin(void) = 0;
   virtual bool available(void) = 0;
   virtual CANFrame getNextMessage(void) = 0;
   virtual bool sendMessage(CANFrame &msg, bool rtr = false, bool ext = false, uint8_t priority = DEFAULT_PRIORITY) = 0;
   virtual void reset(void) = 0;

   // implementations of these methods are provided in the base class

   bool sendSingleOpc(const uint8_t opc);
   bool sendOpcMyNN(const uint8_t opc, const uint8_t dataLen=0, const uint8_t d1=0, const uint8_t d2=0, const uint8_t d3=0, const uint8_t d4=0, const uint8_t d5=0);
   bool sendOpcNN(const uint8_t opc, const uint16_t nodeId, const uint8_t dataLen=0, const uint8_t d1=0, const uint8_t d2=0, const uint8_t d3=0, const uint8_t d4=0, const uint8_t d5=0);
   bool sendMsgMyNN(CANFrame& frame);
   bool sendMsgNN(CANFrame& frame, const uint16_t nodeId);
   bool sendMyEvent(const uint16_t eventNum, const bool onEvent);
   bool sendEvent(const uint16_t eventNode, const uint16_t eventNum, const bool onEvent);
   bool sendEventWithData(uint16_t eventNode, const uint16_t eventNum, const bool onEvent, const uint8_t dataLen=0, const uint8_t d1=0, const uint8_t d2=0, const uint8_t d3=0);
   bool sendDataEvent(const uint16_t nodeId, const uint8_t d1=0, const uint8_t d2=0, const uint8_t d3=0, const uint8_t d4=0, const uint8_t d5=0);

   bool sendWRACK(void);
   bool sendCMDERR(uint8_t cerrno);
   void CANenumeration(void);
   uint8_t getCANID(uint32_t header);
   bool isExt(const CANFrame &msg) const;
   bool isRTR(const CANFrame &msg)const;
   void process(uint8_t num_messages = 3);
   void initFLiM(void);
   void revertSLiM(void);
   void setSLiM(void);
   void renegotiate(void);
   void setParams(cbusparam_t *mparams);
   void setName(module_name_t *moduleName);
   void checkCANenum(void);
   void indicateMode(uint8_t mode);
   void indicateFLiMMode(bool bFLiM);
   void setEventHandlerCB(eventCallback_t evCallback);
   void setEventHandlerExCB(eventExCallback_t evExCallback);
   void setFrameHandler(frameCallback_t, uint8_t *opcodes = nullptr, uint8_t num_opcodes = 0);
   void makeHeader(CANFrame &msg, uint8_t priority = DEFAULT_PRIORITY);

   void setLongMessageHandler(CBUSLongMessage *handler);
   void consumeOwnEvents(CBUScoe *coe);

   // Application Hooks
   virtual bool validateNV(const uint8_t NVindex, const uint8_t oldValue, const uint8_t NVvalue);
   virtual void actUponNVchange(const uint8_t NVindex, const uint8_t oldValue, const uint8_t NVvalue);

   // Message Parsers
   bool parseCBUSMsg(CANFrame &msg);
   bool parseCBUSEvent(CANFrame &msg);
   bool parseFLiMCmd(CANFrame &msg);

   // Message Processors
   uint8_t getParFlags(void);
   void QNNrespond(void);
   void doRqnpn(const uint8_t index);
   void doNvrd(const uint8_t NVindex);
   void doNvset(const uint8_t NVindex, const uint8_t NVvalue);
   void doRqnp(void);
   void doRqmn(void);
   void doSnn(void);

   void doNnclr(void);
   void doEvlrn(const uint8_t evNum, const uint8_t evVal);
   void doReval(const uint8_t enNum, const uint8_t evNum);
   void doEvuln(void);
   void doReqev(const uint8_t evNum);

   void doNnevn(void);
   void doNerd(void);
   void doNenrd(const uint8_t index);
   void doRqevn(void);

   CBUSLED &getCBUSYellowLED(void);
   CBUSLED &getCBUSGreenLED(void);
   CBUSSwitch &getCBUSSwitch(void);

   uint32_t m_numMsgsSent;
   uint32_t m_numMsgsRcvd;

protected: // protected members become private in derived classes
   CBUSLED m_ledGrn;
   CBUSLED m_ledYlw;
   CBUSSwitch m_sw;
   CBUSConfig &m_moduleConfig;
   cbusparam_t *m_pModuleParams;
   module_name_t *m_pModuleName;
   eventCallback_t eventHandler;
   eventExCallback_t eventHandlerEx;
   frameCallback_t frameHandler;
   uint8_t *m_opcodes;
   uint8_t m_numOpcodes;
   uint8_t m_enumResponses[16]; // 128 bits for storing CAN ID enumeration results
   bool m_bModeChanging;
   bool m_bCANenum;
   bool m_bLearn;
   bool m_bThisNN;
   uint16_t m_nodeNumber;
   uint16_t m_eventNumber;
   uint32_t timeOutTimer;
   uint32_t CANenumTime;
   bool m_bEnumerationRequired;

   CBUSLongMessage *longMessageHandler; // CBUS long message object to receive relevant frames
   CBUScoe *m_coeObj;                   // consume-own-events
};

//
/// @brief A basic class to send and receive CBUS long messages per MERG RFC 0005,
/// handles a single message, sending and receiving,
/// suitable for small microcontrollers with limited memory
//

class CBUSLongMessage
{

public:
   explicit CBUSLongMessage(CBUSbase *cbus_object_ptr);
   bool sendLongMessage(const void *msg, const uint32_t msg_len, const uint8_t stream_id, const uint8_t priority = DEFAULT_PRIORITY);
   void subscribe(uint8_t *stream_ids, const uint8_t num_stream_ids, void *receive_buffer, const uint32_t receive_buffer_len, void (*messagehandler)(void *fragment, const uint32_t fragment_len, const uint8_t stream_id, const uint8_t status));
   bool process(void);
   virtual void processReceivedMessageFragment(const CANFrame &frame);
   bool is_sending(void);
   void setDelay(uint8_t delay_in_millis);
   void setTimeout(uint32_t timeout_in_millis);

protected:
   bool sendMessageFragment(CANFrame &frame, const uint8_t priority);

   bool _is_receiving = false;
   uint8_t *_send_buffer = nullptr;
   uint8_t *_receive_buffer = nullptr;
   uint8_t _send_stream_id = 0;
   uint8_t _receive_stream_id = 0;
   uint8_t *_stream_ids = nullptr;
   uint8_t _num_stream_ids = 0;
   uint8_t _send_priority = DEFAULT_PRIORITY;
   uint8_t _msg_delay = LONG_MESSAGE_DEFAULT_DELAY;
   uint8_t _sender_canid = 0;
   uint32_t _send_buffer_len = 0;
   uint32_t _incoming_message_length = 0;
   uint32_t _receive_buffer_len = 0;
   uint32_t _receive_buffer_index = 0;
   uint32_t _send_buffer_index = 0;
   uint32_t _incoming_message_crc = 0;
   uint32_t _incoming_bytes_received = 0;
   uint32_t _receive_timeout = LONG_MESSAGE_RECEIVE_TIMEOUT;
   uint32_t _send_sequence_num = 0;
   uint32_t _expected_next_receive_sequence_num = 0;
   uint32_t _last_fragment_sent = 0UL;
   uint32_t _last_fragment_received = 0UL;

   longMessageCallback_t _messagehandler;
   //void (*_messagehandler)(void *fragment, const uint32_t fragment_len, const uint8_t stream_id, const uint8_t status) = {}; // user callback function to receive long message fragments
   CBUSbase *_cbus_object_ptr;
};

//// extended support for multiple concurrent long messages

/// Receive context for long messsages

typedef struct _receive_context_t
{
   bool in_use;
   uint8_t receive_stream_id, sender_canid;
   uint8_t *buffer;
   uint32_t receive_buffer_index, incoming_bytes_received, incoming_message_length, expected_next_receive_sequence_num, incoming_message_crc;
   uint32_t last_fragment_received;
} receive_context_t;

/// Send context for long messages

typedef struct _send_context_t
{
   bool in_use;
   uint8_t send_stream_id, send_priority, msg_delay;
   uint8_t *buffer;
   uint32_t send_buffer_len, send_buffer_index, send_sequence_num;
   uint32_t last_fragment_sent;
} send_context_t;

//
/// A derived class to extend the base long message class to handle multiple concurrent messages, sending and receiving
//

class CBUSLongMessageEx : public CBUSLongMessage
{

public:
   explicit CBUSLongMessageEx(CBUSbase *cbus_object_ptr)
       : CBUSLongMessage(cbus_object_ptr) {} // derived class constructor calls the base class constructor
   ~CBUSLongMessageEx();

   bool allocateContexts(uint8_t num_receive_contexts = NUM_EX_CONTEXTS, uint32_t receive_buffer_len = EX_BUFFER_LEN, uint8_t num_send_contexts = NUM_EX_CONTEXTS);
   bool sendLongMessage(const void *msg, const uint32_t msg_len, const uint8_t stream_id, const uint8_t priority = DEFAULT_PRIORITY);
   bool process(void);
   void subscribe(uint8_t *stream_ids, const uint8_t num_stream_ids, void (*messagehandler)(void *msg, uint32_t msg_len, uint8_t stream_id, uint8_t status));
   void processReceivedMessageFragment(const CANFrame &frame) override;
   uint8_t is_sending(void);
   void use_crc(bool use_crc);

private:
   bool _use_crc = false;
   uint8_t _num_receive_contexts = NUM_EX_CONTEXTS, _num_send_contexts = NUM_EX_CONTEXTS;
   receive_context_t **_receive_context = nullptr;
   send_context_t **_send_context = nullptr;
};

//
/// A class to manage consuming-own-CBUS events
//

class CBUScoe
{

public:
   CBUScoe(const uint8_t num_items = 4);
   ~CBUScoe();
   CBUScoe &operator=(const CBUScoe &) = delete; // Delete assignment operator to prevent possible memleak
   CBUScoe &operator=(CBUScoe &) = delete;
   CBUScoe(const CBUScoe &) = delete; // Do the same for the default copy constructor
   CBUScoe(CBUScoe &) = delete;
   void put(const CANFrame &msg);
   CANFrame get(void);
   bool available(void);

private:
   CBUSCircularBuffer *coe_buff;
};

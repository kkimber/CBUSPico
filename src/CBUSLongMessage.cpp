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

//
/// Implementation of CBUS Long Message Support RFC 0005 by Dave McCabe (M4933)
/// Developed by Duncan Greenwood (M5767)
//

#include "CBUS.h"
#include "SystemTick.h"
#include "CBUSUtil.h"

#include <cstdlib>
#include <cstring>

uint16_t crc16(uint8_t *data_p, uint16_t length);
uint32_t crc32(const char *s, size_t n);

//
/// constructor
/// receives a pointer to a CBUS object which provides the CAN message handling capability
//

CBUSLongMessage::CBUSLongMessage(CBUSbase *cbus_object_ptr) : _messagehandler{nullptr},
                                                              _cbus_object_ptr{cbus_object_ptr}
{
   if (_cbus_object_ptr)
   {
      _cbus_object_ptr->setLongMessageHandler(this);
   }
}

//
/// subscribe to a range of stream IDs
//

void CBUSLongMessage::subscribe(uint8_t *stream_ids, const uint8_t num_stream_ids, void *receive_buffer, const uint32_t receive_buff_len, longMessageCallback_t messageHandler)
{
   _stream_ids = stream_ids;
   _num_stream_ids = num_stream_ids;
   _receive_buffer = (uint8_t *)receive_buffer;
   _receive_buffer_len = receive_buff_len;
   _messagehandler = messageHandler;
}

//
/// initiate sending of a long message
/// this method sends the first message - the header packet
/// the remainder of the message is sent in chunks from the process() method
//

bool CBUSLongMessage::sendLongMessage(const void *msg, const uint32_t msg_len, const uint8_t stream_id, const uint8_t priority)
{
   // DEBUG_SERIAL << F("> L: sending message header packet, stream id = ") << stream_id << F(", message length = ") << msg_len << F(", first char = ") << (char)msg[0] << endl;

   if (is_sending())
   {
      // DEBUG_SERIAL << F("> L: ERROR: there is already a message in progress") << endl;
      return false;
   }

   // initialise variables
   _send_buffer = (uint8_t *)msg;
   _send_buffer_len = msg_len;
   _send_stream_id = stream_id;
   _send_priority = priority;
   _send_buffer_index = 0;
   _send_sequence_num = 0;

   // Frame obj for message to send - initializes to zero
   CANFrame frame;

   // send the first fragment which forms the message header
   frame.data[1] = _send_stream_id;    // the unique stream id
   frame.data[2] = _send_sequence_num; // sequence number, 0 = header packet
   frame.data[3] = highByte(msg_len);  // the message length
   frame.data[4] = lowByte(msg_len);
   // CRC - not implemented for lite version
   // flags - 0 = standard data message

   bool ret = sendMessageFragment(frame, _send_priority); // send the header packet
   ++_send_sequence_num;                                   // increment the sending sequence number - it's fine if it wraps around

   // DEBUG_SERIAL << F("> L: message header sent, stream id = ") << _send_stream_id << F(", message length = ") << _send_buffer_len << endl;
   return (ret);
}

//
/// the process method is called regularly from the user's loop function
/// we use this to send the individual fragments of an outgoing message and check the message receive timeout
//

bool CBUSLongMessage::process(void)
{
   bool ret = true;
   uint8_t i;

   /// check receive timeout

   if (_is_receiving && (SystemTick::GetMilli() - _last_fragment_received >= _receive_timeout))
   {
      // DEBUG_SERIAL << F("> L: ERROR: timed out waiting for continuation packet") << endl;
      _messagehandler(_receive_buffer, _receive_buffer_index, _receive_stream_id, CBUS_LONG_MESSAGE_TIMEOUT_ERROR);
      _is_receiving = false;
      _incoming_message_length = 0;
      _incoming_bytes_received = 0;

      // timeout error status is surfaced to the user's handler function
   }

   /// send the next outgoing fragment, after a configurable delay to avoid flooding the bus

   if (_send_buffer_index < _send_buffer_len && (SystemTick::GetMilli() - _last_fragment_sent >= _msg_delay))
   {
      _last_fragment_sent = SystemTick::GetMilli();

      // Frame obj for message to send - initializes to zero
      CANFrame frame;

      frame.data[1] = _send_stream_id;
      frame.data[2] = _send_sequence_num;

      /// only the last fragment is potentially less than 5 bytes long

      for (i = 0; i < 5 && _send_buffer_index < _send_buffer_len; i++)
      {                                                        // for up to 5 bytes of payload
         frame.data[i + 3] = _send_buffer[_send_buffer_index]; // take the next uint8_t
         // DEBUG_SERIAL << F("> L: consumed data uint8_t = ") << (char)_send_buffer[_send_buffer_index] << endl;
         ++_send_buffer_index;
      }

      ret = sendMessageFragment(frame, _send_priority); // send the data packet
      // DEBUG_SERIAL << F("> L: process: sent message fragment, seq = ") << _send_sequence_num << F(", size = ") << i << endl;

      ++_send_sequence_num;
   }

   return ret;
}

//
/// handle a received long message fragment
/// this method is called by the main CBUS object each time a long CBUS message is received (opcode 0xe9)
//

void CBUSLongMessage::processReceivedMessageFragment(const CANFrame &frame)
{
   /// handle a received message fragment

   // DEBUG_SERIAL << F("> L: processing received long message packet, message length = ") << _incoming_message_length << F(", rcvd so far = ") << _incoming_bytes_received << endl;

   _last_fragment_received = SystemTick::GetMilli();

   uint8_t i, j;

   if (!_is_receiving)
   { // not currently receiving a long message

      if (frame.data[2] == 0)
      { // sequence zero = a header packet with start of new stream
         if (frame.data[7] == 0)
         { // flags = 0, standard messages
            for (i = 0; i < _num_stream_ids; i++)
            {
               if (_stream_ids[i] == frame.data[1])
               { // are we subscribed to this stream id ?
                  _is_receiving = true;
                  _receive_stream_id = frame.data[1];
                  _incoming_message_length = (frame.data[3] << 8) + frame.data[4];
                  _incoming_message_crc = (frame.data[5] << 8) + frame.data[6];
                  _incoming_bytes_received = 0;
                  memset(_receive_buffer, 0, _receive_buffer_len);
                  _receive_buffer_index = 0;
                  _expected_next_receive_sequence_num = 0;
                  _sender_canid = (frame.id & 0x7f);
                  // DEBUG_SERIAL << F("> L: received header packet for stream id = ") << _receive_stream_id << F(", message length = ") << _incoming_message_length << F(", user buffer len = ") << _receive_buffer_len << endl;
                  break;
               }
            }
         }
         else
         {
            // DEBUG_SERIAL << F("> L: not handling message with non-zero flags") << endl;
         }
      }
   }
   else
   { // we're part way through receiving a message

      if ((frame.id & 0x7f) == _sender_canid)
      { // it's the same sender CANID

         if (frame.data[1] == _receive_stream_id)
         { // it's the same stream id

            if (frame.data[2] == _expected_next_receive_sequence_num)
            { // and it's the expected sequence id

               // DEBUG_SERIAL << F("> L: received continuation packet, seq = ") << _expected_next_receive_sequence_num << endl;

               // for each of the maximum five payload bytes, up to the total message length and the user buffer length
               for (j = 0; j < 5; j++)
               {

                  _receive_buffer[_receive_buffer_index] = frame.data[j + 3]; // take the next uint8_t
                  ++_receive_buffer_index;                                     // increment the buffer index
                  ++_incoming_bytes_received;
                  // DEBUG_SERIAL << F("> L: processing received data uint8_t = ") << (char)frame.data[j + 3] << endl;

                  // if we have read the entire message
                  if (_incoming_bytes_received >= _incoming_message_length)
                  {
                     // DEBUG_SERIAL << F("> L: bytes processed = ") << _incoming_bytes_received << F(", message data has been fully consumed") << endl;
                     _messagehandler(_receive_buffer, _receive_buffer_index, _receive_stream_id, CBUS_LONG_MESSAGE_COMPLETE);
                     _receive_buffer_index = 0;
                     memset(_receive_buffer, 0, _receive_buffer_len);
                     break;

                     // if the user buffer is full, give the user what we have so far
                  }
                  else if (_receive_buffer_index >= _receive_buffer_len)
                  {
                     // DEBUG_SERIAL << F("> L: user buffer is full") << endl;
                     _messagehandler(_receive_buffer, _receive_buffer_index, _receive_stream_id, CBUS_LONG_MESSAGE_INCOMPLETE);
                     _receive_buffer_index = 0;
                     memset(_receive_buffer, 0, _receive_buffer_len);
                  }
               }
            }
            else
            { // it's the wrong sequence id
               // DEBUG_SERIAL << F("> L: ERROR: expected receive sequence num = ") << _expected_next_receive_sequence_num << F(" but got = ") << frame.data[2] << endl;
               _messagehandler(_receive_buffer, _receive_buffer_index, _receive_stream_id, CBUS_LONG_MESSAGE_SEQUENCE_ERROR);
               _incoming_message_length = 0;
               _incoming_bytes_received = 0;
               _is_receiving = false;
            }
         }
         else
         { // probably another stream in progress - we'll ignore it as we don't support concurrent streams
           // DEBUG_SERIAL << F("> L: ignoring unexpected stream id =") << _receive_stream_id << F(", got = ") << frame.data[2] << endl;
         }
      }
      else
      { // a different sender CANID - ignore the fragment
        // DEBUG_SERIAL << F("> L: ignoring fragment from unexpected CANID") << endl;
      }

   } // it's a continuation fragment

   // the sequence number may wrap from 255 to 0, which is absolutely fine
   ++_expected_next_receive_sequence_num;

   /// finally, once the message has been completely received ...

   if (_incoming_message_length > 0 && _incoming_bytes_received >= _incoming_message_length)
   {
      // DEBUG_SERIAL << F("> L: message is complete") << endl;

      // surface any final fragment to the user's code
      if (_receive_buffer_index > 0)
      {
         // DEBUG_SERIAL << F("> L: surfacing final fragment") << endl;
         _messagehandler(_receive_buffer, _receive_buffer_index, _receive_stream_id, CBUS_LONG_MESSAGE_COMPLETE);
      }

      // get ready for the next long message
      _incoming_message_length = 0;
      _incoming_bytes_received = 0;
      _receive_buffer_index = 0;
      _is_receiving = false;
   }
}

//
/// report progress of sending last long message
/// true = complete, false = in progress, incomplete
/// user code must not start a new message until the previous one has been completely sent
//

bool CBUSLongMessage::is_sending(void)
{
   return (_send_buffer_index < _send_buffer_len);
}

//
/// send next message fragment
//

bool CBUSLongMessage::sendMessageFragment(CANFrame &frame, const uint8_t priority)
{
   // these are common to all messages
   frame.len = 8;
   frame.data[0] = OPC_DTXC;

   return (_cbus_object_ptr->sendMessage(frame, false, false, priority));
}

//
/// set the delay between send fragments, to avoid flooding the bus and other modules
/// overrides the default value
//

void CBUSLongMessage::setDelay(uint8_t delay_in_millis)
{
   _msg_delay = delay_in_millis;
}

//
/// set the receive timeout
/// if an expected next fragment is not received, the user's handler function
/// will be called with the message so far assembled and an error status
/// overrides the default value
//

void CBUSLongMessage::setTimeout(uint32_t timeout_in_millis)
{
   _receive_timeout = timeout_in_millis;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//
//// extended support for multiple concurrent long messages
//

CBUSLongMessageEx::~CBUSLongMessageEx()
{
   for (int i = 0; i < _num_receive_contexts; i++)
   {
      if (_receive_context[i])
      {
         free(_receive_context[i]);
      }
   }

   for (int i = 0; i < _num_send_contexts; i++)
   {
      if (_send_context[i])
      {
         free(_send_context[i]);
      }
   }
}

//
/// allocate memory for receive and send contexts
//

bool CBUSLongMessageEx::allocateContexts(uint8_t num_receive_contexts, uint32_t receive_buffer_len, uint8_t num_send_contexts)
{
   uint8_t i;

   // DEBUG_SERIAL << F("> receive_context_t * = ") << sizeof(receive_context_t *) << F(", receive_context_t = ") << sizeof(receive_context_t) << endl;

   _num_receive_contexts = num_receive_contexts;
   _receive_buffer_len = receive_buffer_len;
   _num_send_contexts = num_send_contexts;

   // allocate receive contexts
   if ((_receive_context = (receive_context_t **)malloc(sizeof(receive_context_t *) * _num_receive_contexts)) == nullptr)
   {
      return false;
   }

   for (i = 0; i < _num_receive_contexts; i++)
   {
      if ((_receive_context[i] = (receive_context_t *)malloc(sizeof(receive_context_t))) == nullptr)
      {
         return false;
      }

      if ((_receive_context[i]->buffer = (uint8_t *)malloc(receive_buffer_len * sizeof(uint8_t))) == nullptr)
      {
         return false;
      }

      _receive_context[i]->in_use = false;
   }

   // allocate send contexts - user code provides the buffer when sending
   if ((_send_context = (send_context_t **)malloc(sizeof(send_context_t *) * _num_send_contexts)) == nullptr)
   {
      return false;
   }

   for (i = 0; i < _num_send_contexts; i++)
   {
      if ((_send_context[i] = (send_context_t *)malloc(sizeof(send_context_t))) == nullptr)
      {
         return false;
      }

      _send_context[i]->in_use = false;
   }

   return true;
}

//
/// initiate sending of a long message
/// this method sends the first message - the header packet
/// the remainder of the message is sent in fragments from the process() method
//

bool CBUSLongMessageEx::sendLongMessage(const void *msg, const uint32_t msg_len, const uint8_t stream_id, const uint8_t priority)
{
   uint8_t i;
   uint16_t msg_crc = 0;
   CANFrame frame;

   // DEBUG_SERIAL << F("> Lex: sending message header packet, stream id = ") << stream_id << F(", message length = ") << msg_len << endl;

   // ensure we aren't already sending a message with this stream ID
   for (i = 0; i < _num_send_contexts; i++)
   {
      if (_send_context[i]->in_use && _send_context[i]->send_stream_id == stream_id)
      {
         // DEBUG_SERIAL << F("> Lex: ERROR: already sending this stream ID") << endl;
         return false;
      }
   }

   // find a free send context
   for (i = 0; i < _num_send_contexts; i++)
   {
      if (!_send_context[i]->in_use)
      {
         break;
      }
   }

   if (i > _num_send_contexts)
   {
      // DEBUG_SERIAL << F("> Lex: ERROR: unable to find free send context") << endl;
      return false;
   }

   // DEBUG_SERIAL << F("> Lex: using send context = ") << i << endl;

   // initialise context
   _send_context[i]->in_use = true;
   // _send_context[i]->buffer = (uint8_t *)msg;
   _send_context[i]->buffer = (uint8_t *)strdup((char *)msg); // copy the message to the send content, will free later
   _send_context[i]->send_buffer_len = msg_len;
   _send_context[i]->send_stream_id = stream_id;
   _send_context[i]->send_priority = priority;
   _send_context[i]->send_buffer_index = 0;

   // calc CRC
   if (_use_crc)
   {
      msg_crc = crc16((uint8_t *)msg, msg_len);
   }

   // send the first fragment which forms the header message
   frame.data[1] = _send_context[i]->send_stream_id;            // the stream id
   frame.data[2] = 0;                                           // sequence number, 0 = header packet
   frame.data[3] = highByte(_send_context[i]->send_buffer_len); // the message length
   frame.data[4] = lowByte(_send_context[i]->send_buffer_len);
   frame.data[5] = highByte(msg_crc); // CRC, zero if not implemented
   frame.data[6] = lowByte(msg_crc);
   frame.data[7] = 0; // flags - 0 = standard data message

   bool ret = sendMessageFragment(frame, _send_context[i]->send_priority); // send the header packet
   _send_context[i]->send_sequence_num = 1;                                 // the next send sequence number - it's fine if it wraps around

   // DEBUG_SERIAL << F("> Lex: message header sent, stream id = ") << stream_id << F(", message length = ") << msg_len << F(", ret = ") << ret << endl;
   return (ret);
}

//
/// the process method is called regularly from the user's loop function
/// we use this to send the individual fragments of any outgoing messages and check the message receive timeouts
//

bool CBUSLongMessageEx::process(void)
{
   bool ret = true;
   uint8_t i;
   CANFrame frame; // Initializes to zero

   static uint8_t context = 0; // we round-robin the context list when sending

   /// check receive timeout for each active context

   for (i = 0; i < _num_receive_contexts; i++)
   {
      if (_receive_context[i]->in_use && (SystemTick::GetMilli() - _receive_context[i]->last_fragment_received >= _receive_timeout))
      {

         // DEBUG_SERIAL << F("> Lex: ERROR: timed out waiting for continuation packet in context = ") << i << F(", timeout = ") << _receive_timeout << endl;
         _messagehandler(_receive_context[i]->buffer, _receive_context[i]->receive_buffer_index, _receive_context[i]->receive_stream_id, CBUS_LONG_MESSAGE_TIMEOUT_ERROR);
         _receive_context[i]->in_use = false;
         // _receive_context[i]->incoming_message_length = 0;
         // _receive_context[i]->incoming_bytes_received = 0;
      }
   }

   /// send the next outgoing fragment from each active context, after a configurable delay to avoid flooding the bus
   /// concurrent streams will be interleaved

   if (_send_context[context]->in_use && SystemTick::GetMilli() - _send_context[context]->last_fragment_sent >= _msg_delay)
   {
      // DEBUG_SERIAL << F("> Lex: processing send context = ") << context << endl;

      frame.data[1] = _send_context[context]->send_stream_id;
      frame.data[2] = _send_context[context]->send_sequence_num;

      /// only the last fragment is potentially less than 5 bytes long

      for (i = 0; i < 5 && _send_context[context]->send_buffer_index < _send_context[context]->send_buffer_len; i++)
      {                                                                                                 // for up to 5 bytes of payload
         frame.data[i + 3] = _send_context[context]->buffer[_send_context[context]->send_buffer_index]; // take the next uint8_t
         // DEBUG_SERIAL << F("> Lex: consumed data uint8_t = ") << (char)_send_context[context]->buffer[_send_context[context]->send_buffer_index] << endl;
         ++_send_context[context]->send_buffer_index;
      }

      ret = sendMessageFragment(frame, _send_context[context]->send_priority); // send the data packet
      // DEBUG_SERIAL << F("> Lex: process: sent message fragment, seq = ") << _send_context[context]->send_sequence_num << F(", size = ") << i << F(", ret  = ") << ret << endl;

      // release context once message content exhausted
      if (_send_context[context]->send_buffer_index >= _send_context[context]->send_buffer_len)
      {
         _send_context[context]->in_use = false;
         _send_context[context]->send_buffer_len = 0;
         free(_send_context[context]->buffer);
         // DEBUG_SERIAL << F("> Lex: message complete, context released") << endl;
      }
      else
      {
         ++_send_context[context]->send_sequence_num;
         _send_context[context]->last_fragment_sent = SystemTick::GetMilli();
      }
   }

   // increment context counter and wrap
   ++context;
   context = (context >= _num_send_contexts) ? 0 : context;
   return ret;
}

//
/// subscribe to a range of stream IDs
//

void CBUSLongMessageEx::subscribe(uint8_t *stream_ids, const uint8_t num_stream_ids, void (*messagehandler)(void *msg, uint32_t msg_len, uint8_t stream_id, uint8_t status))
{
   _stream_ids = stream_ids;
   _num_stream_ids = num_stream_ids;
   _messagehandler = messagehandler;
}

//
/// report state of long message sending
/// returns number of streams currently in progress
//

uint8_t CBUSLongMessageEx::is_sending(void)
{
   uint8_t i, num_streams;

   for (i = 0, num_streams = 0; i < _num_send_contexts; i++)
   {
      if (_send_context[i]->in_use)
      {
         ++num_streams;
      }
   }

   return num_streams;
}

//
/// handle an incoming long message CBUS message fragment
//

void CBUSLongMessageEx::processReceivedMessageFragment(const CANFrame &frame)
{
   uint8_t i;
   uint8_t status;
   uint16_t tmpcrc = 0;

   // DEBUG_SERIAL << F("> Lex: handling incoming message fragment") << endl;
   // DEBUG_SERIAL.flush();

   if (frame.data[2] == 0)
   { // sequence zero = a header packet with start of new stream
      if (frame.data[7] == 0)
      { // flags = 0, standard message

         // DEBUG_SERIAL << F("> Lex: this is a data message header packet") << endl;

         for (i = 0; i < _num_stream_ids; i++)
         {
            if (_stream_ids[i] == frame.data[1])
            { // are we subscribed to this stream id ?

               // DEBUG_SERIAL << F("> Lex: we are subscribed to this stream ID = ") << frame.data[1] << endl;

               // find a free receive context
               for (i = 0; i < _num_receive_contexts; i++)
               {
                  if (!_receive_context[i]->in_use)
                  {
                     // DEBUG_SERIAL << F("> Lex: using receive context = ") << i << endl;
                     break;
                  }
               }

               if (i < _num_receive_contexts)
               {
                  _receive_context[i]->in_use = true;
                  _receive_context[i]->receive_stream_id = frame.data[1];
                  _receive_context[i]->incoming_message_length = (frame.data[3] << 8) + frame.data[4];
                  _receive_context[i]->incoming_message_crc = (frame.data[5] << 8) + frame.data[6];
                  _receive_context[i]->incoming_bytes_received = 0;
                  memset(_receive_context[i]->buffer, 0, _receive_buffer_len);
                  _receive_context[i]->receive_buffer_index = 0;
                  _receive_context[i]->expected_next_receive_sequence_num = 1;
                  _receive_context[i]->sender_canid = (frame.id & 0x7f);
                  _receive_context[i]->last_fragment_received = SystemTick::GetMilli();
                  // DEBUG_SERIAL << F("> Lex: received header packet for stream id = ") << _receive_context[i]->receive_stream_id << F(", message length = ") << _receive_context[i]->incoming_message_length << endl;
               }
               else
               {
                  // DEBUG_SERIAL << F("> Lex: unable to find free receive context for new message") << endl;
               }

               break;
            }
         }
      }
      else
      {
         // DEBUG_SERIAL << F("> Lex: not handling header packet with non-zero flags") << endl;
      }
   }
   else
   { // continuation packet

      // DEBUG_SERIAL << F("> Lex: this is a continuation packet") << endl;

      // find a matching receive context, using the stream ID and sender CANID
      for (i = 0; i < _num_receive_contexts; i++)
      {
         if (_receive_context[i]->in_use && _receive_context[i]->receive_stream_id == frame.data[1] && _receive_context[i]->sender_canid == (frame.id & 0x7f))
         {
            break;
         }
      }

      // return if not found
      if (i >= _num_receive_contexts)
      {
         return;
      }

      // error if out of sequence
      if (frame.data[2] != _receive_context[i]->expected_next_receive_sequence_num)
      {
         _messagehandler(_receive_context[i]->buffer, _receive_context[i]->receive_buffer_index, _receive_context[i]->receive_stream_id, CBUS_LONG_MESSAGE_SEQUENCE_ERROR);
         _receive_context[i]->in_use = false;
         return;
      }

      // consume up to 5 bytes of message data from this fragment
      for (int_fast8_t j = 0; j < 5; j++)
      {
         _receive_context[i]->buffer[_receive_context[i]->receive_buffer_index] = frame.data[j + 3];
         ++_receive_context[i]->receive_buffer_index;
         ++_receive_context[i]->incoming_bytes_received;
         _receive_context[i]->last_fragment_received = SystemTick::GetMilli();

         // if we have consumed the entire message, surface it to the user's handler
         if (_receive_context[i]->incoming_bytes_received >= _receive_context[i]->incoming_message_length)
         {
            if (_use_crc && _receive_context[i]->incoming_message_crc != 0)
            {
               tmpcrc = crc16((uint8_t *)_receive_context[i]->buffer, _receive_context[i]->receive_buffer_index);
            }

            if (_receive_context[i]->incoming_message_crc != tmpcrc)
            {
               status = CBUS_LONG_MESSAGE_CRC_ERROR;
            }
            else
            {
               status = CBUS_LONG_MESSAGE_COMPLETE;
            }

            _messagehandler(_receive_context[i]->buffer, _receive_context[i]->receive_buffer_index, _receive_context[i]->receive_stream_id, status);
            _receive_context[i]->in_use = false;
            break;

            // if the buffer is now full, give the user what we have with an error status
         }
         else if (_receive_context[i]->receive_buffer_index >= _receive_buffer_len)
         {
            _messagehandler(_receive_context[i]->buffer, _receive_context[i]->receive_buffer_index, _receive_context[i]->receive_stream_id, CBUS_LONG_MESSAGE_TRUNCATED);
            _receive_context[i]->in_use = false;
            break;
         }
      }

      // increment the expected next sequence number for this stream context
      ++_receive_context[i]->expected_next_receive_sequence_num;
   }
}

//
/// set whether to calculate and compare a CRC of the message
//

void CBUSLongMessageEx::use_crc(bool use_crc)
{
   _use_crc = use_crc;
}

///////////////////////////////////////////////////////////////////////////////
//////// CRC implementations

uint32_t crc32(const uint8_t *s, size_t n)
{
   uint32_t crc = 0xFFFFFFFF;

   for (size_t i = 0; i < n; i++)
   {
      uint8_t ch = s[i];
      for (size_t j = 0; j < 8; j++)
      {
         uint32_t b = (ch ^ crc) & 1;
         crc >>= 1;
         if (b)
            crc = crc ^ 0xEDB88320;
         ch >>= 1;
      }
   }

   return ~crc;
}

/*
//                                      16   12   5
// this is the CCITT CRC 16 polynomial X  + X  + X  + 1.
// This works out to be 0x1021, but the way the algorithm works
// lets us use 0x8408 (the reverse of the bit pattern).  The high
// bit is always assumed to be set, thus we only use 16 bits to
// represent the 17 bit value.
*/

// http://stjarnhimlen.se/snippets/crc-16.c

#define POLY 0x8408

uint16_t crc16(uint8_t *data_p, uint16_t length)
{
   uint8_t i;
   uint16_t data;
   uint16_t crc = 0xffff;

   if (length == 0)
   {
      return (~crc);
   }

   do
   {
      for (i = 0, data = (uint16_t)0xff & *data_p++;
           i < 8;
           i++, data >>= 1)
      {
         if ((crc & 0x0001) ^ (data & 0x0001))
            crc = (crc >> 1) ^ POLY;
         else
            crc >>= 1;
      }
   } while (--length);

   crc = ~crc;
   data = crc;
   crc = (crc << 8) | (data >> 8 & 0xff);

   return (crc);
}

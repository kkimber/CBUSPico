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

#include "CBUSGridConnect.h"
#include "CBUSACAN2040.h"
#include <cstdio>
#include <cctype>
#include <cstdlib>
#include <new>

constexpr uint16_t TCP_PORT = 5550;
constexpr uint8_t FIFO_SIZE = 10;

///
/// Class to encode and decode CBUS Grid Connect messages
///
CBUSGridConnect::CBUSGridConnect() : m_tcpServer{}
{
   // NOTE: Single CAN circular buffer for all instances of this class
   // really we should only ever have a single instance, but avoid singleton
   m_pCANBuffer = new (std::nothrow) CBUSCircularBuffer(FIFO_SIZE);
}

CBUSGridConnect::~CBUSGridConnect()
{
   // Clean up CAN buffer
   if (m_pCANBuffer != nullptr)
   {
      delete m_pCANBuffer;
   }
}

///
/// @brief Start the GridConnect TCP Server
///
/// @return true the server was successfully created and started
/// @return false the server could not be created or started
///
bool CBUSGridConnect::startServer()
{
   if (m_pCANBuffer == nullptr)
   {
      return false;
   }

   if (!serverOpen(static_cast<void *>(&m_tcpServer)))
   {
      serverClose(static_cast<void *>(&m_tcpServer));
      return false;
   }

   return true;
}

///
/// @brief Stop the GridConnect Server
///
/// @return true the server was shutdown successfully
/// @return false the server could not be cleanly shutdown
///
bool CBUSGridConnect::stopServer()
{
   /// @todo check clean shutdown of server
   m_tcpServer.complete = true;
   return serverClose(static_cast<void *>(&m_tcpServer)) == ERR_OK;
}

///
/// @brief Create / open a server connection
///
/// @param arg Pointer to a TCP Server struct to hold information relating to this connection
/// @return true the server connection was opened successfully
/// @return false the server connection could not be opened
///
bool CBUSGridConnect::serverOpen(void *arg)
{
   TCPServer_t *state = static_cast<TCPServer_t *>(arg);

   // Create new TCP protocol control block
   struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);

   if (!pcb)
   {
      return false;
   }

   // Bind the control block to the required TCP Port
   err_t err = tcp_bind(pcb, IP_ADDR_ANY, TCP_PORT);

   if (err)
   {
      return false;
   }

   // Set the control block into listen mode
   // we re-allocate the CB, so we must reassign here
   // allow one connection in the connection queue
   state->server_pcb = tcp_listen_with_backlog(pcb, 1);

   if (!state->server_pcb)
   {
      if (pcb)
      {
         tcp_close(pcb);
      }

      return false;
   }

   // Assign state to be returned in callbacks
   tcp_arg(state->server_pcb, state);

   // Specify callback to be used for Accept
   tcp_accept(state->server_pcb, serverAccept);

   return true;
}

err_t CBUSGridConnect::serverCloseConn(struct tcp_pcb *client_pcb)
{
   err_t err = ERR_OK;

   // Is there a client connection
   if (client_pcb != nullptr)
   {
      // Clear all callback function pointers
      tcp_arg(client_pcb, nullptr);
      tcp_poll(client_pcb, nullptr, 0);
      tcp_sent(client_pcb, nullptr);
      tcp_recv(client_pcb, nullptr);
      tcp_err(client_pcb, nullptr);

      // Close client connection
      err = tcp_close(client_pcb);
      if (err != ERR_OK)
      {
         // Failed to close cleanly, so abort connection
         tcp_abort(client_pcb);
         err = ERR_ABRT;
      }
   }

   return err;
}

///
/// @brief Close a TCP Server connection
///
/// @param arg Pointer to a TCP Server struct holding information on the TCP Server connection to close
/// @return err_t ERR_OK on success
///
err_t CBUSGridConnect::serverClose(void *arg)
{
   TCPServer_t *state = static_cast<TCPServer_t *>(arg);

   err_t err = ERR_OK;

   // Is there a client connection
   if (state->client_pcb != nullptr)
   {
      serverCloseConn(state->client_pcb);

      // Clean-up
      state->client_pcb = nullptr;
   }

   // Is the server valid
   if (state->server_pcb)
   {
      // Clear callback function pointer
      tcp_arg(state->server_pcb, nullptr);

      // Close the server
      tcp_close(state->server_pcb);

      // Clean-up
      state->server_pcb = nullptr;
   }

   return err;
}

///
/// @brief Perform Grid Connect background procesing
///
void CBUSGridConnect::run()
{
   // Do we need to do anything?
}

///
/// @brief send a CAN message to Ethernet clients
///
/// @param msg reference to CAN frame to forward
///
void CBUSGridConnect::sendCANFrame(const CANFrame &msg)
{
   gcMessage_t gcMsg;

   if (encodeGC(msg, gcMsg))
   {
      if (m_tcpServer.client_pcb)
      {
         serverSend(&m_tcpServer, m_tcpServer.client_pcb, gcMsg);
      }
   }
}

///
/// @brief 
/// 
/// @return true 
/// @return false 
///
bool CBUSGridConnect::available()
{
   if (m_pCANBuffer != nullptr)
   {
      return m_pCANBuffer->available();
   }

   return false;
}

///
/// @brief 
/// 
/// @return CANFrame 
///
CANFrame CBUSGridConnect::get()
{
   if (m_pCANBuffer != nullptr)
   {
      CANFrame* msg = m_pCANBuffer->get();
      return *msg;
   }

   // Return empty frame
   CANFrame msg;
   return msg;
}

///
/// @brief Accept a client connection
///
/// @param arg TCP Server state associated with the server accept
/// @param client_pcb Pointer to a client control block to describe the client connection
/// @param err Error code if there was an error accepting the connection
/// @return err_t ERR_OK on success
///
err_t CBUSGridConnect::serverAccept(void *arg, struct tcp_pcb *client_pcb, err_t err)
{
   TCPServer_t *state = static_cast<TCPServer_t *>(arg);

   // Check for client control block and errors
   if ((err != ERR_OK) || (client_pcb == nullptr))
   {
      serverClose(arg);
      return ERR_VAL;
   }

   // Disable nagle - TODO [good idea]
   tcp_nagle_disable(client_pcb);

   // Set priority - TODO [good idea]
   tcp_setprio(client_pcb, TCP_PRIO_MIN);

   // Save the client connection control block
   state->client_pcb = client_pcb;

   // Assign state to be returned in callbacks
   tcp_arg(client_pcb, state);

   // Specify callback functions
   tcp_sent(client_pcb, serverSent);
   tcp_recv(client_pcb, serverRecv);
   // tcp_poll(client_pcb, serverPoll, xx); ///@ todo for client timeouts
   tcp_err(client_pcb, serverErr);

   return ERR_OK;
}

///
/// @brief Receive data from a client connection
///
/// @param arg TCP Server state associated with the server providing data
/// @param tpcb Pointer to the Control Block providing data
/// @param p Pointer to the received data buffer, or null if the connection is closed
/// @param err Error code if there was an error receiving data
/// @return err_t ERR_OK on success
///
err_t CBUSGridConnect::serverRecv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
   TCPServer_t *state = static_cast<TCPServer_t *>(arg);

   // Check for error or closed connection
   if ((err != ERR_OK) || (p == nullptr) || (arg == nullptr))
   {
      // Error or client closed connection
      if (p != nullptr)
      {
         // Inform stack we have received the data
         tcp_recved(tpcb, p->tot_len);
         pbuf_free(p);
      }

      serverCloseConn(tpcb);

      // Clean-up client connection
      if (state)
      {
         state->client_pcb = nullptr;
      }

      return ERR_OK;
   }

   // this method is callback from lwIP, so cyw43_arch_lwip_begin is not required, however you
   // can use this method to cause an assertion in debug mode, if this method is called when
   // cyw43_arch_lwip_begin IS needed
   cyw43_arch_lwip_check();

   // Check for received data length
   if (p->tot_len > 0)
   {
      // Receive into the buffer
      const uint16_t buffer_left = GC_MAX_MSG - state->bufferRecv.len;
      state->bufferRecv.len += pbuf_copy_partial(p, &state->bufferRecv.byte[state->bufferRecv.len],
                                                 p->tot_len > buffer_left ? buffer_left : p->tot_len, 0);
      tcp_recved(tpcb, p->tot_len);
   }

   // Free the buffer, now we have copied the data
   pbuf_free(p);

   // Have we have received a complete Grid Connect message yet?
   // Check end of frame indicator
   if ((state->bufferRecv.byte[state->bufferRecv.len - 1] == ';') ||
       (state->bufferRecv.byte[state->bufferRecv.len - 2] == ';') || // Allow 1x EOL char
       (state->bufferRecv.byte[state->bufferRecv.len - 3] == ';'))   // Allow 2x EOL chars
   {
      // Check start of frame indicator
      if (state->bufferRecv.byte[0] == ':')
      {
         // Yes, complete message received
         // Attempt to parse into a CANFrame
         CANFrame canMsg;

         if (decodeGC(state->bufferRecv, canMsg))
         {
            // Parse successful, so queue for sending on CAN
            m_pCANBuffer->put(canMsg);
            CBUSACAN2040::sendCANMessage(canMsg);
         }

         // Init receive buffer
         state->bufferRecv = {};
      }
   }

   return ERR_OK;
}

err_t CBUSGridConnect::serverSent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
   TCPServer_t *state = static_cast<TCPServer_t *>(arg);

   state->sent_len += len;

   if (state->sent_len >= GC_MAX_MSG)
   {
      // We should get the data back from the client
      state->bufferRecv.len = 0;
   }

   return ERR_OK;
}

err_t CBUSGridConnect::serverSend(void *arg, struct tcp_pcb *tpcb, gcMessage_t msg)
{
   TCPServer_t *state = static_cast<TCPServer_t *>(arg);

   // LwIP Stack is NOT thread safe - lock stack as calling from different context
   cyw43_arch_lwip_begin();

   state->sent_len = 0;

   err_t err = tcp_write(tpcb, msg.byte, msg.len, TCP_WRITE_FLAG_COPY);

   if (err == ERR_OK)
   {
      err = tcp_output(tpcb);
   }

   if (err != ERR_OK)
   {
      err = serverClose(arg);
   }

   // Unlock stack
   cyw43_arch_lwip_end();

   return err;
}

err_t CBUSGridConnect::serverPoll(void *arg, struct tcp_pcb *tpcb)
{
   // @todo - do we want this?
   return ERR_OK;
}

void CBUSGridConnect::serverErr(void *arg, err_t err)
{
   if (err != ERR_ABRT)
   {
      serverClose(arg);
   }
}

///
/// @brief Encode a CANFrame as info a GridConnect string
///
/// @param canMsg CAN Frame to encode
/// @param gcMsg Grid Connect message where result will be stored
/// @return true if the provided CANFrame was successfully encoded
/// @return false if the provided CANFrame could not be encoded into GridConnect
///
bool CBUSGridConnect::encodeGC(const CANFrame &canMsg, gcMessage_t &gcMsg)
{
   hexByteChars_t tmp;
   uint8_t offset = 0;

   gcMsg = {}; // Initialize / null terminate provided buffer

   // Check for validity of the CAN frame
   if ((canMsg.len > 8) ||                                 // Check max length
       ((canMsg.ext == false) && (canMsg.id > 0x7FF)) ||   // Check ID for standard CAN frame
       ((canMsg.ext == true) && (canMsg.id > 0x1FFFFFFF))) // Check ID for extended ID CAN frame
   {
      return false;
   }

   // Mark start of frame
   gcMsg.byte[0] = ':';

   // set starting character & standard or extended CAN identifier
   if (canMsg.ext)
   {
      // mark as extended message
      gcMsg.byte[1] = 'X';

      // extended 29 bit CAN idenfier in bytes 2 to 9

      // chars 2 & 3 are ID bits 21 to 28
      uint8ToHex((canMsg.id >> 21), tmp);
      gcMsg.byte[2] = tmp.upperNibble;
      gcMsg.byte[3] = tmp.lowerNibble;

      //  char 4 -  bits 1 to 3 are ID bits 18 to 20
      uint8ToHex((canMsg.id >> 17) & 0xE, tmp);
      gcMsg.byte[4] = tmp.lowerNibble;

      //  char 5 -  bits 0 to 1 are ID bits 16 & 17
      uint8ToHex((canMsg.id >> 16) & 0x3, tmp);
      gcMsg.byte[5] = tmp.lowerNibble;

      // chars 6 to 9 are ID bits 0 to 15
      uint8ToHex((canMsg.id & 0xff00) >> 8, tmp);
      gcMsg.byte[6] = tmp.upperNibble;
      gcMsg.byte[7] = tmp.lowerNibble;

      uint8ToHex((canMsg.id & 0x00ff), tmp);
      gcMsg.byte[8] = tmp.upperNibble;
      gcMsg.byte[9] = tmp.lowerNibble;

      offset = 10; // ':X' + 8
   }
   else
   {
      // mark as standard message
      gcMsg.byte[1] = 'S';

      // standard 11 bit CAN idenfier in bytes 2 to 5, left shifted 5 to occupy highest bits

      uint16_t canId = (canMsg.id << 5);

      uint8ToHex((canId & 0xff00) >> 8, tmp);
      gcMsg.byte[2] = tmp.upperNibble;
      gcMsg.byte[3] = tmp.lowerNibble;

      uint8ToHex((canId & 0x00ff), tmp);
      gcMsg.byte[4] = tmp.upperNibble;
      gcMsg.byte[5] = tmp.lowerNibble;

      offset = 6; // ':S' + 4
   }

   // set RTR or normal, in byte 6 or 10
   gcMsg.byte[offset++] = canMsg.rtr ? 'R' : 'N';

   // now add hex data from byte 7 or 11, if len > 0
   for (int_fast8_t i = 0; i < canMsg.len; i++)
   {
      // convert data byte to hex
      uint8ToHex(canMsg.data[i], tmp);
      gcMsg.byte[offset++] = tmp.upperNibble;
      gcMsg.byte[offset++] = tmp.lowerNibble;
   }

   // finally add terminator
   gcMsg.byte[offset++] = ';';

   // set length of resulting Grid Connect message
   gcMsg.len = offset;

   return true;
}

///
/// @brief Decode a Grid Connect message into a CANFrame
///
/// @param gcMsg Reference to Grid Connect message to decode
/// @param canMsg Reference to a CANFrame where decoded frame is written
/// @return true if the provided Grid Connect message was successfully decoded
/// @return false if the provided Grid Connect message could not be decoded
///
bool CBUSGridConnect::decodeGC(const gcMessage_t &gcMsg, CANFrame &canMsg)
{
   hexByteChars_t tmp;
   uint8_t offset = 0;

   // Sanity check overal length first
   if (gcMsg.len > GC_MAX_MSG)
   {
      return false;
   }

   // must start with message start indicator
   if (gcMsg.byte[offset++] != ':')
   {
      return false;
   }

   // CAN Identifier, must be either 'X' or 'S'
   if (gcMsg.byte[offset] == 'X')
   {
      // Extended ID CAN Frame
      canMsg.ext = true;

      // now get 29 bit ID - convert from hex, but check they are all hex first
      if (checkHexChars(gcMsg, 2, 8) == false)
      {
         return false;
      }

      // all chars are hex, so build up id from characters 2 to 9
      // chars 2 & 3 are bits 21 to 28
      tmp.upperNibble = gcMsg.byte[2];
      tmp.lowerNibble = gcMsg.byte[3];
      canMsg.id = static_cast<uint32_t>(hexToUint8(tmp)) << 21;

      // chars 4 & 5 -  bits 5 to 7 are bits 18 to 20
      tmp.upperNibble = gcMsg.byte[4];
      tmp.lowerNibble = gcMsg.byte[5];
      canMsg.id += static_cast<uint32_t>(hexToUint8(tmp) & 0xE0) << 13;

      // chars 4 & 5 -  bits 0 to 1 are bits 16 & 17
      tmp.upperNibble = gcMsg.byte[4];
      tmp.lowerNibble = gcMsg.byte[5];
      canMsg.id += static_cast<uint32_t>(hexToUint8(tmp) & 0x3) << 16;

      // chars 6 & 7 are bits 8 to
      tmp.upperNibble = gcMsg.byte[6];
      tmp.lowerNibble = gcMsg.byte[7];
      canMsg.id += static_cast<uint32_t>(hexToUint8(tmp)) << 8;

      // chars 8 & 9 are bits 0 to 7
      tmp.upperNibble = gcMsg.byte[8];
      tmp.lowerNibble = gcMsg.byte[9];
      canMsg.id += hexToUint8(tmp);

      offset = 10; // ':X' + 8
   }
   else if (gcMsg.byte[offset] == 'S')
   {
      // Standard ID CAN Frame
      canMsg.ext = false;

      // now get 11 bit ID - convert from hex, but check they are all hex first
      if (checkHexChars(gcMsg, 2, 4) == false)
      {
         return false;
      }

      // 11 bit identifier needs to be shifted right by 5
      canMsg.id = std::strtol(&gcMsg.byte[2], nullptr, 16) >> 5;

      offset = 6; // ':S' + 4
   }
   else
   {
      // Not marked either Extended or Standard ID frame
      return false;
   }

   // do RTR flag
   if (gcMsg.byte[offset] == 'R')
   {
      canMsg.rtr = true;
   }
   else if (gcMsg.byte[offset] == 'N')
   {
      canMsg.rtr = false;
   }
   else
   {
      // Not marked either RTR or Normal frame
      return false;
   }

   offset++; // set to next character afert RTR flag

   // Convert the CAN frame data

   // first find out how many chars of data there are
   // data length should be gcMsg.len minus current offset
   // as it is now (after RTR flag), minus terminator
   int dataLength = gcMsg.len - offset - 1;

   // must be even number of hex characters, and no more than 16
   if ((dataLength % 2) || (dataLength > 16))
   {
      return false;
   }

   // set byte length of the CANFrame
   canMsg.len = dataLength / 2;

   // now convert hex data into bytes
   for (int_fast8_t i = 0; i < canMsg.len; i++)
   {
      // check they are hex chars first
      if (checkHexChars(gcMsg, offset, 2) == false)
      {
         return false;
      }

      // convert byte
      tmp.upperNibble = gcMsg.byte[offset++];
      tmp.lowerNibble = gcMsg.byte[offset++];
      canMsg.data[i] = hexToUint8(tmp);
   }

   //
   // must have end of message character
   if (gcMsg.byte[gcMsg.len - 1] != ';')
   {
      return false;
   }

   // Grid Connect message parsed successfully
   return true;
}

///
/// @brief Convert a uint8_t value to two hex characters
///
/// @param u8 value to convert
/// @param byteStr converted hex characters
///
void CBUSGridConnect::uint8ToHex(const uint8_t u8, hexByteChars_t &byteStr)
{
   constexpr char HexChars[16]{'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

   byteStr.lowerNibble = HexChars[u8 & 0x0f];
   byteStr.upperNibble = HexChars[(u8 >> 4) & 0x0f];
}

///
/// @brief Convert two hex characters to a uint8_t
///
/// @param hexChar hex characters to convert, upper and lower nibbles
/// @return converted uint8_t value
///
uint8_t CBUSGridConnect::hexToUint8(const hexByteChars_t &hexChar)
{
   uint8_t result;

   // Convert lower nibble
   if (hexChar.lowerNibble < 'A')
   {
      result = hexChar.lowerNibble - '0';
   }
   else
   {
      result = hexChar.lowerNibble - 'A' + 10;
   }

   // Convert upper nibble
   if (hexChar.upperNibble < 'A')
   {
      result += (hexChar.upperNibble - '0') << 4;
   }
   else
   {
      result += (hexChar.upperNibble - 'A' + 10) << 4;
   }

   return result;
}

///
/// @brief Check all characeters in the GC message, in the range specified, are hex
///
/// @param msg Reference to the grid connect message to check
/// @param start byte offset of the first character to check
/// @param count number of characters to check
/// @return true if the characters in range are all hex
/// @return false if one or more characters are not hex
///
bool CBUSGridConnect::checkHexChars(const gcMessage_t &msg, uint8_t start, uint8_t count)
{
   // check number of characters requested
   for (int_fast8_t i = start; i < start + count; i++)
   {
      // must be upper case, so fail if lower case
      if (islower(msg.byte[i]))
      {
         return false;
      }

      // must be hexadecimal character
      if (!isxdigit(msg.byte[i]))
      {
         return false;
      }
   }

   return true;
}

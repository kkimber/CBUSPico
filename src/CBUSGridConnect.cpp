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

constexpr uint8_t FIFO_SIZE = 10;

///
/// Class to encode and decode CBUS Grid Connect messages
///
CBUSGridConnect::CBUSGridConnect() : m_tcpServer{}
{
   // NOTE: Single CAN FIFO for all instances of this class
   // really we should only ever have a single instance, but avoid singleton
   m_pCANBuffer = new (std::nothrow) CBUSCircularBuffer(FIFO_SIZE);
}

///
/// @brief Destroy the CBUSGridConnect object instance
///
CBUSGridConnect::~CBUSGridConnect()
{
   // Clean up CAN FIFO
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
bool CBUSGridConnect::startServer(uint16_t nPort)
{
   // Check we have a FIFO before starting the server
   if (m_pCANBuffer == nullptr)
   {
      return false;
   }

   // Attempt to start the server
   if (!serverOpen(nPort))
   {
      // Cleaup on failure to start
      serverShutdown(&m_tcpServer);
      return false;
   }

   // Server started successfully
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
   return serverShutdown(&m_tcpServer) == ERR_OK;
}

///
/// @brief Create / open a server connection
///
/// @param nPort TCP port for the Grid Connect server
/// @return true the server connection was opened successfully
/// @return false the server connection could not be opened
///
bool CBUSGridConnect::serverOpen(uint16_t nPort)
{
   // Create new TCP protocol control block, allow IPv4 and IPv6
   m_pServerCB = tcp_new_ip_type(IPADDR_TYPE_ANY);

   if (m_pServerCB == nullptr)
   {
      // failed to create server control block
      return false;
   }

   // Bind the control block to the required TCP Port for IPv4 and IPv6
   err_t err = tcp_bind(m_pServerCB, IP_ADDR_ANY, nPort);

   if (err != ERR_OK)
   {
      // failed to bind
      return false;
   }

   // Set the control block into listen mode
   // we re-allocate the CB, so we must reassign here
   /// @todo mulitple clients !!
   m_pServerCB = tcp_listen_with_backlog(m_pServerCB, 1);

   // Assign state to be returned in callbacks
   tcp_arg(m_pServerCB, &m_tcpServer);

   // Specify callback to be used for Accept
   tcp_accept(m_pServerCB, serverAccept);

   return true;
}

void CBUSGridConnect::extractAndQueueGC(TCPServer_t *state, uint16_t nStart, struct pbuf *p)
{
   // GridConnect start and end frame delimiters
   static uint8_t gcStart[1] = {':'};
   static uint8_t gcEnd[1] = {';'};

   // Attempt to find the start of a GridConnect frame in the pbuf
   nStart = pbuf_memfind(p, gcStart, 1, nStart);

   // Was the start delimiter found?
   if (nStart != 0xFFFF)
   {
      // Yes, so attempt find the end delimiter
      uint16_t nEnd = pbuf_memfind(p, gcEnd, 1, nStart);
      if (nEnd != 0xFFFF)
      {
         // End delimiter found, sanity check on GC length
         uint16_t gcMsgLen = (nEnd + 1) - nStart;
         if (gcMsgLen <= GC_MAX_MSG)
         {
            // Looks OK, so extract the complete GC frame
            if (pbuf_copy_partial(p, &state->bufferRecv.byte[0], gcMsgLen, nStart))
            {
               // Set length 
               state->bufferRecv.len = gcMsgLen;

               // Attempt to parse into a CANFrame
               CANFrame canMsg;

               if (decodeGC(state->bufferRecv, canMsg))
               {
                  // Parse successful, so queue for sending on CAN
                  m_pCANBuffer->put(canMsg);
                  CBUSACAN2040::sendCANMessage(canMsg);
               }
            }

            // Init receive buffer, we either queued a CAN msg, or corrupt data
            state->bufferRecv = {};
         }

         // Any data still left in pbuf?
         if ((p->tot_len - (gcMsgLen + nStart)) > 0)
         {
            // Recursive parse - potentially another frame, or partial frame received
            extractAndQueueGC(state, gcMsgLen + nStart, p);
         }
         else
         {
            // Done with the pbuf, so free it
            pbuf_free(p);
         }
      }
      else
      {
         // We didn't find the end delimiter, copy data received so far into buffer
         uint16_t bufLeft = p->tot_len - nStart;
         if (bufLeft <= GC_MAX_MSG)
         {
            // Copy partial buffer and save length
            pbuf_copy_partial(p, &state->bufferRecv.byte[0], bufLeft, nStart);
            state->bufferRecv.len = bufLeft;
         }

         // We either copied partial frame, or data looks corrupt as its too long for a GC frame
         // either way, free the buffer
         pbuf_free(p);
      }
   }
   else
   {
      // Start delimiter not found - trash data
      pbuf_free(p);
   }
}

///
/// @brief Close a client connection
/// 
/// @param pClientCB Pointer to client Control Block for the connection to close
/// @param server Pointer to TCP Server struct
///
void CBUSGridConnect::serverCloseConn(struct tcp_pcb *pClientCB, TCPServer_t* server)
{
   // Is there a client connection
   if (pClientCB != nullptr)
   {
      // Clear all callback function pointers
      tcp_arg(pClientCB, nullptr);
      tcp_poll(pClientCB, nullptr, 0);
      tcp_sent(pClientCB, nullptr);
      tcp_recv(pClientCB, nullptr);
      tcp_err(pClientCB, nullptr);

      // Close client connection
      tcp_close(pClientCB);

      // Clean up the connection
      server->pClientCB = nullptr;
   }
}

///
/// @brief Close a TCP Server connection
///
/// @param state Pointer to a TCP Server struct holding information on the TCP Server connection to close
/// @return err_t ERR_OK on success
///
err_t CBUSGridConnect::serverShutdown(TCPServer_t* state)
{
   err_t err = ERR_OK;

   // Is there a client connection
   if (state->pClientCB != nullptr)
   {
      serverCloseConn(state->pClientCB, state);

      // Clean-up
      state->pClientCB = nullptr;
   }

   // Is the server valid
   if (m_pServerCB)
   {
      // Clear callback function pointer
      tcp_arg(m_pServerCB, nullptr);

      // Close the server
      tcp_close(m_pServerCB);

      // Clean-up
      m_pServerCB = nullptr;
   }

   return err;
}

///
/// @brief send a CAN message to Ethernet clients
///
/// @param msg reference to CAN frame to forward
///
void CBUSGridConnect::sendCANFrame(const CANFrame &msg)
{
   gcMessage_t gcMsg;

   // Try to encode the provided CAN frame into a Grid Connect message
   if (encodeGC(msg, gcMsg))
   {
      // If we have a client connection, initiate sending the frame to the client
      if (m_tcpServer.pClientCB != nullptr)
      {
         serverSend(&m_tcpServer, m_tcpServer.pClientCB, gcMsg);
      }
   }
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
      // Try to force write immediately
      err = tcp_output(tpcb);
   }
   else if (err == ERR_MEM)
   {
      // Low on memory to send
      // Save message to send later - enqueue into a pbuf ??
      state->bufferSent = msg;
   }
   else
   {
      // some other error - TODO shutdown??
      err = serverShutdown(state);
   }

   // Unlock stack
   cyw43_arch_lwip_end();

   return err;
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
/// @param pClientCB Pointer to a client control block to describe the client connection
/// @param err Error code if there was an error accepting the connection
/// @return err_t ERR_OK on success
///
err_t CBUSGridConnect::serverAccept(void *arg, struct tcp_pcb *pClientCB, err_t err)
{
   TCPServer_t *state = static_cast<TCPServer_t *>(arg);

   // Check for client control block and errors
   if ((err != ERR_OK) || (pClientCB == nullptr))
   {
      serverShutdown(state);
      return ERR_VAL;
   }

   // Disable nagle - TODO [good idea]
   tcp_nagle_disable(pClientCB);

   // Set priority - TODO [good idea]
   tcp_setprio(pClientCB, TCP_PRIO_MIN);

   // Save the client connection control block
   state->pClientCB = pClientCB;
   state->clientState = clientState_t::CS_ACCEPTED;

   // Assign state to be returned in callbacks
   tcp_arg(pClientCB, state);

   // Specify callback functions
   tcp_sent(pClientCB, serverSent);
   tcp_recv(pClientCB, serverRecv);
   // tcp_poll(pClientCB, serverPoll, xx); ///@ todo for client timeouts
   tcp_err(pClientCB, serverErr);

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
   err_t retErr;

   // this method is callback from lwIP, so cyw43_arch_lwip_begin is not required, however you
   // can use this method to cause an assertion in debug mode, if this method is called when
   // cyw43_arch_lwip_begin IS needed
   cyw43_arch_lwip_check();

   // Check for closed connection
   if (p == nullptr)
   {
      // Remote host closed connection
      state->clientState = clientState_t::CS_CLOSING;
      if (state->p == nullptr)
      {
         // We're done sending, close it
         serverCloseConn(tpcb, state);
      }
      else
      {
         // we're not done yet TODO
         // tcp_sent(tpcb, serverSent);
         // serverSend(state, tpcb, xx)d;
      }
      retErr = ERR_OK;
   }
   else if (err != ERR_OK)
   {
      // Cleanup, for unknown reason
      if (p != NULL)
      {
         state->p = NULL;
         pbuf_free(p);
      }
      retErr = err;
   }
   else if (state->clientState == clientState_t::CS_ACCEPTED)
   {
      // First receive
      state->clientState = clientState_t::CS_RECEIVED;

      // Check we have data to parse
      if (p->tot_len > 0)
      {
         // Attempt to extract and queue Grid Connect frames
         extractAndQueueGC(state, 0, p);

         // Inform the stack we've received the data
         tcp_recved(tpcb, p->tot_len);
      }
   
      retErr = ERR_OK;
   }
   else if (state->clientState == clientState_t::CS_RECEIVED)
   {
      // Check we have data to parse
      if (p->tot_len > 0)
      {
         // Save actual received data length
         uint16_t nLen = p->tot_len;

         // Do we have a partial message stored?
         if (state->bufferRecv.len != 0)
         {
            // Yes, so allocate a buffer
            struct pbuf* pFragment =  pbuf_alloc(PBUF_RAW, state->bufferRecv.len, PBUF_RAM);

            // Check for allocation OK
            if (pFragment == nullptr)
            {
               // out of memory, trash the data
               pbuf_free(p);
               tcp_recved(tpcb, p->tot_len);
               return ERR_OK;
            }

            // Copy in data to fragment - TODO optimize?
            for (uint16_t i= 0; i < state->bufferRecv.len; i++)
            {
               pbuf_put_at(pFragment, i, state->bufferRecv.byte[i]);
            }

            // Chain previous fragment to latest data
            pbuf_cat(pFragment, p);

            // Reassign back to original pbuf
            p = pFragment;
         }

         // Now extract and queue GC frames from start of pbuf
         extractAndQueueGC(state, 0, p);

         // Inform the stack we've received all the data
         tcp_recved(tpcb, nLen);
      }

      retErr = ERR_OK;
   }
   else if (state->clientState == clientState_t::CS_CLOSING)
   {
      // Remote side closing twice, trash data
      tcp_recved(tpcb, p->tot_len);
      pbuf_free(p);

      retErr = ERR_OK;
   }
   else
   {
      // Unknown state - trash the data
      tcp_recved(tpcb, p->tot_len);
      pbuf_free(p);
      retErr = ERR_OK;
   }

   return retErr;
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

err_t CBUSGridConnect::serverPoll(void *arg, struct tcp_pcb *tpcb)
{
   // @todo - do we want this?
   TCPServer_t *state = static_cast<TCPServer_t *>(arg);

   if (state->bufferSent.len != 0)
   {
      // Install sent notification
      tcp_sent(state->pClientCB, serverSent);
      serverSend(state, state->pClientCB, state->bufferSent);
   }

   return ERR_OK;
}

void CBUSGridConnect::serverErr(void *arg, err_t err)
{
   TCPServer_t *state = static_cast<TCPServer_t *>(arg);

   if (err != ERR_ABRT)
   {
      serverShutdown(state);
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

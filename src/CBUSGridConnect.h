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

#include <cstdint>
#include "CBUSCircularBuffer.h"

#include "pico/cyw43_arch.h"

#include "lwip/pbuf.h"
#include "lwip/tcp.h"

/// Maximum possible length of a Grid Connect "string", excluding null and any EOL chars
constexpr uint8_t GC_MAX_MSG = 28;

/// Type for holding a Grid Connect string
typedef struct
{
   char byte[GC_MAX_MSG]; ///< Grid Connect message - null terminated C string
   uint8_t len;           ///< Actual length of the Grid Connect message
} gcMessage_t;

/// Type to hold upper and lower nibble characters of an 8 bit hex value
typedef struct
{
   unsigned char lowerNibble; ///< Lower nibble character
   unsigned char upperNibble; ///< Upper nibble character
} hexByteChars_t;

enum class clientState_t : uint8_t
{
  CS_NONE = 0,
  CS_ACCEPTED,
  CS_RECEIVED,
  CS_CLOSING
};

/// Type to hold status information for the GridConnect TCP server
typedef struct
{
   struct tcp_pcb *pClientCB; ///< Pointer to Client control block
   clientState_t clientState; ///< Client state
   struct pbuf *p;            ///< pbuf (chain) to recycle
   gcMessage_t bufferRecv;    ///< Buffer for received GC frames
   gcMessage_t bufferSent;    ///< Buffer for transmitted GC frames
   int sent_len;
} TCPServer_t;

///
/// class to support CBUS Grid Connect protocol
///

class CBUSGridConnect
{
public:
   CBUSGridConnect();
   ~CBUSGridConnect();
   bool startServer(uint16_t nPort);
   bool stopServer(void);
   bool serverOpen(uint16_t nPort);
   static void extractAndQueueGC(TCPServer_t *state, uint16_t nStart, struct pbuf *p);
   // Interface for clients to send us frames from CAN
   void sendCANFrame(const CANFrame &msg);
   static err_t serverSend(void *arg, struct tcp_pcb *tpcb, gcMessage_t msg);
   // Interface to receive CAN Frames from GridConnect clients
   bool available(void);
   CANFrame get(void);
   // Helper to close connection - called from LwIP callback
   static void serverCloseConn(struct tcp_pcb *pClientCB, TCPServer_t* server);
   // Helper to shutdown server - called from LwIP callback
   static err_t serverShutdown(TCPServer_t *state);
   // LwIP callback methods
   static err_t serverAccept(void *arg, struct tcp_pcb *pClientCB, err_t err);
   static err_t serverRecv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
   static err_t serverSent(void *arg, struct tcp_pcb *tpcb, u16_t len);
   static err_t serverPoll(void *arg, struct tcp_pcb *tpcb);
   static void serverErr(void *arg, err_t err);

private:
   ///
   TCPServer_t m_tcpServer;
   static inline struct tcp_pcb *m_pServerCB = nullptr;
   static inline CBUSCircularBuffer *m_pCANBuffer = nullptr;
   static bool encodeGC(const CANFrame &canMsg, gcMessage_t &gcMsg);
   static bool decodeGC(const gcMessage_t &gcMsg, CANFrame &canMsg);
   static void uint8ToHex(const uint8_t u8, hexByteChars_t &byteStr);
   static uint8_t hexToUint8(const hexByteChars_t &hexChar);
   static bool checkHexChars(const gcMessage_t &msg, uint8_t start, uint8_t count);
};

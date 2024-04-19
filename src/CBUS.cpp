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

#include "CBUS.h"
#include "SystemTick.h"
#include "CBUSUtil.h"

#include <pico/stdlib.h>
#include <new>
#include <cstring>
#include <cstdlib>

constexpr uint8_t EVENT_SET_MASK = 0b10010000;
constexpr uint8_t EVENT_CLR_MASK = 0b00000110;
constexpr uint8_t EVENT_SHORT_MASK = 0b00001000;

// forward function declarations
void makeHeader_impl(CANFrame &msg, uint8_t id, uint8_t priority = 0x0b);

//
/// construct a CBUS object with a CBUSConfig object that the user provides.
/// note that this CBUSConfig object must have a lifetime longer than the CBUS object.
//

CBUSbase::CBUSbase(CBUSConfig &config) : m_numMsgsSent{0x0L},
                                         m_numMsgsRcvd{0x0UL},
                                         m_moduleConfig{config},
                                         m_pModuleParams{nullptr},
                                         m_pModuleName{nullptr},
                                         eventHandler{nullptr},
                                         eventHandlerEx{nullptr},
                                         frameHandler{nullptr},
                                         m_opcodes{nullptr},
                                         m_numOpcodes{0x0U},
                                         m_enumResponses{},
                                         m_bModeChanging{false},
                                         m_bCANenum{false},
                                         m_bLearn{false},
                                         m_bThisNN{false},
                                         m_nodeNumber{0x0U},
                                         m_eventNumber{0x0U},
                                         timeOutTimer{0x0UL},
                                         CANenumTime{0x0UL},
                                         m_bEnumerationRequired{false},
                                         longMessageHandler{nullptr},
                                         m_coeObj{nullptr}
{
}

//
/// register the user callback for learned events
//

void CBUSbase::setEventHandlerCB(eventCallback_t evCallback)
{
   eventHandler = evCallback;
}

// register a user event callback which receives the opcode on/off state and the first event variable

void CBUSbase::setEventHandlerExCB(eventExCallback_t evExCallback)
{
   eventHandlerEx = evExCallback;
}

//
/// register a user callback for CAN frames
/// default args in .h declaration for opcodes array (nullptr) and size (0)
//

void CBUSbase::setFrameHandler(frameCallback_t frameCallback, uint8_t opcodes[], uint8_t num_opcodes)
{
   frameHandler = frameCallback;
   m_opcodes = opcodes;
   m_numOpcodes = num_opcodes;
}

//
/// assign the module parameter set
//

void CBUSbase::setParams(cbusparam_t *mparams)
{
   m_pModuleParams = mparams;
}

//
/// assign the module name
//

void CBUSbase::setName(module_name_t *moduleName)
{
   m_pModuleName = moduleName;
}

//
/// set module to SLiM mode
//

void CBUSbase::setSLiM()
{
   m_bModeChanging = false;
   m_moduleConfig.setNodeNum(0);
   m_moduleConfig.setFLiM(false);
   m_moduleConfig.setCANID(0);

   indicateFLiMMode(m_moduleConfig.getFLiM());
}

//
/// extract CANID from CAN frame header
//

inline uint8_t CBUSbase::getCANID(uint32_t header)
{
   return header & 0x7f;
}

///
/// @brief Send single byte frame - opcode only.
///
/// @param opc the CBUS opcode to be sent
/// @return true frame was sent OK
/// @return false frame could not be sent
///
bool CBUSbase::sendSingleOpc(const uint8_t opc)
{
   CANFrame frame;
   frame.id = m_moduleConfig.getCANID();
   frame.len = 1; // Single byte frame
   frame.data[0] = opc;

   // send the frame
   return sendMessage(frame);
};

///
/// @brief Send opcode plus our node number, can be used to send simple 3-byte frame,
/// or may have up to five further bytes appended.
///
/// @param opc the CBUS opcode to be sent
/// @param dataLen the number of optional bytes to send
/// @param d1 Optional first data byte
/// @param d2 Optional second data byte
/// @param d3 Optional third data byte
/// @param d4 Optional fourth data byte
/// @param d5 Optional fifth data byte
/// @return true frame was send OK
/// @return false frame could not be sent
///
bool CBUSbase::sendOpcMyNN(const uint8_t opc, const uint8_t dataLen, const uint8_t d1, const uint8_t d2, const uint8_t d3, const uint8_t d4, const uint8_t d5)
{
   CANFrame frame;
   frame.id = m_moduleConfig.getCANID();
   frame.len = 3 + dataLen; // Minimal frame with node number is 3 bytes
   frame.data[0] = opc;

   // Node Number is set in sendMsgMyNN

   // Copy in up to 5 extra bytes of data
   frame.data[3] = d1;
   frame.data[4] = d2;
   frame.data[5] = d3;
   frame.data[6] = d4;
   frame.data[7] = d5;

   // and send the frame with our node number
   return sendMsgMyNN(frame);
};

///
/// @brief Send opcode plus our node number, can be used to send simple 3-byte frame,
/// or may have up to five further bytes appended.
///
/// @param opc the CBUS opcode to be sent
/// @param nodeId the CBUS node number to be sent
/// @param dataLen the number of optional bytes to send
/// @param d1 Optional first data byte
/// @param d2 Optional second data byte
/// @param d3 Optional third data byte
/// @param d4 Optional fourth data byte
/// @param d5 Optional fith data byte
/// @return true the frame was sent OK
/// @return false frame could no be sent
///
bool CBUSbase::sendOpcNN(const uint8_t opc, const uint16_t nodeId, const uint8_t dataLen, const uint8_t d1, const uint8_t d2, const uint8_t d3, const uint8_t d4, const uint8_t d5)
{
   CANFrame frame;
   frame.id = m_moduleConfig.getCANID();
   frame.len = 3 + dataLen; // Minimal frame with node number is 3 bytes
   frame.data[0] = opc;

   // Node Number is set in sendMsgNN

   // Copy in up to 5 extra bytes of data
   frame.data[3] = d1;
   frame.data[4] = d2;
   frame.data[5] = d3;
   frame.data[6] = d4;
   frame.data[7] = d5;

   // and send the frame with the specified node number
   return sendMsgNN(frame, nodeId);
};

///
/// @brief Send a frame using our Node Number
///
/// @param frame reference to the partially filled frame to send
/// @return true the frame was sent OK
/// @return false the frame could not be sent
///
bool CBUSbase::sendMsgMyNN(CANFrame &frame)
{
   // Set our node number into the frame
   frame.data[1] = highByte(m_moduleConfig.getNodeNum());
   frame.data[2] = lowByte(m_moduleConfig.getNodeNum());

   // and send the frame
   return sendMessage(frame);
};

///
/// @brief Send a frame using the specified Node Number
///
/// @param frame reference to the partially filled frame to send
/// @param nodeId node number to insert into the frame before sending
/// @return true the frame was sent OK
/// @return false the frame could not be sent
///
bool CBUSbase::sendMsgNN(CANFrame &frame, const uint16_t nodeId)
{
   // Set node number into the frame
   frame.data[1] = highByte(nodeId);
   frame.data[2] = lowByte(nodeId);

   // and send the frame
   return sendMessage(frame);
}

///
/// @brief Send long CBUS event, no data, using our node number.
///
/// @param eventNum the CBUS Event Number
/// @param onEvent if true, send ON event, otherwise send OFF event
/// @return true the frame was sent OK
/// @return false the frame cound not be sent
///
bool CBUSbase::sendMyEvent(const uint16_t eventNum, const bool onEvent)
{
   // Send event with our node number
   return sendEventWithData(m_moduleConfig.getNodeNum(), eventNum, onEvent);
};

///
/// @brief Send CBUS event, no data, using specified node number.
/// send short event if eventNode is 0
///
/// @param eventNode the event node number, 0 for a short event
/// @param eventNum the CBUS event node number
/// @param onEvent if true send ON event, otherwise send OFF event
/// @return true the frame was sent OK
/// @return false frame could no be sent
///
bool CBUSbase::sendEvent(const uint16_t eventNode, const uint16_t eventNum, const bool onEvent)
{
   // Send event with specified node number
   return sendEventWithData(eventNode, eventNum, onEvent);
};

///
/// @brief Send CBUS event with optional data bytes
///
/// @param eventNode the event node number, 0 for a short event
/// @param eventNum the CBUS event node number
/// @param onEvent if true send ON event, otherwise send OFF event
/// @param dataLen number of optional; data bytes to send [0-3]
/// @param d1 Optional first event data byte
/// @param d2 Optional second event data byte
/// @param d3 Optional third event data byte
/// @return true the frame was sent OK
/// @return false frame could no be sent
///
bool CBUSbase::sendEventWithData(uint16_t eventNode, const uint16_t eventNum, const bool onEvent, const uint8_t dataLen, const uint8_t d1, const uint8_t d2, const uint8_t d3)
{
   CANFrame frame;
   frame.id = m_moduleConfig.getCANID();
   frame.len = 5 + dataLen;
   frame.data[0] = OPC_ACON; // Start with long event opcode

   // Send short event if node ID is zero
   if (eventNode == 0)
   {
      frame.data[0] |= 0x08;                   // Short event opcode
      eventNode = m_moduleConfig.getNodeNum(); // Add module node id
   }

   // Check if off event
   if (!onEvent)
   {
      frame.data[0] |= 0x01; // Off event
   }

   // Check if optional data is being appended
   if (dataLen > 0)
   {
      frame.data[0] |= (dataLen << 5); // Set opcode based on data length
   }

   // Set the event number
   frame.data[3] = highByte(eventNum);
   frame.data[4] = lowByte(eventNum);

   // Copy in optional event data
   frame.data[5] = d1;
   frame.data[6] = d2;
   frame.data[7] = d3;

   // If we're consuming our own events, post to rx queue
   if (m_coeObj != nullptr)
   {
      m_coeObj->put(frame);
   }

   return sendMsgNN(frame, eventNode);
};

///
/// @brief Send a debug message with 5 data bytes
/// data bytes will be zero if not specifically specified
///
/// @param nodeId the node number to be sent
/// @param d1 first data byte (optional)
/// @param d2 second data byte (optional)
/// @param d3 third data byte (optional)
/// @param d4 fourth data byte (optional)
/// @param d5 fifth data byte (optional)
/// @return true the frame was sent OK
/// @return false frame could no be sent
///
bool CBUSbase::sendDataEvent(const uint16_t nodeId, const uint8_t d1, const uint8_t d2, const uint8_t d3, const uint8_t d4, const uint8_t d5)
{
   CANFrame frame;
   frame.id = m_moduleConfig.getCANID();
   frame.len = 8; // Full length CAN frame
   frame.data[0] = OPC_ACDAT;
   frame.data[3] = d1;
   frame.data[4] = d2;
   frame.data[5] = d3;
   frame.data[6] = d4;
   frame.data[7] = d5;

   return sendMsgNN(frame, nodeId);
}

//
/// send a WRACK (write acknowledge) message
//

bool CBUSbase::sendWRACK()
{
   // send a write acknowledgement response
   return sendOpcMyNN(OPC_WRACK);
}

//
/// send a CMDERR (command error) message
//

bool CBUSbase::sendCMDERR(uint8_t cerrno)
{
   // send a command error response, 1 extra byte for error code
   return sendOpcMyNN(OPC_CMDERR, 1, cerrno);
}

//
/// is this an Extended CAN frame ?
//

bool CBUSbase::isExt(const CANFrame &amsg) const
{
   return amsg.ext;
}

//
/// is this a Remote frame ?
//

bool CBUSbase::isRTR(const CANFrame &amsg) const
{
   return amsg.rtr;
}

//
/// if in FLiM mode, initiate a CAN ID enumeration cycle
//

void CBUSbase::CANenumeration()
{
   // initiate CAN bus enumeration cycle, either due to ENUM opcode, ID clash, or user button press
   CANFrame msg;

   // set global variables
   m_bCANenum = true;                    // we are enumerating
   CANenumTime = SystemTick::GetMilli(); // the cycle start time
   memset(m_enumResponses, 0, sizeof(m_enumResponses));

   // send zero-length RTR frame
   msg.len = 0;
   sendMessage(msg, true, false); // fixed arg order in v 1.1.4, RTR - true, ext = false
}

//
/// initiate the transition from SLiM to FLiM mode
//
void CBUSbase::initFLiM()
{
   indicateMode(MODE_CHANGING);

   m_bModeChanging = true;
   timeOutTimer = SystemTick::GetMilli();

   // send RQNN message with current NN, which may be zero if a virgin/SLiM node
   sendOpcMyNN(OPC_RQNN);
}

//
/// revert from FLiM to SLiM mode
//
void CBUSbase::revertSLiM()
{
   // send NNREL message
   sendOpcMyNN(OPC_NNREL);

   // revert to SLiM
   setSLiM();
}

//
/// change or re-confirm node number
//
void CBUSbase::renegotiate()
{
   initFLiM();
}

//
/// set the CBUS LEDs to indicate the current mode
//
void CBUSbase::indicateMode(uint8_t mode)
{
   switch (mode)
   {
   case MODE_FLIM:
      m_ledYlw.on();
      m_ledGrn.off();
      break;

   case MODE_SLIM:
      m_ledYlw.off();
      m_ledGrn.on();
      break;

   case MODE_CHANGING:
      m_ledYlw.blink();
      m_ledGrn.off();
      break;

   default:
      break;
   }
}

void CBUSbase::indicateFLiMMode(bool bFLiM)
{
   indicateMode(bFLiM ? MODE_FLIM : MODE_SLIM);
}

//
/// main CBUS message processing procedure
//
void CBUSbase::process(uint8_t num_messages)
{
   CANFrame msg;

   // start bus enumeration if required
   if (m_bEnumerationRequired)
   {
      m_bEnumerationRequired = false;
      CANenumeration();
   }

   //
   // process FLiM UI
   //

   // allow LEDs to update
   m_ledGrn.run();
   m_ledYlw.run();

   // allow the CBUS switch some processing time
   m_sw.run();

   //
   /// use LEDs to indicate that the user can release the switch
   //

   if (m_sw.isPressed() && m_sw.getCurrentStateDuration() > SW_TR_HOLD)
   {
      indicateMode(MODE_CHANGING);
   }

   //
   /// handle switch state changes
   //

   if (m_sw.stateChanged())
   {
      // has switch been released ?
      if (!m_sw.isPressed())
      {
         // how long was it pressed for ?
         uint32_t press_time = m_sw.getLastStateDuration();

         // long hold > 6 secs
         if (press_time > SW_TR_HOLD)
         {
            // initiate mode change
            if (!m_moduleConfig.getFLiM())
            {
               initFLiM();
            }
            else
            {
               revertSLiM();
            }
         }

         // short 1-2 secs
         if (press_time >= 1000 && press_time < 2000)
         {
            renegotiate();
         }

         // very short < 0.5 sec
         if (press_time < 500 && m_moduleConfig.getFLiM())
         {
            CANenumeration();
         }
      }
      else
      {
         // do any switch release processing here
      }
   }

   // get received CAN frames from buffer
   // process by default 3 messages per run so the user's application code doesn't appear unresponsive under load

   uint8_t mcount = 0;

   while ((available() ||                                      // Message on local queue
         (m_coeObj != nullptr && m_coeObj->available()))       // Message on COE queue
         && mcount < num_messages)                             // Limit messages processed per run
   {
      ++mcount;

      // at least one CAN frame is available in either the reception buffer or the COE buffer
      // retrieve the next one

      bool bOwnEvent = false;

      if (m_coeObj != nullptr && m_coeObj->available())
      {
         msg = m_coeObj->get();
         
         // Flag this is from us, so we don't trigger enumeration
         bOwnEvent = true;
      }
      else
      {
         // Process message received off CAN
         msg = getNextMessage();
      }

      // extract OPC and node number
      uint8_t opc = msg.data[0];
      uint16_t nodeID = (msg.data[1] << 8) + msg.data[2];

      // determine if frame is directed at this node number
      m_bThisNN = (msg.data[0] >> 5) >= 2 && (nodeID == m_moduleConfig.getNodeNum());

      //
      /// extract the CANID() of the sending module
      //

      uint8_t remoteCANID = getCANID(msg.id);

      //
      /// if registered, call the user handler with this new frame
      //

      if (frameHandler != nullptr)
      {
         // check if incoming opcode is in the user list, if list length > 0
         if (m_numOpcodes > 0)
         {
            for (int_fast8_t i = 0; i < m_numOpcodes; i++)
            {
               if (opc == m_opcodes[i])
               {
                  frameHandler(msg);
                  break;
               }
            }
         }
         else
         {
            frameHandler(msg);
         }
      }

      // is this a CANID() enumeration request from another node (RTR set) ?
      if (msg.rtr)
      {
         // send an empty message to show our CANID()
         msg.len = 0;
         sendMessage(msg);
         continue;
      }

      //
      /// Set flag if we find a CANID() conflict with the frame's producer from another module
      /// i.e. not from our own consumed events!
      /// Doesn't apply to RTR or zero-length frames, so as not to trigger an enumeration loop
      //

      if (!bOwnEvent && (remoteCANID == m_moduleConfig.getCANID()) && (msg.len > 0))
      {
         m_bEnumerationRequired = true;
      }

      // is this an extended frame ? we currently ignore these as bootloader, etc data may confuse us !
      if (msg.ext)
      {
         continue;
      }

      // are we enumerating CANID()s ?
      if (m_bCANenum && msg.len == 0)
      {
         // store this response in the responses array
         if (remoteCANID > 0)
         {
            // fix to correctly record the received CANID()
            bitWrite(m_enumResponses[(remoteCANID / 16)], remoteCANID % 8, 1);
         }

         continue;
      }

      // Parse and process CBUS messages
      bool bConsumed = parseCBUSMsg(msg);

      /// Show activity on the LED's
      if (m_moduleConfig.getFLiM())
      {
         // In FLiM the green LED is flickered on activity
         // short ficker if not not consumed
         m_ledGrn.pulse(!bConsumed);
      }
      else
      {
         // In SLiM the yellow LED is flickered on activity
         // short flicker if not consumed
         m_ledYlw.pulse(!bConsumed);
      }

   } // while messages available

   // check CAN bus enumeration timer
   checkCANenum();

   //
   /// check 30 sec timeout for SLiM/FLiM negotiation with FCU
   //

   if (m_bModeChanging && ((SystemTick::GetMilli() - timeOutTimer) >= 30000))
   {
      indicateFLiMMode(m_moduleConfig.getFLiM());
      m_bModeChanging = false;
   }

   //
   /// end of CBUS message processing
   //
}

void CBUSbase::checkCANenum()
{
   //
   /// check the 100ms CAN enumeration cycle timer
   //

   uint8_t selected_id = 1; // default if no responses from other modules

   // if (bCANenum && !bCANenumComplete && (SystemTick::GetMilli() - CANenumTime) >= 100) {
   if (m_bCANenum && (SystemTick::GetMilli() - CANenumTime) >= 100)
   {
      // enumeration timer has expired -- stop enumeration and process the responses

      // iterate through the 128 bit field
      for (int_fast8_t i = 0; i < 16; i++)
      {

         // ignore if this uint8_t is all 1's -> there are no unused IDs in this group of numbers
         if (m_enumResponses[i] == 0xff)
         {
            continue;
         }

         // for each bit in the uint8_t
         for (int_fast8_t b = 0; b < 8; b++)
         {

            // ignore first bit of first uint8_t -- CAN ID zero is not used for nodes
            if (i == 0 && b == 0)
            {
               continue;
            }

            // if the bit is not set
            if (bitRead(m_enumResponses[i], b) == 0)
            {
               selected_id = ((i * 16) + b);

               // i = 16; // ugh ... but probably better than a goto :)
               // but using a goto saves 4 bytes of program size ;)
               goto check_done;
            }
         }
      }

   check_done:

      // bCANenumComplete = true;
      m_bCANenum = false;
      CANenumTime = 0UL;

      // store the new CAN ID
      m_moduleConfig.setCANID(selected_id);

      // send NNACK
      sendOpcMyNN(OPC_NNACK);
   }
}

//
/// set the long message handler object to receive long message frames
//

void CBUSbase::setLongMessageHandler(CBUSLongMessage *handler)
{
   longMessageHandler = handler;
}

void CBUSbase::consumeOwnEvents(CBUScoe *coe)
{
   m_coeObj = coe;
}

///
/// @brief Validate a new NV value
///
/// @param NVindex Index of the NV being changed
/// @param oldValue Old value of the NV
/// @param NVvalue New value of the NV
/// @return true the new NV value is valid
/// @return false the new NV value is invalid
///
bool CBUSbase::validateNV(const uint8_t NVindex, const uint8_t oldValue, const uint8_t NVvalue)
{
   // Derived class can provide validation
   return true;
}

///
/// @brief Notifies of change of NV value, allowing module specific processing
///
/// @param NVindex Index of the changed NV
/// @param oldValue Old value of the NV
/// @param NVvalue New value of the NV
///
void CBUSbase::actUponNVchange(const uint8_t NVindex, const uint8_t oldValue, const uint8_t NVvalue)
{
   // Handled in derived class
   ;
}

///
/// @brief Process CBUS opcodes.  Anything module explicit will already been
/// deault with in the application code before calling here.
///
/// @param msg reference to the CBUS message
/// @return true if the message was processed
/// @return false if the message was ignored
///
bool CBUSbase::parseCBUSMsg(CANFrame &msg)
{
   // Check if this is an Event
   if (((msg.data[0] & EVENT_SET_MASK) == EVENT_SET_MASK) &&
       ((~msg.data[0] & EVENT_CLR_MASK) == EVENT_CLR_MASK))
   {
      // if this is an event, pass to module's event processing
      return parseCBUSEvent(msg);
   }

   // FLiM processing doesn't process events
   // Process the incoming (non-event) message
   return parseFLiMCmd(msg);
}

///
/// @brief Consumes a CBUS event
///
/// @param msg reference to the received CBUS event message
/// @return true if the message was processed
/// @return false if the message was ignored
///
bool CBUSbase::parseCBUSEvent(CANFrame &msg)
{
   // Check for short or long event
   if (msg.data[0] & EVENT_SHORT_MASK)
   {
      // Short event
      m_nodeNumber = 0;
   }
   else
   {
      // Long event, so extract node number
      m_nodeNumber = (msg.data[1] << 8) + msg.data[2];
   }

   // Extract and cache event number
   m_eventNumber = (msg.data[3] << 8) + msg.data[4];

   // try to find a matching stored event -- match on nn, en
   uint8_t index = m_moduleConfig.findExistingEvent(m_nodeNumber, m_eventNumber);

   // call any registered event handler

   if (index < m_moduleConfig.EE_MAX_EVENTS)
   {
      // Call standard event handler (if defined)
      if (eventHandler != nullptr)
      {
         eventHandler(index, msg);

         return true; // Processed event
      }
      // Call extended event handler (if defined)
      else if (eventHandlerEx != nullptr)
      {
         // Determine if this is an ON or OFF event from the OPC
         bool bOnEvent = (msg.data[0] % 2 == 0);

         eventHandlerEx(index, msg, bOnEvent,
                        ((m_moduleConfig.EE_NUM_EVS > 0) ? m_moduleConfig.getEventEVval(index, 1) : 0));

         return true; // Processed event
      }
   }

   return false;
}

///
/// @brief Process CBUS opcode for FLiM.  Called after any module specific
/// CBUS opcodes have been dealt with.
///
/// @param msg reference to the received CBUS message
/// @return true if the message was processed
/// @return false if the message was ignored
///
bool CBUSbase::parseFLiMCmd(CANFrame &msg)
{
   bool cmdProcessed = false;

   // extract the OPC
   uint8_t opc = msg.data[0];

   // extract node number and event number and cache for use in OPC processors
   m_nodeNumber = (msg.data[1] << 8) + msg.data[2];
   m_eventNumber = (msg.data[3] << 8) + msg.data[4];

   if (m_bLearn)
   {
      cmdProcessed = true;

      switch (opc)
      {
      case OPC_NNLRN:
         if (m_bThisNN)
         {
            // Addressed to us and we're already in learn mode, nothing to do
            break;
         }
         // If in learn mode and receive a learn message for another node, we must exit learn mode
         // fall through - no break!!
      case OPC_NNULN:
         // Release node from learn mode
         m_bLearn = false;
         break;

      case OPC_NNCLR:
         // Clear all events
         doNnclr();
         break;

      case OPC_EVULN:
         // Unlearn event
         doEvuln();
         break;

      case OPC_EVLRN:
         // Teach event whilst in learn mode
         doEvlrn(msg.data[5], msg.data[6]);
         break;

      case OPC_EVLRNI:
         // Teach event whilst in learn mode with event index
         doEvlrn(msg.data[6], msg.data[7]); // Current implementation does not use index to save learnt event
         break;

      case OPC_REQEV:
         // Read event variable by event id
         doReqev(msg.data[5]);
         break;

      default:
         cmdProcessed = false;
         break;
      }
   } // in learn mode

   // process commands specifically addressed to us
   if (!cmdProcessed && m_bThisNN)
   {
      cmdProcessed = true;

      switch (opc)
      {
      case OPC_RQNPN:
         // Read one node parameter by index
         doRqnpn(msg.data[3]);
         break;

      case OPC_NNLRN:
         // Put node into learn mode if we're in FLiM
         if (m_moduleConfig.getFLiM())
         {
            m_bLearn = true;
         }
         break;

      case OPC_NNEVN:
         // Read available event slots
         doNnevn();
         break;

      case OPC_NERD:
         // Read all stored events
         doNerd();
         break;

      case OPC_NENRD:
         // Read a single stored event by index
         doNenrd(msg.data[3]);
         break;

      case OPC_RQEVN:
         // Read number of stored events
         doRqevn();
         break;

      case OPC_NVRD:
         // Read value of a node variable
         doNvrd(msg.data[3]);
         break;

      case OPC_NVSET:
         // Set a node variable
         doNvset(msg.data[3], msg.data[4]);
         break;

      case OPC_REVAL:
         // Read event variable by index
         doReval(msg.data[3], msg.data[4]);
         break;

#ifdef BOOTLOADER_PRESENT
      case OPC_BOOT:
         // Enter bootloader mode
         ee_write((WORD)EE_BOOT_FLAG, 0xFF);
         Reset();
         break;
#endif

      case OPC_CANID:
         //    if (!setNewCanId(msg.data[3]))
         {
            sendCMDERR(CMDERR_INVALID_EVENT); // seems a strange error code but that's what the spec says...
         }
         break;

      case OPC_ENUM:
         //     doEnum(true);
         break;

      default:
         cmdProcessed = false;
         break;
      }
   } // this NN

   if (!cmdProcessed)
   { // Process any command not sent specifically to us that still needs action
      switch (msg.data[0])
      {
#ifdef AREQ_SUPPORT
      case OPC_AREQ:
         // Illicit a response indicating last long event state
         // The NN supplied is actually the Long event's NN instead of the addressed module
         // but these should be the same for the producer of the event.
         doAreq((((WORD)msg.data[d1]) << 8) + msg.data[d2], (((WORD)msg.data[d3]) << 8) + msg.data[d4]);
         cmdProcessed = TRUE;
         break;

      case OPC_ASRQ:
         // Illicit a response indicating last short event state
         // The NN supplied can either be zero (short event) or the NN of the producer module
         // Any module that produces that short event can respond if the NN is zero, if it is non-zero, then only that producer module should respond.
         if (((((WORD)msg.data[d1]) << 8) + msg.data[d2] == 0) || m_bThisNN(msg.data))
         {
            doAreq(0, (((WORD)msg.data[d3]) << 8) + msg.data[d4]);
            cmdProcessed = TRUE;
         }
         break;
#endif
      case OPC_QNN:
         QNNrespond(); // Respond to node query 	//  ??? update to do new spec response agreed
         cmdProcessed = TIMER_PAUSE_RESET;
         break;
      }
   }

   // In setup mode, also check for FLiM commands not addressed to
   // any particular node

   if ((!cmdProcessed) && (m_bModeChanging))
   {
      cmdProcessed = true;

      switch (msg.data[0])
      {
      case OPC_RQNP:
         // Read node parameters
         doRqnp();
         break;

      case OPC_RQMN:
         // Read module type name
         doRqmn();
         break;

      case OPC_SNN:
         // Set node number
         doSnn();
         break;

      default:
         cmdProcessed = false;
         break;
      }
   }

   return (cmdProcessed);
}

///
/// @brief Returns the current parameter flags
///
/// @return uint8_t parameter flags
///
uint8_t CBUSbase::getParFlags()
{
   // Indicate FLiM / Learn if in FLiM learn mode
   if (m_bLearn)
   {
      return (PF_LRN | PF_FLiM | m_pModuleParams->param[PAR_FLAGS]);
   }

   // Indicate FLiM if we're in FLiM mode
   if (m_moduleConfig.getFLiM())
   {
      return (PF_FLiM | m_pModuleParams->param[PAR_FLAGS]);
   }

   // Otherwise return defined flags
   return m_pModuleParams->param[PAR_FLAGS];
}

///
/// @brief QNN Respond - send response bytes to QNN query
///
void CBUSbase::QNNrespond()
{
   // respond with PNN
   sendOpcMyNN(OPC_PNN, 3,
               m_pModuleParams->param[PAR_MANU],
               m_pModuleParams->param[PAR_MTYP],
               getParFlags());
}

///
/// @brief Read one node parameter by index
///
/// @param index parameter index
///
void CBUSbase::doRqnpn(const uint8_t index)
{
   // Check requested parameter index is valid
   if (index <= m_pModuleParams->param[PAR_NPARAMS])
   {
      // Get requested parameter value
      uint8_t val = m_pModuleParams->param[index];

      // Get dynamic flags
      if (index == PAR_FLAGS)
      {
         val = getParFlags();
      }

      // Reply with requested parameter
      sendOpcMyNN(OPC_PARAN, 2, index, val);
   }
   else
   {
      sendCMDERR(CMDERR_INV_PARAM_IDX);
   }
}

///
/// @brief Read a node variable
///
/// @param NVindex the index of the node variable to read
///
void CBUSbase::doNvrd(const uint8_t NVindex)
{
   // check the bounds of NVindex, It starts at 1
   if ((NVindex == 0) || (NVindex > m_moduleConfig.EE_NUM_NVS))
   {
      sendCMDERR(CMDERR_INV_NV_IDX);
   }
   else
   {
      // respond with NVANS
      sendOpcMyNN(OPC_NVANS, 2, NVindex, m_moduleConfig.readNV(NVindex));
   }
}

///
/// @brief Set a node variable
///
/// @param NVindex the index of the NV to be written
/// @param NVvalue the new NV value
///
void CBUSbase::doNvset(const uint8_t NVindex, const uint8_t NVvalue)
{
   // received NVSET -- set NV by index
   if ((NVindex == 0) > (m_moduleConfig.EE_NUM_NVS))
   {
      sendCMDERR(CMDERR_INV_NV_IDX);
   }
   else
   {
      // Check NV value is valid
      uint8_t oldValue = m_moduleConfig.readNV(NVindex);
      if (validateNV(NVindex, oldValue, NVvalue))
      {
         // update EEPROM for this NV -- NVs are indexed from 1, not zero
         m_moduleConfig.writeNV(NVindex, NVvalue);

         actUponNVchange(NVindex, oldValue, NVvalue);

         // respond with WRACK
         sendWRACK();
      }
      else
      {
         sendCMDERR(CMDERR_INV_NV_VALUE);
      }
   }
}

///
/// @brief Return the first few parameters when requested via RQNP in FLiM setup mode
///
void CBUSbase::doRqnp()
{
   CANFrame msg;

   // respond with PARAMS message
   msg.len = 8;
   msg.data[0] = OPC_PARAMS;                         // opcode
   msg.data[1] = m_pModuleParams->param[PAR_MANU];   // manf code
   msg.data[2] = m_pModuleParams->param[PAR_MINVER]; // minor code ver
   msg.data[3] = m_pModuleParams->param[PAR_MTYP];   // module ident
   msg.data[4] = m_pModuleParams->param[PAR_EVNUM];  // number of events
   msg.data[5] = m_pModuleParams->param[PAR_EVNUM];  // events vars per event
   msg.data[6] = m_pModuleParams->param[PAR_NVNUM];  // number of NVs
   msg.data[7] = m_pModuleParams->param[PAR_MAJVER]; // major code ver

   sendMessage(msg);
}

///
/// @brief Read the module name
/// Module name is returned as a 7 byte space padded string in OPC_NAME
///
void CBUSbase::doRqmn()
{
   CANFrame msg;

   msg.len = 8;
   msg.data[0] = OPC_NAME;
   memcpy(&msg.data[1], m_pModuleName, sizeof(module_name_t));
   sendMessage(msg);
}

///
/// @brief Set and save the assigned node number, module is now in FLiM
///
void CBUSbase::doSnn()
{
   // save the node Number
   m_moduleConfig.setNodeNum(m_nodeNumber);

   // we are now in FLiM mode - update the configuration
   m_bModeChanging = false;
   m_moduleConfig.setFLiM(true);
   indicateFLiMMode(m_moduleConfig.getFLiM());

   // enumerate the CAN bus to allocate a free CAN ID
   CANenumeration(); /// @todo checkif this is needed here

   /// @todo rebuild hash??

   // respond with NNACK
   sendOpcMyNN(OPC_NNACK);
}

///
/// @brief Clear all Events
///
void CBUSbase::doNnclr()
{
   if (m_bLearn == true)
   {
      // Clear all events
      m_moduleConfig.clearEventsEEPROM();

      // recreate the hash table
      m_moduleConfig.clearEvHashTable();

      sendWRACK();
   }
   else
   {
      // We're not in learn mode
      sendCMDERR(CMDERR_NOT_LRN);
   }
}

///
/// @brief Do event learn
///
/// @param evNum event variable number
/// @param evVal event variable value
///
void CBUSbase::doEvlrn(const uint8_t evNum, const uint8_t evVal)
{
   // evNum starts at 1
   if (evNum == 0)
   {
      sendCMDERR(CMDERR_INV_NV_IDX);
      return;
   }

   ////// @todo APP callback

   // search for this NN, EN as we may just be adding an EV to an existing learned event
   uint8_t index = m_moduleConfig.findExistingEvent(m_nodeNumber, m_eventNumber);

   // not found - it's a new event
   if (index >= m_moduleConfig.EE_MAX_EVENTS)
   {
      index = m_moduleConfig.findEventSpace();
   }

   // if existing or new event space found, write the event data
   if (index < m_moduleConfig.EE_MAX_EVENTS)
   {
      // write the event to EEPROM at this location -- EVs are indexed from 1 but storage offsets start at zero !!

      // don't repeat this for subsequent EVs
      if (evNum < 2)
      {
         EVENT_INFO_t evInfo{.nodeNumber = m_nodeNumber, .eventNumber = m_eventNumber};
         m_moduleConfig.writeEvent(index, evInfo);
      }

      m_moduleConfig.writeEventEV(index, evNum, evVal);

      // recreate event hash table entry
      m_moduleConfig.updateEvHashEntry(index);

      // respond with WRACK
      sendWRACK();
   }
   else
   {
      // respond with CMDERR
      sendCMDERR(CMDERR_INV_NV_IDX);
   }
}

///
/// @brief Read an event variable by index,
/// does not require module to be in learn mode
///
/// @param enNum event number
/// @param evNum event variable number
///
void CBUSbase::doReval(const uint8_t enNum, const uint8_t evNum)
{
   // Check event variable is in range
   if (evNum > m_moduleConfig.EE_NUM_EVS)
   {
      sendCMDERR(CMDERR_INV_EV_IDX);
      return;
   }

   // Checks valid event number
   if (m_moduleConfig.getEvTableEntry(enNum) != 0)
   {
      // return request event variable
      sendOpcMyNN(OPC_NEVAL, 3, enNum, evNum, m_moduleConfig.getEventEVval(enNum, evNum));
   }
   else
   {
      // Invalid event specified
      sendCMDERR(CMDERR_INVALID_EVENT);
   }
}

///
/// @brief Unlearn event
///
void CBUSbase::doEvuln()
{
   // search for this NN and EN pair
   uint8_t index = m_moduleConfig.findExistingEvent(m_nodeNumber, m_eventNumber);

   if (index < m_moduleConfig.EE_MAX_EVENTS)
   {
      m_moduleConfig.clearEventEEPROM(index);

      // update hash table
      m_moduleConfig.updateEvHashEntry(index);

      // @todo PIC does NOT respond with WRACK
      // sendWRACK();
   }
   else
   {
      // @todo PIC does NOT respond with CMDERR
      // sendCMDERR(CMDERR_INV_NV_IDX);
   }
}

///
/// @brief Read an event variable in learn mode
///
/// @param evNum event variable number
///
void CBUSbase::doReqev(const uint8_t evNum)
{
   // Check event variable is in range
   if (evNum > m_moduleConfig.EE_NUM_EVS)
   {
      sendCMDERR(CMDERR_INV_EV_IDX);
      return;
   }

   // Checks valid event number
   if (m_moduleConfig.getEvTableEntry(m_eventNumber) != 0)
   {
      // return request event variable
      sendOpcMyNN(OPC_NEVAL, 3, m_eventNumber, evNum, m_moduleConfig.getEventEVval(m_eventNumber, evNum));
   }
   else
   {
      // Invalid event specified
      sendCMDERR(CMDERR_INVALID_EVENT);
   }
}

///
/// @brief Request for the number of available event slots
///
void CBUSbase::doNnevn()
{
   uint8_t free_slots = 0;

   // count free slots using the event hash table
   for (int_fast8_t i = 0; i < m_moduleConfig.EE_MAX_EVENTS; i++)
   {
      if (m_moduleConfig.getEvTableEntry(i) == 0)
      {
         ++free_slots;
      }
   }

   // Send response with number of free event table slots
   sendOpcMyNN(OPC_EVNLF, 1, free_slots);
}

///
/// @brief Read all stored events
///
void CBUSbase::doNerd()
{
   /// @todo investigate timed response code in PIC lib

   CANFrame msg;
   msg.len = 8;
   msg.data[0] = OPC_ENRSP;              // response opcode
   msg.data[1] = highByte(m_nodeNumber); // my NN hi
   msg.data[2] = lowByte(m_nodeNumber);  // my NN lo

   // Loop for all events in the event table
   for (int_fast8_t i = 0; i < m_moduleConfig.EE_MAX_EVENTS; i++)
   {
      // Check for valid event
      if (m_moduleConfig.getEvTableEntry(i) != 0)
      {
         // it's a valid stored event

         // read the event data from EEPROM
         // construct and send a ENRSP message
         EVENT_INFO_t evInfo;
         m_moduleConfig.readEvent(i, evInfo);

         msg.data[3] = highByte(evInfo.nodeNumber);
         msg.data[4] = lowByte(evInfo.nodeNumber);
         msg.data[5] = highByte(evInfo.eventNumber);
         msg.data[6] = lowByte(evInfo.eventNumber);
         msg.data[7] = i; // event table index

         while (!sendMessage(msg))
         {
            ; // busy waiting to send
         }
      }
   }
}

///
/// @brief Read a single stored event by index and return an ENRSP response
///
/// @param index index into the event table
///
void CBUSbase::doNenrd(const uint8_t index)
{
   /// @todo - validate behaviour

   // Check for valid index
   if (m_moduleConfig.getEvTableEntry(index) == 0)
   {
      // invalid index
      sendCMDERR(CMDERR_INVALID_EVENT);
      return;
   }

   CANFrame msg;
   msg.len = 8;
   msg.data[0] = OPC_ENRSP;              // response opcode
   msg.data[1] = highByte(m_nodeNumber); // my NN hi
   msg.data[2] = lowByte(m_nodeNumber);  // my NN lo

   EVENT_INFO_t evInfo;
   m_moduleConfig.readEvent(index, evInfo);
   msg.data[3] = highByte(evInfo.nodeNumber);
   msg.data[4] = lowByte(evInfo.nodeNumber);
   msg.data[5] = highByte(evInfo.eventNumber);
   msg.data[6] = lowByte(evInfo.eventNumber);

   msg.data[7] = index; // event table index

   sendMessage(msg);
}

///
/// @brief Read number of stored events
///
void CBUSbase::doRqevn()
{
   // respond with 0x74 NUMEV
   sendOpcMyNN(OPC_NUMEV, 1, m_moduleConfig.numEvents());
}

///
/// @brief Retrieve internal CBUS Yellow LED UI object it can be configured
///        The caller is expected to call CBUSLED::setPin() on the returned object
///
/// @return Reference to the internal CBUSLED object for the Yellow FLiM LED
///
CBUSLED &CBUSbase::getCBUSYellowLED()
{
   return m_ledYlw;
}

///
/// @brief Retrieve internal CBUS Green LED UI object it can be configured
///        The caller is expected to call CBUSLED::setPin() on the returned object
///
/// @return Reference to the internal CBUSLED object for the Green FLiM LED
///
CBUSLED &CBUSbase::getCBUSGreenLED()
{
   return m_ledGrn;
}

///
/// @brief Retrieve internal CBUS Switch UI object it can be configured
///        The caller is expected to call CBUSSwitch::setPin() on the returned object
///
/// @return Reference to the internal CBUSSwitch object for the FLiM switch
///
CBUSSwitch &CBUSbase::getCBUSSwitch()
{
   return m_sw;
}

//
/// utility method to populate a CBUS message header
//

void CBUSbase::makeHeader(CANFrame &msg, uint8_t priority)
{
   makeHeader_impl(msg, m_moduleConfig.getCANID(), priority);
}

//
/// actual implementation of the makeHeader method
/// so it can be called directly or as a CBUS class method
/// the 11 bit ID of a standard CAN frame is comprised of: (4 bits of CBUS priority) + (7 bits of CBUS CAN ID)
/// priority = 1011 (0xB hex, 11 dec) as default argument, which translates to medium/low
//

void makeHeader_impl(CANFrame &msg, uint8_t id, uint8_t priority)
{
   msg.id = (priority << 7) + (id & 0x7f);
}

//
/// consume own events class
//

CBUScoe::CBUScoe(const uint8_t num_items)
{
   coe_buff = new (std::nothrow) CBUSCircularBuffer(num_items);
}

CBUScoe::~CBUScoe()
{
   if (coe_buff)
   {
      delete coe_buff;
   }
}

void CBUScoe::put(const CANFrame &msg)
{
   if (!coe_buff)
   {
      return;
   }

   coe_buff->put(msg);
}

bool CBUScoe::available()
{
   if (!coe_buff)
   {
      return false;
   }

   return coe_buff->available();
}

CANFrame CBUScoe::get()
{
   if (!coe_buff)
   {
      CANFrame msg;
      return msg;
   }

   CANFrame* msg = coe_buff->get();

   return *msg;
}

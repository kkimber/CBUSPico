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

#include "CBUS.h"
#include "SystemTick.h"
#include "CBUSUtil.h"

#include <pico/stdlib.h>
#include <new>
#include <cstring>
#include <cstdlib>

// forward function declarations
void makeHeader_impl(CANFrame &msg, uint8_t id, uint8_t priority = 0x0b);

//
/// construct a CBUS object with a CBUSConfig object that the user provides.
/// note that this CBUSConfig object must have a lifetime longer than the CBUS object.
//

CBUSbase::CBUSbase(CBUSConfig &config) : m_numMsgsSent{0x0L},
                                         m_numMsgsRcvd{0x0UL},
                                         m_msg{},
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

void CBUSbase::setSLiM(void)
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

//
/// send a WRACK (write acknowledge) message
//

bool CBUSbase::sendWRACK(void)
{
   // send a write acknowledgement response

   m_msg.len = 3;
   m_msg.data[0] = OPC_WRACK;
   m_msg.data[1] = highByte(m_moduleConfig.getNodeNum());
   m_msg.data[2] = lowByte(m_moduleConfig.getNodeNum());

   return sendMessage(m_msg);
}

//
/// send a CMDERR (command error) message
//

bool CBUSbase::sendCMDERR(uint8_t cerrno)
{
   // send a command error response

   m_msg.len = 4;
   m_msg.data[0] = OPC_CMDERR;
   m_msg.data[1] = highByte(m_moduleConfig.getNodeNum());
   m_msg.data[2] = lowByte(m_moduleConfig.getNodeNum());
   m_msg.data[3] = cerrno;

   return sendMessage(m_msg);
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

void CBUSbase::CANenumeration(void)
{
   // initiate CAN bus enumeration cycle, either due to ENUM opcode, ID clash, or user button press

   // DEBUG_SERIAL << F("> beginning self-enumeration cycle") << endl;

   // set global variables
   m_bCANenum = true;                    // we are enumerating
   CANenumTime = SystemTick::GetMilli(); // the cycle start time
   memset(m_enumResponses, 0, sizeof(m_enumResponses));

   // send zero-length RTR frame
   m_msg.len = 0;
   sendMessage(m_msg, true, false); // fixed arg order in v 1.1.4, RTR - true, ext = false
}

//
/// initiate the transition from SLiM to FLiM mode
//
void CBUSbase::initFLiM(void)
{
   // DEBUG_SERIAL << F("> initiating FLiM negotation") << endl;

   indicateMode(MODE_CHANGING);

   m_bModeChanging = true;
   timeOutTimer = SystemTick::GetMilli();

   // send RQNN message with current NN, which may be zero if a virgin/SLiM node
   m_msg.len = 3;
   m_msg.data[0] = OPC_RQNN;
   m_msg.data[1] = highByte(m_moduleConfig.getNodeNum());
   m_msg.data[2] = lowByte(m_moduleConfig.getNodeNum());
   sendMessage(m_msg);
}

//
/// revert from FLiM to SLiM mode
//
void CBUSbase::revertSLiM(void)
{
   // DEBUG_SERIAL << F("> reverting to SLiM mode") << endl;

   // send NNREL message
   m_msg.len = 3;
   m_msg.data[0] = OPC_NNREL;
   m_msg.data[1] = highByte(m_moduleConfig.getNodeNum());
   m_msg.data[2] = lowByte(m_moduleConfig.getNodeNum());

   sendMessage(m_msg);
   setSLiM();
}

//
/// change or re-confirm node number
//
void CBUSbase::renegotiate(void)
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
   uint8_t evindex;
   uint8_t evval;

   // start bus enumeration if required
   if (m_bEnumerationRequired)
   {
      m_bEnumerationRequired = false;
      CANenumeration();
   }

   //
   // process switch operations
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

   while ((available() || (m_coeObj != nullptr && m_coeObj->available())) && mcount < num_messages)
   {
      ++mcount;

      // at least one CAN frame is available in either the reception buffer or the COE buffer
      // retrieve the next one

      if (m_coeObj != nullptr && m_coeObj->available())
      {
         m_msg = m_coeObj->get();
      }
      else
      {
         m_msg = getNextMessage();
      }

      // extract OPC, NN, EN
      uint32_t opc = m_msg.data[0];
      uint32_t nn = (m_msg.data[1] << 8) + m_msg.data[2];
      uint32_t en = (m_msg.data[3] << 8) + m_msg.data[4];

      //
      /// extract the CANID() of the sending module
      //

      uint8_t remoteCANID = getCANID(m_msg.id);

      //
      /// if registered, call the user handler with this new frame
      //

      if (frameHandler != nullptr)
      {
         // check if incoming opcode is in the user list, if list length > 0
         if (m_numOpcodes > 0)
         {
            for (uint8_t i = 0; i < m_numOpcodes; i++)
            {
               if (opc == m_opcodes[i])
               {
                  frameHandler(m_msg);
                  break;
               }
            }
         }
         else
         {
            frameHandler(m_msg);
         }
      }

      /// pulse the green LED
      m_ledGrn.pulse();

      // is this a CANID() enumeration request from another node (RTR set) ?
      if (m_msg.rtr)
      {
         // DEBUG_SERIAL << F("> CANID() enumeration RTR from CANID() = ") << remoteCANID << endl;
         // send an empty message to show our CANID()
         m_msg.len = 0;
         sendMessage(m_msg);
         continue;
      }

      //
      /// set flag if we find a CANID() conflict with the frame's producer
      /// doesn't apply to RTR or zero-length frames, so as not to trigger an enumeration loop
      //

      if (remoteCANID == m_moduleConfig.getCANID() && m_msg.len > 0)
      {
         // DEBUG_SERIAL << F("> CAN id clash, enumeration required") << endl;
         m_bEnumerationRequired = true;
      }

      // is this an extended frame ? we currently ignore these as bootloader, etc data may confuse us !
      if (m_msg.ext)
      {
         // DEBUG_SERIAL << F("> extended frame ignored, from CANID() = ") << remoteCANID << endl;
         continue;
      }

      // are we enumerating CANID()s ?
      if (m_bCANenum && m_msg.len == 0)
      {
         // store this response in the responses array
         if (remoteCANID > 0)
         {
            // fix to correctly record the received CANID()
            bitWrite(m_enumResponses[(remoteCANID / 16)], remoteCANID % 8, 1);
            // DEBUG_SERIAL << F("> stored CANID() ") << remoteCANID << F(" at index = ") << (remoteCANID / 8) << F(", bit = ") << (remoteCANID % 8) << endl;
         }

         continue;
      }

      //
      /// process the message opcode
      /// if we got this far, it's a standard CAN frame (not extended, not RTR) with a data payload length > 0
      //

      if (m_msg.len > 0)
      {
         uint8_t index;

         switch (opc)
         {

         case OPC_ACON:
         case OPC_ACON1:
         case OPC_ACON2:
         case OPC_ACON3:

         case OPC_ACOF:
         case OPC_ACOF1:
         case OPC_ACOF2:
         case OPC_ACOF3:

         case OPC_ARON:
         case OPC_AROF:

            // lookup this accessory event in the event table and call the user's registered callback function
            if (eventHandler || eventHandlerEx)
            {
               processAccessoryEvent(nn, en, (opc % 2 == 0));
            }

            break;

         case OPC_ASON:
         case OPC_ASON1:
         case OPC_ASON2:
         case OPC_ASON3:

         case OPC_ASOF:
         case OPC_ASOF1:
         case OPC_ASOF2:
         case OPC_ASOF3:

            // lookup this accessory event in the event table and call the user's registered callback function
            if (eventHandler || eventHandlerEx)
            {
               processAccessoryEvent(0, en, (opc % 2 == 0));
            }

            break;

         case OPC_RQNP:
            // RQNP message - request for node paramters -- does not contain a NN or EN, so only respond if we
            // are in transition to FLiM
            // DEBUG_SERIAL << F("> RQNP -- request for node params during FLiM transition for NN = ") << nn << endl;

            // only respond if we are in transition to FLiM mode
            if (m_bModeChanging == true)
            {

               // DEBUG_SERIAL << F("> responding to RQNP with PARAMS") << endl;

               // respond with PARAMS message
               m_msg.len = 8;
               m_msg.data[0] = OPC_PARAMS;                            // opcode
               m_msg.data[1] = m_pModuleParams->param[IDX_MANUFR_ID]; // manf code -- MERG
               m_msg.data[2] = m_pModuleParams->param[IDX_MINOR_VER]; // minor code ver
               m_msg.data[3] = m_pModuleParams->param[IDX_MODULE_ID]; // module ident
               m_msg.data[4] = m_pModuleParams->param[IDX_NO_EVENTS]; // number of events
               m_msg.data[5] = m_pModuleParams->param[IDX_EVS_PR_EV]; // events vars per event
               m_msg.data[6] = m_pModuleParams->param[IDX_MXNUM_NVS]; // number of NVs
               m_msg.data[7] = m_pModuleParams->param[IDX_MAJOR_VER]; // major code ver
               // final param[8] = node flags is not sent here as the max message payload is 8 bytes (0-7)
               sendMessage(m_msg);
            }

            break;

         case OPC_RQNPN:
            // RQNPN message -- request parameter by index number
            // index 0 = number of params available;
            // respond with PARAN

            if (nn == m_moduleConfig.getNodeNum())
            {

               uint8_t paran = m_msg.data[3];

               // DEBUG_SERIAL << F("> RQNPN request for parameter # ") << paran << F(", from nn = ") << nn << endl;

               if (paran <= m_pModuleParams->param[IDX_NO_PARAMS])
               {

                  paran = m_msg.data[3];

                  m_msg.len = 5;
                  m_msg.data[0] = OPC_PARAN;
                  // _msg.data[1] = highByte(module_config.nodeNum);
                  // _msg.data[2] = lowByte(module_config.nodeNum);
                  m_msg.data[3] = paran;
                  m_msg.data[4] = m_pModuleParams->param[paran];
                  sendMessage(m_msg);
               }
               else
               {
                  // DEBUG_SERIAL << F("> RQNPN - param #") << paran << F(" is out of range !") << endl;
                  sendCMDERR(9);
               }
            }

            break;

         case OPC_SNN:
            // received SNN - set node number
            // DEBUG_SERIAL << F("> received SNN with NN = ") << nn << endl;

            if (m_bModeChanging)
            {
               // DEBUG_SERIAL << F("> buf[1] = ") << _msg.data[1] << ", buf[2] = " << _msg.data[2] << endl;

               // save the NN
               // module_config.setNodeNum((_msg.data[1] << 8) + _msg.data[2]);
               m_moduleConfig.setNodeNum(nn);

               // respond with NNACK
               m_msg.len = 3;
               m_msg.data[0] = OPC_NNACK;
               // _msg.data[1] = highByte(module_config.nodeNum);
               // _msg.data[2] = lowByte(module_config.nodeNum);

               sendMessage(m_msg);

               // DEBUG_SERIAL << F("> sent NNACK for NN = ") << module_config.nodeNum << endl;

               // we are now in FLiM mode - update the configuration
               m_bModeChanging = false;
               m_moduleConfig.setFLiM(true);
               indicateFLiMMode(m_moduleConfig.getFLiM());

               // enumerate the CAN bus to allocate a free CAN ID
               CANenumeration();

               // DEBUG_SERIAL << F("> FLiM mode = ") << module_config.FLiM << F(", node number = ") << module_config.nodeNum << F(", CANID() = ") << module_config.CANID() << endl;
            }
            else
            {
               // DEBUG_SERIAL << F("> received SNN but not in transition") << endl;
            }

            break;

         case OPC_RQNN:
            // Another module has entered setup.
            // If we are in setup, abort as only one module can be in setup

            if (m_bModeChanging)
            {
               m_bModeChanging = false;
               indicateMode(m_moduleConfig.getFLiM());
               // respond with NNACK
               m_msg.len = 3;
               m_msg.data[0] = OPC_NNACK;
               m_msg.data[1] = highByte(m_moduleConfig.getNodeNum());
               m_msg.data[2] = lowByte(m_moduleConfig.getNodeNum());

               sendMessage(m_msg);
            }
            break;

         case OPC_CANID:
            // CAN -- set CANID()
            // DEBUG_SERIAL << F("> CANID() for nn = ") << nn << F(" with new CANID() = ") << _msg.data[3] << endl;

            if (nn == m_moduleConfig.getNodeNum())
            {
               // DEBUG_SERIAL << F("> setting my CANID() to ") << _msg.data[3] << endl;
               if (m_msg.data[3] < 1 || m_msg.data[3] > 99)
               {
                  sendCMDERR(7);
               }
               else
               {
                  m_moduleConfig.setCANID(m_msg.data[3]);
               }
            }

            break;

         case OPC_ENUM:
            // received ENUM -- start CAN bus self-enumeration
            // DEBUG_SERIAL << F("> ENUM message for nn = ") << nn << F(" from CANID() = ") << remoteCANID << endl;
            // DEBUG_SERIAL << F("> my nn = ") << module_config.nodeNum << endl;

            if (nn == m_moduleConfig.getNodeNum() && remoteCANID != m_moduleConfig.getCANID() && !m_bCANenum)
            {
               // DEBUG_SERIAL << F("> initiating enumeration") << endl;
               CANenumeration();
            }

            break;

         case OPC_NVRD:
            // received NVRD -- read NV by index
            if (nn == m_moduleConfig.getNodeNum())
            {
               uint8_t nvindex = m_msg.data[3];
               if (nvindex > m_moduleConfig.EE_NUM_NVS)
               {
                  sendCMDERR(10);
               }
               else
               {
                  // respond with NVANS
                  m_msg.len = 5;
                  m_msg.data[0] = OPC_NVANS;
                  // _msg.data[1] = highByte(module_config.nodeNum);
                  // _msg.data[2] = lowByte(module_config.nodeNum);
                  m_msg.data[4] = m_moduleConfig.readNV(nvindex);
                  sendMessage(m_msg);
               }
            }

            break;

         case OPC_NVSET:
            // received NVSET -- set NV by index
            // DEBUG_SERIAL << F("> received NVSET for nn = ") << nn << endl;

            if (nn == m_moduleConfig.getNodeNum())
            {
               if (m_msg.data[3] > m_moduleConfig.EE_NUM_NVS)
               {
                  sendCMDERR(10);
               }
               else
               {
                  // update EEPROM for this NV -- NVs are indexed from 1, not zero
                  m_moduleConfig.writeNV(m_msg.data[3], m_msg.data[4]);
                  // respond with WRACK
                  sendWRACK();
                  // DEBUG_SERIAL << F("> set NV ok") << endl;
               }
            }

            break;

         case OPC_NNLRN:
            // received NNLRN -- place into learn mode
            // DEBUG_SERIAL << F("> NNLRN for node = ") << nn << F(", learn mode on") << endl;

            if (nn == m_moduleConfig.getNodeNum())
            {
               m_bLearn = true;
               // DEBUG_SERIAL << F("> set lean mode ok") << endl;
               // set bit 5 in parameter 8
               bitSet(m_pModuleParams->param[IDX_MOD_FLAGS], 5);
            }

            break;

         case OPC_EVULN:
            // received EVULN -- unlearn an event, by event number
            // en = (_msg.data[3] << 8) + _msg.data[4];
            // DEBUG_SERIAL << F("> EVULN for nn = ") << nn << F(", en = ") << en << endl;

            // we must be in learn mode
            if (m_bLearn == true)
            {
               // DEBUG_SERIAL << F("> searching for existing event to unlearn") << endl;

               // search for this NN and EN pair
               index = m_moduleConfig.findExistingEvent(nn, en);

               if (index < m_moduleConfig.EE_MAX_EVENTS)
               {

                  // DEBUG_SERIAL << F("> deleting event at index = ") << index << F(", evs ") << endl;
                  m_moduleConfig.cleareventEEPROM(index);

                  // update hash table
                  m_moduleConfig.updateEvHashEntry(index);

                  // respond with WRACK
                  sendWRACK();
               }
               else
               {
                  // DEBUG_SERIAL << F("> did not find event to unlearn") << endl;
                  // respond with CMDERR
                  sendCMDERR(10);
               }

            } // if in learn mode

            break;

         case OPC_NNULN:
            // received NNULN -- exit from learn mode

            if (nn == m_moduleConfig.getNodeNum())
            {
               m_bLearn = false;
               // DEBUG_SERIAL << F("> NNULN for node = ") << nn << F(", learn mode off") << endl;
               // clear bit 5 in parameter 8
               bitClear(m_pModuleParams->param[IDX_MOD_FLAGS], 5);
            }

            break;

         case OPC_RQEVN:
            // received RQEVN -- request for number of stored events
            // DEBUG_SERIAL << F("> RQEVN -- number of stored events for nn = ") << nn << endl;

            if (nn == m_moduleConfig.getNodeNum())
            {

               // respond with 0x74 NUMEV
               m_msg.len = 4;
               m_msg.data[0] = OPC_NUMEV;
               // _msg.data[1] = highByte(module_config.nodeNum);
               // _msg.data[2] = lowByte(module_config.nodeNum);
               m_msg.data[3] = m_moduleConfig.numEvents();

               sendMessage(m_msg);
            }

            break;

         case OPC_NERD:
            // request for all stored events
            // DEBUG_SERIAL << F("> NERD : request all stored events for nn = ") << nn << endl;

            if (nn == m_moduleConfig.getNodeNum())
            {
               m_msg.len = 8;
               m_msg.data[0] = OPC_ENRSP;                             // response opcode
               m_msg.data[1] = highByte(m_moduleConfig.getNodeNum()); // my NN hi
               m_msg.data[2] = lowByte(m_moduleConfig.getNodeNum());  // my NN lo

               for (uint8_t i = 0; i < m_moduleConfig.EE_MAX_EVENTS; i++)
               {

                  if (m_moduleConfig.getEvTableEntry(i) != 0)
                  {
                     // it's a valid stored event

                     // read the event data from EEPROM
                     // construct and send a ENRSP message
                     m_moduleConfig.readEvent(i, &m_msg.data[3]);
                     m_msg.data[7] = i; // event table index

                     // DEBUG_SERIAL << F("> sending ENRSP reply for event index = ") << i << endl;
                     sendMessage(m_msg);
                     sleep_ms(10);

                  } // valid stored ev
               }    // loop each ev
            }       // for me

            break;

         case OPC_REVAL:
            // received REVAL -- request read of an event variable by event index and ev num
            // respond with NEVAL

            if (nn == m_moduleConfig.getNodeNum())
            {

               if (m_moduleConfig.getEvTableEntry(m_msg.data[3]) != 0)
               {

                  m_msg.len = 6;
                  m_msg.data[0] = OPC_NEVAL;
                  // _msg.data[1] = highByte(module_config.nodeNum);
                  // _msg.data[2] = lowByte(module_config.nodeNum);
                  m_msg.data[5] = m_moduleConfig.getEventEVval(m_msg.data[3], m_msg.data[4]);
                  sendMessage(m_msg);
               }
               else
               {

                  // DEBUG_SERIAL << F("> request for invalid event index") << endl;
                  sendCMDERR(6);
               }
            }

            break;

         case OPC_NNCLR:
            // NNCLR -- clear all stored events

            if (m_bLearn == true && nn == m_moduleConfig.getNodeNum())
            {

               // DEBUG_SERIAL << F("> NNCLR -- clear all events") << endl;

               for (uint8_t e = 0; e < m_moduleConfig.EE_MAX_EVENTS; e++)
               {
                  m_moduleConfig.cleareventEEPROM(e);
               }

               // recreate the hash table
               m_moduleConfig.clearEvHashTable();
               // DEBUG_SERIAL << F("> cleared all events") << endl;

               sendWRACK();
            }

            break;

         case OPC_NNEVN:
            // request for number of free event slots

            if (m_moduleConfig.getNodeNum() == nn)
            {

               uint8_t free_slots = 0;

               // count free slots using the event hash table
               for (uint8_t i = 0; i < m_moduleConfig.EE_MAX_EVENTS; i++)
               {
                  if (m_moduleConfig.getEvTableEntry(i) == 0)
                  {
                     ++free_slots;
                  }
               }

               // DEBUG_SERIAL << F("> responding to to NNEVN with EVNLF, free event table slots = ") << free_slots << endl;
               // memset(&_msg, 0, sizeof(_msg));
               m_msg.len = 4;
               m_msg.data[0] = OPC_EVNLF;
               // _msg.data[1] = highByte(module_config.nodeNum);
               // _msg.data[2] = lowByte(module_config.nodeNum);
               m_msg.data[3] = free_slots;
               sendMessage(m_msg);
            }

            break;

         case OPC_QNN:
            // this is probably a config recreate -- respond with PNN if we have a node number
            // DEBUG_SERIAL << F("> QNN received, my node number = ") << module_config.nodeNum << endl;

            if (m_moduleConfig.getNodeNum() > 0)
            {
               // DEBUG_SERIAL << ("> responding with PNN message") << endl;
               m_msg.len = 6;
               m_msg.data[0] = OPC_PNN;
               m_msg.data[1] = highByte(m_moduleConfig.getNodeNum());
               m_msg.data[2] = lowByte(m_moduleConfig.getNodeNum());
               m_msg.data[3] = m_pModuleParams->param[IDX_MANUFR_ID];
               m_msg.data[4] = m_pModuleParams->param[IDX_MODULE_ID];
               m_msg.data[5] = m_pModuleParams->param[IDX_MOD_FLAGS];
               sendMessage(m_msg);
            }

            break;

         case OPC_RQMN:
            // request for node module name, excluding "CAN" prefix
            // sent during module transition, so no node number check
            // DEBUG_SERIAL << F("> RQMN received") << endl;

            // only respond if in transition to FLiM

            // respond with NAME
            if (m_bModeChanging)
            {
               m_msg.len = 8;
               m_msg.data[0] = OPC_NAME;
               memcpy(m_msg.data + 1, m_pModuleName, sizeof(module_name_t));
               sendMessage(m_msg);
            }

            break;

         case OPC_EVLRN:
            // received EVLRN -- learn an event
            evindex = m_msg.data[5];
            evval = m_msg.data[6];

            // DEBUG_SERIAL << endl << F("> EVLRN for source nn = ") << nn << F(", en = ") << en << F(", evindex = ") << evindex << F(", evval = ") << evval << endl;

            // we must be in learn mode
            if (m_bLearn == true)
            {

               // search for this NN, EN as we may just be adding an EV to an existing learned event
               // DEBUG_SERIAL << F("> searching for existing event to update") << endl;
               index = m_moduleConfig.findExistingEvent(nn, en);

               // not found - it's a new event
               if (index >= m_moduleConfig.EE_MAX_EVENTS)
               {
                  // DEBUG_SERIAL << F("> existing event not found - creating a new one if space available") << endl;
                  index = m_moduleConfig.findEventSpace();
               }

               // if existing or new event space found, write the event data

               if (index < m_moduleConfig.EE_MAX_EVENTS)
               {

                  // write the event to EEPROM at this location -- EVs are indexed from 1 but storage offsets start at zero !!
                  // DEBUG_SERIAL << F("> writing EV = ") << evindex << F(", at index = ") << index << F(", offset = ") << (module_config.EE_EVENTS_START + (index * module_config.EE_BYTES_PER_EVENT)) << endl;

                  // don't repeat this for subsequent EVs
                  if (evindex < 2)
                  {
                     m_moduleConfig.writeEvent(index, &m_msg.data[1]);
                  }

                  m_moduleConfig.writeEventEV(index, evindex, evval);

                  // recreate event hash table entry
                  // DEBUG_SERIAL << F("> updating hash table entry for idx = ") << index << endl;
                  m_moduleConfig.updateEvHashEntry(index);

                  // respond with WRACK
                  sendWRACK();
               }
               else
               {
                  // DEBUG_SERIAL << F("> no free event storage, index = ") << index << endl;
                  // respond with CMDERR
                  sendCMDERR(10);
               }
            }
            else
            { // bLearn == true
              // DEBUG_SERIAL << F("> error -- not in learn mode") << endl;
            }

            break;

         case OPC_AREQ:
            // AREQ message - request for node state, only producer nodes

            if ((m_msg.data[1] == highByte(m_moduleConfig.getNodeNum())) && (m_msg.data[2] == lowByte(m_moduleConfig.getNodeNum())))
            {
               eventHandler(0, m_msg);
            }

            break;

         case OPC_BOOT:
            // boot mode
            break;

         case OPC_RSTAT:
            // command station status -- not applicable to accessory modules
            break;

            // case OPC_ARST:
            // system reset ... this is not what I thought it meant !
            // module_config.reboot();
            // break;

         case OPC_DTXC:
            // CBUS long message
            if (longMessageHandler != nullptr)
            {
               longMessageHandler->processReceivedMessageFragment(m_msg);
            }
            break;

         default:
            // unknown or unhandled OPC
            // DEBUG_SERIAL << F("> opcode 0x") << _HEX(opc) << F(" is not currently implemented")  << endl;
            break;
         }
      }
      else
      {
         // DEBUG_SERIAL << F("> oops ... zero - length frame ?? ") << endl;
      }
   } // while messages available

   // check CAN bus enumeration timer
   checkCANenum();

   //
   /// check 30 sec timeout for SLiM/FLiM negotiation with FCU
   //

   if (m_bModeChanging && ((SystemTick::GetMilli() - timeOutTimer) >= 30000))
   {

      // DEBUG_SERIAL << F("> timeout expired, FLiM = ") << FLiM << F(", mode change = ") << bModeChanging << endl;
      indicateFLiMMode(m_moduleConfig.getFLiM());
      m_bModeChanging = false;
   }

   // DEBUG_SERIAL << F("> end of opcode processing, time = ") << (micros() - mtime) << "us" << endl;

   //
   /// end of CBUS message processing
   //
}

void CBUSbase::checkCANenum(void)
{
   //
   /// check the 100ms CAN enumeration cycle timer
   //

   uint8_t selected_id = 1; // default if no responses from other modules

   // if (bCANenum && !bCANenumComplete && (SystemTick::GetMilli() - CANenumTime) >= 100) {
   if (m_bCANenum && (SystemTick::GetMilli() - CANenumTime) >= 100)
   {

      // enumeration timer has expired -- stop enumeration and process the responses

      // DEBUG_SERIAL << F("> enum cycle complete at ") << SystemTick::GetMilli() << F(", start = ") << CANenumTime << F(", duration = ") << (SystemTick::GetMilli() - CANenumTime) << endl;
      // DEBUG_SERIAL << F("> processing received responses") << endl;

      // iterate through the 128 bit field
      for (uint8_t i = 0; i < 16; i++)
      {

         // ignore if this uint8_t is all 1's -> there are no unused IDs in this group of numbers
         if (m_enumResponses[i] == 0xff)
         {
            continue;
         }

         // for each bit in the uint8_t
         for (uint8_t b = 0; b < 8; b++)
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
               // DEBUG_SERIAL << F("> bit ") << b << F(" of uint8_t ") << i << F(" is not set, first free CAN ID = ") << selected_id << endl;
               // i = 16; // ugh ... but probably better than a goto :)
               // but using a goto saves 4 bytes of program size ;)
               goto check_done;
            }
         }
      }

   check_done:

      // DEBUG_SERIAL << F("> enumeration responses = ") << enums << F(", lowest available CAN id = ") << selected_id << endl;

      // bCANenumComplete = true;
      m_bCANenum = false;
      CANenumTime = 0UL;

      // store the new CAN ID
      m_moduleConfig.setCANID(selected_id);

      // send NNACK
      m_msg.len = 3;
      m_msg.data[0] = OPC_NNACK;
      m_msg.data[1] = highByte(m_moduleConfig.getNodeNum());
      m_msg.data[2] = lowByte(m_moduleConfig.getNodeNum());
      sendMessage(m_msg);
   }
}

//
/// for accessory event messages, lookup the event in the event table and call the user's registered event handler function
//

void CBUSbase::processAccessoryEvent(uint32_t nn, uint32_t en, bool is_on_event)
{
   // try to find a matching stored event -- match on nn, en
   uint8_t index = m_moduleConfig.findExistingEvent(nn, en);

   // call any registered event handler

   if (index < m_moduleConfig.EE_MAX_EVENTS)
   {
      if (eventHandler != nullptr)
      {
         eventHandler(index, m_msg);
      }
      else if (eventHandlerEx != nullptr)
      {
         eventHandlerEx(index, m_msg, is_on_event,
                        ((m_moduleConfig.EE_NUM_EVS > 0) ? m_moduleConfig.getEventEVval(index, 1) : 0));
      }
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

bool CBUScoe::available(void)
{
   if (!coe_buff)
   {
      return false;
   }

   return coe_buff->available();
}

CANFrame CBUScoe::get(void)
{
   CANFrame msg;

   if (!coe_buff)
   {
      return msg;
   }

   msg = *coe_buff->get();

   return msg;
}

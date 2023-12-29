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

// CBUS library header files

#include "CBUSACAN2040.h" // CAN controller and CBUS class
#include "CBUSSwitch.h"   // pushbutton switch
#include "CBUSLED.h"      // CBUS LEDs
#include "CBUSConfig.h"   // module configuration
#include "CBUSParams.h"   // CBUS parameters
#include "cbusdefs.h"     // MERG CBUS constants
#include "CBUSUtil.h"     // Utility macros

#include <cstdio>
#include <pico/stdlib.h>


// constants
const uint8_t VER_MAJ = 1;    // code major version
const char VER_MIN = 'a';     // code minor version
const uint8_t VER_BETA = 0;   // code beta sub-version
const uint8_t MODULE_ID = 99; // CBUS module type

// Map CBUS LED's switch to HW
const uint8_t LED_GRN = 21; // CBUS green SLiM LED pin
const uint8_t LED_YLW = 20; // CBUS yellow FLiM LED pin
const uint8_t SWITCH0 = 17; // CBUS push button switch pin

const uint8_t CAN_RX = 11; // CAN2040 Rx pin
const uint8_t CAN_TX = 12; // CAN2040 Tx pin

// CBUS objects
CBUSConfig module_config; // configuration object

// Construct CBUS Object and assign the module configuration
CBUSACAN2040 CBUS(module_config);

// module objects
CBUSSwitch moduleSwitch; // an example switch as input
CBUSLED moduleLED;       // an example LED as output

// module name, must be 7 characters, space padded.
uint8_t mname[7] = {'1', 'I', 'N', '1', 'O', 'U', 'T'};

// forward function declarations
void eventhandler(uint8_t index, CANFrame *msg);
void processModuleSwitchChange(void);

//
/// setup CBUS - runs once at power on from setup()
//

void setupCBUS()
{
   // set config layout parameters
   module_config.EE_NVS_START = 10;
   module_config.EE_NUM_NVS = 10;
   module_config.EE_EVENTS_START = 20;
   module_config.EE_MAX_EVENTS = 32;
   module_config.EE_NUM_EVS = 1;
   module_config.EE_BYTES_PER_EVENT = (module_config.EE_NUM_EVS + 4);

   // initialise and load configuration
   module_config.setEEPROMtype(EEPROM_INTERNAL);
   module_config.begin();

   // set module parameters
   CBUSParams params(module_config);
   params.setVersion(VER_MAJ, VER_MIN, VER_BETA);
   params.setModuleId(MODULE_ID);
   params.setFlags(PF_FLiM | PF_COMBI);

   // assign to CBUS
   CBUS.setParams(params.getParams());
   CBUS.setName(mname);

   // Get the internal CBUS UI objects
   CBUSLED& ledGrn = CBUS.getCBUSGreenLED();
   CBUSLED& ledYlw = CBUS.getCBUSYellowLED();
   CBUSSwitch& sw = CBUS.getCBUSSwitch();

   // set CBUS LED pins
   ledGrn.setPin(LED_GRN);
   ledYlw.setPin(LED_YLW);

   // initialise CBUS switch
   sw.setPin(SWITCH0, false);
   sw.run();

   // module reset - if switch is depressed at startup and module is in SLiM mode
   if (sw.isPressed() && !module_config.FLiM)
   {
      module_config.resetModule(ledGrn, ledYlw, sw);
   }

   // opportunity to set default NVs after module reset
   if (module_config.isResetFlagSet())
   {
      module_config.clearResetFlag();
   }

   // register our CBUS event handler, to receive event messages of learned events
   CBUS.setEventHandler(eventhandler);

   // set CBUS LEDs to indicate mode
   CBUS.indicateFLiMMode(module_config.FLiM);

   // configure and start CAN bus and CBUS message processing
   CBUS.setNumBuffers(16, 4);    // more buffers = more memory used, fewer = less
   CBUS.setPins(CAN_TX, CAN_RX); // select pins for CAN tx and rx

   if (!CBUS.begin())
   {
      // Init OK
   }
}

//
/// setup - runs once at power on
//

void setup()
{
   // Setup CBUS Library
   setupCBUS();

   // configure the module switch, attached to IO1 / pin 11, active low
   moduleSwitch.setPin(2, false);

   // configure the module LED, attached to Red LED / pin 22 via a 1K resistor
   moduleLED.setPin(22);
}

//
/// loop - runs forever
//

void loop()
{
   //
   /// do CBUS message, switch and LED processing
   //

   CBUS.process();

   //
   /// give the switch and LED code some time to run
   //

   moduleSwitch.run();
   moduleLED.run();

   //
   /// Check if switch changed and do any processing for this change.
   //

   processModuleSwitchChange();
}

//
/// test for switch input
/// as an example, it must be have been pressed or released for at least half a second
/// then send a long CBUS event with opcode ACON for on and ACOF for off
/// event number (EN) is 1

/// you can just watch for this event in FCU or JMRI, or teach it to another CBUS consumer module
//
void processModuleSwitchChange()
{
   if (moduleSwitch.stateChanged())
   {
      CANFrame msg;
      msg.id = module_config.CANID;
      msg.len = 5;
      msg.data[0] = (moduleSwitch.isPressed() ? OPC_ACON : OPC_ACOF);
      msg.data[1] = highByte(module_config.nodeNum);
      msg.data[2] = lowByte(module_config.nodeNum);
      msg.data[3] = 0;
      msg.data[4] = 1; // event number (EN) = 1

      if (CBUS.sendMessage(&msg))
      {
         // Sent OK
      }
   }
}

//
/// user-defined event processing function
/// called from the CBUS library when a learned event is received
/// it receives the event table index and the CAN frame
//

void eventhandler(uint8_t index, CANFrame *msg)
{
   // as an example, control an LED

   // read the value of the first event variable (EV) associated with this learned event
   uint8_t evval = module_config.getEventEVval(index, 1);

   // set the LED according to the opcode of the received event, if the first EV equals 0
   // we turn on the LED and if the first EV equals 1 we use the blink() method of the LED object as an example

   if (msg->data[0] == OPC_ACON)
   {
      if (evval == 0)
      {
         moduleLED.on();
      }
      else if (evval == 1)
      {
         moduleLED.blink();
      }
   }
   else if (msg->data[0] == OPC_ACOF)
   {
      moduleLED.off();
   }
}

// MODULE MAIN ENTRY

extern "C" int main(int argc, char *argv[])
{
   // Init stdio lib (only really required if UART logging etc.)
   stdio_init_all();

   // Initialize
   setup();

   // Run periodic processing - forever
   while (1)
   {
      loop();
      sleep_ms(1);
   }
}
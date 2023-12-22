// CBUS library header files

#include "CBUSACAN2040.h" // CAN controller and CBUS class
#include "CBUSSwitch.h"   // pushbutton switch
#include "CBUSLED.h"      // CBUS LEDs
#include "CBUSConfig.h"   // module configuration
#include "CBUSParams.h"   // CBUS parameters
#include "cbusdefs.h"     // MERG CBUS constants
#include "CBUSUtil.h"     // Utility macros

#include <cstdio>

// constants
const uint8_t VER_MAJ = 1;    // code major version
const char VER_MIN = 'a';  // code minor version
const uint8_t VER_BETA = 0;   // code beta sub-version
const uint8_t MODULE_ID = 99; // CBUS module type

const uint8_t LED_GRN = 8;  // CBUS green SLiM LED pin
const uint8_t LED_YLW = 9;  // CBUS yellow FLiM LED pin
const uint8_t SWITCH0 = 10; // CBUS push button switch pin

// CBUS objects
CBUSConfig module_config;          // configuration object
CBUSACAN2040 CBUS(&module_config); // CBUS object
CBUSLED ledGrn, ledYlw;            // two LED objects
CBUSSwitch pb_switch;              // switch object

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

   //Serial << F("> mode = ") << ((module_config.FLiM) ? "FLiM" : "SLiM") << F(", CANID = ") << module_config.CANID;
   //Serial << F(", NN = ") << module_config.nodeNum << endl;

   // set module parameters
   CBUSParams params(module_config);
   params.setVersion(VER_MAJ, VER_MIN, VER_BETA);
   params.setModuleId(MODULE_ID);
   params.setFlags(PF_FLiM | PF_COMBI);

   // assign to CBUS
   CBUS.setParams(params.getParams());
   CBUS.setName(mname);

   // set CBUS LED pins and assign to CBUS
   ledGrn.setPin(LED_GRN);
   ledYlw.setPin(LED_YLW);
   CBUS.setLEDs(ledGrn, ledYlw);

   // initialise CBUS switch and assign to CBUS
   pb_switch.setPin(SWITCH0, false);
   pb_switch.run();
   CBUS.setSwitch(pb_switch);

   // module reset - if switch is depressed at startup and module is in SLiM mode
   if (pb_switch.isPressed() && !module_config.FLiM)
   {
      //Serial << F("> switch was pressed at startup in SLiM mode") << endl;
      module_config.resetModule(ledGrn, ledYlw, pb_switch);
   }

   // opportunity to set default NVs after module reset
   if (module_config.isResetFlagSet())
   {
      //Serial << F("> module has been reset") << endl;
      module_config.clearResetFlag();
   }

   // register our CBUS event handler, to receive event messages of learned events
   CBUS.setEventHandler(eventhandler);

   // set CBUS LEDs to indicate mode
   CBUS.indicateMode(module_config.FLiM);

   // configure and start CAN bus and CBUS message processing
   CBUS.setNumBuffers(16, 4); // more buffers = more memory used, fewer = less
   CBUS.setPins(1, 2);        // select pins for CAN tx and rx

   if (!CBUS.begin())
   {
      //Serial << F("> can2040 init fail") << endl;
   }
   else
   {
      //Serial << F("> can2040 init ok") << endl;
   }
}

//
/// setup - runs once at power on
//

void setup()
{
   //Serial.begin(115200);
   //while (!Serial)
   //   ;
   //Serial << endl
   //       << endl
   //       << F("> ** CBUS 1 in 1 out v1 ** ") << __FILE__ << endl;

   setupCBUS();

   // configure the module switch, attached to pin 11, active low
   moduleSwitch.setPin(11, false);

   // configure the module LED, attached to pin 12 via a 1K resistor
   moduleLED.setPin(12);

   // end of setup
   //Serial << F("> ready") << endl
   //       << endl;
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

   // bottom of loop()
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
         //Serial << F("> sent CBUS message") << endl;
      }
      else
      {
         //Serial << F("> error sending CBUS message") << endl;
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

   //Serial << F("> event handler: index = ") << index << F(", opcode = 0x") << _HEX(msg->data[0]) << endl;

   // read the value of the first event variable (EV) associated with this learned event
   uint8_t evval = module_config.getEventEVval(index, 1);
   //Serial << F("> EV1 = ") << evval << endl;

   // set the LED according to the opcode of the received event, if the first EV equals 0
   // we turn on the LED and if the first EV equals 1 we use the blink() method of the LED object as an example

   if (msg->data[0] == OPC_ACON)
   {
      if (evval == 0)
      {
         //Serial << F("> switching the LED on") << endl;
         moduleLED.on();
      }
      else if (evval == 1)
      {
         //Serial << F("> switching the LED to blink") << endl;
         moduleLED.blink();
      }
   }
   else if (msg->data[0] == OPC_ACOF)
   {
      //Serial << F("> switching the LED off") << endl;
      moduleLED.off();
   }
}

// MAIN ENTRY 
extern "C" int main(int argc, char* argv[])
{
   setup();

   while(1)
   {
      loop();
   }
}
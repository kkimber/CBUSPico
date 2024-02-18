# CBUS Core

Core functionality for a CBUS module is provided by the base class, CBUSbase.  CBUS Modules are implemented by deriving a class from this base class, which allows the core CBUS functionality to be bound to a suitable external interface, normally CAN.

The base class requires the derived class to implement a number of pure virtual methods defined in the base class to orovide the interface between the CBUS functionality and the specific external inteface.  These methods are:

   * CBUSbase::begin
   * CBUSbase::available
   * CBUSbase::getNextMessage
   * CBUSbase::sendMessage
   * CBUSbase::reset

## begin

This method is responsible for initializing the CBUS module, typical resposibilities include allocating buffers for frames, setting up ISR etc.

## available

The available method is called to determine if data has been received from the external interface and is therefore available for processing. Should the available method indicate a frame is available, CBUSbase will retrieve messages via getNextMessage.

## getNextMessage

getNextMessage is called to retrieve the next stored frame from the external interface.

## sendMessage

sendMessage is called from CBUSbase to transmit messages on the external interface.

## reset

The reset method should reset the state of the CAN controller and associated memory buffers.

## Related Classes

* CBUSbase base class providing CBUS functionality

<div class="section_buttons">
 
| Previous                  |                                Next |
|:--------------------------|------------------------------------:|
| [Introduction](README.md) | [CBUS Module Parameters](params.md) |
 
</div>


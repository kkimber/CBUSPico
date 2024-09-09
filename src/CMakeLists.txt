set(CAN2040_SOURCE can2040/src)
set(CBUSDEFS cbusdefs)
set(CBUSPICOLIB CBUSPicoLib)

add_executable(CBUSPico
   # Source for CAN2040 PIO based CAN controller
   ${CAN2040_SOURCE}/can2040.c
   # Sources for CBUSPico libary 
   ${CBUSPICOLIB}/CBUSLED.cpp
   ${CBUSPICOLIB}/CBUSSwitch.cpp
   ${CBUSPICOLIB}/CBUSConfig.cpp
   ${CBUSPICOLIB}/CBUSParams.cpp
   ${CBUSPICOLIB}/SystemTick.cpp
   ${CBUSPICOLIB}/CBUSCircularBuffer.cpp
   ${CBUSPICOLIB}/CBUSLongMessage.cpp
   ${CBUSPICOLIB}/CBUS.cpp
   ${CBUSPICOLIB}/ACAN2040.cpp
   ${CBUSPICOLIB}/CBUSACAN2040.cpp
   # Example module using libary
   CBUS_1in1out.cpp
)

target_include_directories(CBUSPico PRIVATE
   ${CMAKE_CURRENT_SOURCE_DIR}
   ${CAN2040_SOURCE}
   ${CBUSDEFS}
   ${CBUSPICOLIB}
   ${CBUSPICOLIB}/GridConnectDummy
)

# Custom linker scipt to put CAN2040 code into RAM
pico_set_linker_script(CBUSPico ${CMAKE_CURRENT_SOURCE_DIR}/memmap_block.ld)

target_link_libraries(
   CBUSPico
   pico_stdlib
   #pico_stdio_semihosting
   pico_multicore
   cmsis_core
   hardware_i2c
   hardware_flash
)

pico_add_extra_outputs(CBUSPico)

set(CAN2040_SOURCE can2040/src)
set(CBUSDEFS cbusdefs)

add_executable(CBUSPico
   # Sources for CBUSPico libary 
   CBUSLED.cpp
   CBUSSwitch.cpp
   CBUSConfig.cpp
   CBUSParams.cpp
   SystemTick.cpp
   CBUSLongMessage.cpp
   CBUS.cpp
   ACAN2040.cpp
   CBUSACAN2040.cpp
   ${CAN2040_SOURCE}/can2040.c
   CBUS_1in1out.cpp
)

target_include_directories(CBUSPico PRIVATE
   ${CMAKE_CURRENT_SOURCE_DIR}
   ${CAN2040_SOURCE}
   ${CBUSDEFS}
)
target_link_libraries(CBUSPico pico_stdlib pico_multicore cmsis_core hardware_i2c hardware_flash)

pico_add_extra_outputs(CBUSPico)

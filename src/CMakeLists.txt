add_executable(CBUSPico
   # Sources for CBUSPico libary 
   CBUSLED.cpp
   CBUSSwitch.cpp
   SystemTick.cpp
   ExampleModule.c
)

target_include_directories(CBUSPico PRIVATE
   ${CMAKE_CURRENT_SOURCE_DIR}
   ${CAN2040_SOURCE}
   ${CBUSLIB_SOURCE}
   ${CBUSDEFS}
)
target_link_libraries(CBUSPico pico_stdlib pico_multicore cmsis_core hardware_i2c hardware_flash)

pico_add_extra_outputs(CBUSPico)


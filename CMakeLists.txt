# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NEVER EDIT THE NEXT LINES for Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 1.5.1)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set (PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in the SDK (must be before project definition)
include(pico_sdk_import.cmake)

project(CBUSPico C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Set source paths
set(CAN2040_SOURCE src/can2040/src)
set(CBUSDEFS src/cbusdefs)
set(CBUSPICOLIB src/CBUSPicoLib)
set(SRC src)

# Add executable. Default name is the project name, version 0.1
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
   # Example module using library
   ${SRC}/CBUS_1in1out.cpp
)

# Setup include paths
target_include_directories(CBUSPico PRIVATE
   ${SRC}
   ${CAN2040_SOURCE}
   ${CBUSDEFS}
   ${CBUSPICOLIB}
   ${CBUSPICOLIB}/GridConnectDummy
)

# Setup compiler options
target_compile_options(CBUSPico PRIVATE
   # can2040 has an unused param, -Wpedantic is a step too far !
   -Wall -Wextra -Werror -Wno-unused-parameter 
)

# Custom linker scipt to put CAN2040 code into RAM
pico_set_linker_script(CBUSPico ${CMAKE_CURRENT_SOURCE_DIR}/memmap_cbuspico.ld)

# pull in common dependencies
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

# create map/bin/hex file etc.
pico_add_extra_outputs(CBUSPico)

# add url via pico_set_program_url

# Look for the Doxygen package
find_package(Doxygen)

# Build documentation
if (DOXYGEN_FOUND)

   set(DOXYGEN_PROJECT_NAME "CBUS Pico")
   set(DOXYGEN_PROJECT_BRIEF "Library Documentation")
   set(DOXYGEN_PROJECT_NUMBER "1.0a")
   set(DOXYGEN_GENERATE_LATEX NO)
   set(DOXYGEN_GENERATE_HTML YES)
   set(DOXYGEN_EXTRACT_PACKAGE YES)
   set(DOXYGEN_INLINE_SIMPLE_STRUCTS YES)
   set(DOXYGEN_TYPEDEF_HIDES_STRUCT YES)
   set(DOXYGEN_EXTRACT_ALL YES)
   set(DOXYGEN_RECURSIVE NO)
   set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)
   set(DOXYGEN_PROJECT_LOGO ${PROJECT_SOURCE_DIR}/doc/logo.png)
   set(DOXYGEN_IMAGE_PATH ${PROJECT_SOURCE_DIR}/doc/)
   set(DOXYGEN_GENERATE_TREEVIEW YES)
   set(DOXYGEN_DISABLE_INDEX NO)
   set(DOXYGEN_FULL_SIDEBAR NO)
   set(DOXYGEN_HTML_HEADER ${PROJECT_SOURCE_DIR}/doc/header.html)
   set(DOXYGEN_HTML_EXTRA_STYLESHEET ${PROJECT_SOURCE_DIR}/doc/doxygen-awesome.css
      ${PROJECT_SOURCE_DIR}/doc/doxygen-awesome-sidebar-only.css
      ${PROJECT_SOURCE_DIR}/doc/doxygen-awesome-sidebar-only-darkmode-toggle.css)
   set(DOXYGEN_HTML_EXTRA_FILES ${PROJECT_SOURCE_DIR}/doc/doxygen-awesome-darkmode-toggle.js
      ${PROJECT_SOURCE_DIR}/doc/doxygen-awesome-fragment-copy-button.js)
   set(DOXYGEN_HTML_COLORSYTLE LIGHT)
   
   doxygen_add_docs(
      doxygen
      ${PROJECT_SOURCE_DIR}/README.md
      ${PROJECT_SOURCE_DIR}/doc/cbus.md
      ${PROJECT_SOURCE_DIR}/doc/params.md
      ${PROJECT_SOURCE_DIR}/doc/config.md
      ${PROJECT_SOURCE_DIR}/doc/ui.md
      ${PROJECT_SOURCE_DIR}/doc/canif.md
      ${PROJECT_SOURCE_DIR}/src
      ${PROJECT_SOURCE_DIR}/doc
      COMMENT "Generate CBUS Library Docs"
   )
else (DOXYGEN_FOUND)
   message("Doxygen need to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)


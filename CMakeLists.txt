cmake_minimum_required(VERSION 3.15)

# Define the target board
set (PICO_BOARD pico_w)

# Pull in the SDK (Must be before project definition)
include(pico_sdk_import.cmake)

# Define project
project(MERG-CBUSPico-Library C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

# Add sources
add_subdirectory(src)

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
   set(DOXYGEN_HTML_EXTRA_FILES ${PROJECT_SOURCE_DIR}/doc/doxygen-awesome-darkmode-toggle.js)
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


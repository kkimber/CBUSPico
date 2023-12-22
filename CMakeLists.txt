cmake_minimum_required(VERSION 3.15)

# Define the target board
set (PICO_BOARD pico)

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
find_package(Doxygen
   REQUIRED dot)

# Build documentation
if (DOXYGEN_FOUND)

   set(DOXYGEN_GENERATE_LATEX NO)
   set(DOXYGEN_GENERATE_HTML YES)
   set(DOXYGEN_EXTRACT_PACKAGE YES)
   set(DOXYGEN_INLINE_SIMPLE_STRUCTS YES)
   set(DOXYGEN_TYPEDEF_HIDES_STRUCT YES)
   set(DOXYGEN_RECURSIVE NO)
   set(DOXYGEN_PROJECT_LOGO ${PROJECT_SOURCE_DIR}/doc/logo.png)

   doxygen_add_docs(
      doxygen
      ${PROJECT_SOURCE_DIR}/src
      ${PROJECT_SOURCE_DIR}/doc
      COMMENT "Generate CBUS Library Docs"
   )
else (DOXYGEN_FOUND)
   message("Doxygen need to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)


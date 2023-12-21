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
find_package(Doxygen)

# Build documentation
if (DOXYGEN_FOUND)

set(DOXYGEN_GENERATE_LATEX NO)
set(DOXYGEN_GENERATE_HTML YES)
doxygen_add_docs(
   doxygen
   ${PROJECT_SOURCE_DIR}
)
   # set input and output files
   set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs_doxygen/Doxyfile.in)
   set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile.out)

   # request to configure the file
   configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
   message("Doxygen build started")

   # Note: do not put "ALL" - this builds docs together with application EVERY TIME!
   add_custom_target( docs
      COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
      COMMENT "Generating API documentation with Doxygen"
      VERBATIM )
else (DOXYGEN_FOUND)
   message("Doxygen need to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)


cmake_minimum_required(VERSION 2.8)
project(DepthColorRegistration)

# Linux pretty much has to have these sorts of things
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "4.7")
    set (CMAKE_CXX_COMPILER "g++-4.7")
  endif()
endif()

include_directories(${CMAKE_SOURCE_DIR}/src)

set (EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
if (NOT IS_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})
  file (MAKE_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})
endif ()

add_subdirectory (src)

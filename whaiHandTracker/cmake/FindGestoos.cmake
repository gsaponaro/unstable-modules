# Locate Gestoos SDK, set include paths and libraries
#
# See also:
#     http://www.gestoos.com/
#     http://www.fezoo.cat/
#
# Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>

cmake_minimum_required (VERSION 2.8)

set(Gestoos_DIR $ENV{Gestoos_DIR})

if(Gestoos_DIR)
    message(STATUS "Gestoos SDK found in ${Gestoos_DIR}")
else(Gestoos_DIR)
    message(FATAL_ERROR "Gestoos_DIR environment variable is undefined")
endif(Gestoos_DIR)

unset(FEZOOLIB CACHE)
if(${CMAKE_BUILD_TYPE} MATCHES "Debug")
    #message(STATUS "Building fezoolib in Debug")
    find_library(FEZOOLIB
                 NAMES fezoolib
                 PATHS "${Gestoos_DIR}/lib/debug"
                )
else(${CMAKE_BUILD_TYPE} MATCHES "Release")
    #message(STATUS "Building fezoolib in Release")
    find_library(FEZOOLIB
                 NAMES fezoolib
                 PATHS "${Gestoos_DIR}/lib/release"
                )
    add_definitions(-DNDEBUG)
endif()

if(DEFINED FEZOOLIB)
     message(STATUS "fezoolib found in ${FEZOOLIB}")
else(NOT DEFINED FEZOOLIB)
     message(FATAL_ERROR "fezoolib not found")
endif()
#message(STATUS "Found library ${FEZOOLIB}")

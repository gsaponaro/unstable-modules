# Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
#                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
# Authors: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# Based on actionsRenderingEngine by Carlo Ciliberto, Vadim Tikhanoff

cmake_minimum_required(VERSION 2.8.9)

project(gesturesRenderingEngine)

find_package(YARP REQUIRED)
find_package(ICUBcontrib REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

include(YarpIDL)
include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
include(ICUBcontribOptions)

icubcontrib_set_default_prefix()

yarp_add_idl(IDL_GEN_FILES ${PROJECT_NAME}.thrift)

include_directories(${PROJECT_SOURCE_DIR}/include ${YARP_INCLUDE_DIRS})

file(GLOB source src/*.cpp)
file(GLOB header include/*.h)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

add_executable(${PROJECT_NAME} ${source} ${header} ${PROJECT_NAME}.thrift ${IDL_GEN_FILES})

target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES})

yarp_install(TARGETS ${PROJECT_NAME} DESTINATION bin)

icubcontrib_add_uninstall_target()

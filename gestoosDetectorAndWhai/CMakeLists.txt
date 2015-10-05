# Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
# This program uses Gestoos SDK by Fezoo Labs:
#     http://www.gestoos.com/
#     http://www.fezoo.cat/

cmake_minimum_required (VERSION 2.8)

project(gestoosDetectorAndWhai)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Gestoos, fezoolib
find_package(Gestoos)

# OpenCV 3
find_package(OpenCV 3.0 PATHS /home/gsaponaro/software/opencv-3.0.0/build)
#message (STATUS "OpenCV ${OpenCV_VERSION} found, libraries ${OpenCV_LIBS}, include directories ${OpenCV_INCLUDE_DIRS}")
message (STATUS "OpenCV ${OpenCV_VERSION} found")

# Boost 1.54
set(Boost_LIBRARIES ${Gestoos_DIR}/lib/libboost_filesystem.so.1.54.0 ${Gestoos_DIR}/lib/libboost_serialization.so.1.54.0 ${Gestoos_DIR}/lib/libboost_thread.so.1.54.0 ${Gestoos_DIR}/lib/libboost_program_options.so.1.54.0 ${Gestoos_DIR}/lib/libboost_system.so.1.54.0)
set(Boost_INCLUDE_DIRS ${Gestoos_DIR}/include)
set(Boost_LIBRARY_DIRS ${Gestoos_DIR}/lib)
#message(STATUS "Boost 1.54 found: libraries ${Boost_LIBRARIES}, include directories ${Boost_INCLUDE_DIRS}")
message(STATUS "Boost 1.54 found")

# libcurl
find_library(CURL NAMES curl)
message(STATUS "libcurl found in ${CURL}")

# OpenNI2
set(OPENNI2_LIB ${Gestoos_DIR}/lib/libOpenNI2.so)
set(OPENNI2_INCLUDE ${Gestoos_DIR}/include/OpenNI2/Include)
message(STATUS "OpenNI found: libraries ${OPENNI2_LIB}, include directories ${OPENNI2_INCLUDE}")

# compile using C++11 if GCC version >= 4.7
execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion
                OUTPUT_VARIABLE GCC_VERSION)
if (GCC_VERSION VERSION_GREATER 4.7 OR GCC_VERSION VERSION_EQUAL 4.7)
    message(STATUS "Compiler Version >= 4.7  -->  compiling using c++11")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 
endif()

find_package(YARP)
if(YARP_FOUND)
    #message(STATUS "YARP found: libraries in ${YARP_LIBRARIES}, include directories in ${YARP_INCLUDE_DIRS}")
    message(STATUS "YARP found")
else(NOT YARP_FOUND)
    message(FATAL_ERROR "YARP not found")
endif(YARP_FOUND)

message(STATUS "Copying configuration files from ${CMAKE_SOURCE_DIR}/config to ${PROJECT_BINARY_DIR}")
file(COPY ${CMAKE_SOURCE_DIR}/config DESTINATION ${PROJECT_BINARY_DIR})

file(GLOB folder_source src/*.cpp)
file(GLOB folder_header include/iCub/*.hpp)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${Gestoos_DIR}/include
                    ${Gestoos_DIR}/include/opencv
                    ${Gestoos_DIR}/include/opencv2
                    ${YARP_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ${folder_source} ${folder_header})

target_link_libraries(${PROJECT_NAME}
                      ${FEZOOLIB}
                      ${Boost_LIBRARIES}
                      ${OpenCV_LIBS}
                      ${OPENNI2_LIB}
                      ${CURL}
                      ${LIBGLSL}
                      ${YARP_LIBRARIES})

#set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
#message(STATUS "Setting RPATH ${OPENNI2_ROOT}/Redist:${Boost_LIBRARY_DIRS}")  

set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "${OPENNI2_ROOT}/Redist;${Boost_LIBRARY_DIRS}")

# Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
# This program uses Gestoos SDK by Fezoo Labs:
#     http://www.gestoos.com/
#     http://www.fezoo.cat/

cmake_minimum_required (VERSION 2.8)

project(gestoosDetectorAndWhai)

set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

# copied from GestoosDetector, replacing ${CMAKE_SOURCE_DIR}
# with Gestoos_DIR=/home/gsaponaro/software/GestoosSDK_v0.6
set(Gestoos_DIR /home/gsaponaro/software/GestoosSDK_v0.6)
#OpenCV
find_package(OpenCV 3.0 PATHS /home/gsaponaro/software/opencv-3.0.0/build)
message (STATUS "Configured OpenCV ${OpenCV_VERSION}, libraries: ${OpenCV_LIBS}")
#Boost
message(STATUS "Setting Boost 1.54")
set(Boost_LIBRARIES ${Gestoos_DIR}/lib/libboost_filesystem.so.1.54.0 ${Gestoos_DIR}/lib/libboost_serialization.so.1.54.0 ${Gestoos_DIR}/lib/libboost_thread.so.1.54.0 ${Gestoos_DIR}/lib/libboost_program_options.so.1.54.0 ${Gestoos_DIR}/lib/libboost_system.so.1.54.0)
set(Boost_INCLUDE_DIRS ${Gestoos_DIR}/include)
set(Boost_LIBRARY_DIRS ${Gestoos_DIR}/lib)
message(STATUS "Using local version of Boost in ${Boost_INCLUDE_DIRS} and ${Boost_LIBRARIES}")
#Lib curl
find_library(CURL NAMES curl)
message(STATUS "Found libcurl: ${CURL}")
#OpenNI
set (OPENNI2_LIB ${Gestoos_DIR}/lib/libOpenNI2.so)
set (OPENNI2_INCLUDE ${Gestoos_DIR}/include/OpenNI2/Include)
message(STATUS "Found OpenNI: ${OPENNI2_INCLUDE}, ${OPENNI2_LIB}")

unset(FEZOOLIB CACHE)
message(STATUS "Build type ${CMAKE_BUILD_TYPE}")
if (${CMAKE_BUILD_TYPE} MATCHES "Debug")
    message(STATUS "Building Debug")
    find_library(FEZOOLIB
                 NAMES fezoolib
                 PATHS "${Gestoos_DIR}/lib/debug"
                )
else (${CMAKE_BUILD_TYPE} MATCHES "Release")
    message(STATUS "Building Release")
    find_library(FEZOOLIB
                 NAMES fezoolib
                 PATHS "${Gestoos_DIR}/lib/release"
                )
    add_definitions(-DNDEBUG)
endif()
# compile using C++11 if GCC version >= 4.7
execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion
                OUTPUT_VARIABLE GCC_VERSION)
if (GCC_VERSION VERSION_GREATER 4.7 OR GCC_VERSION VERSION_EQUAL 4.7)
    message(STATUS "Compiler Version >= 4.7  -->  compiling using c++11")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 
endif()
message(STATUS "Found library ${FEZOOLIB}")

find_package(YARP)

message(STATUS "Copying configuration files from ${CMAKE_SOURCE_DIR}/config to ${PROJECT_BINARY_DIR}")
#execute_process(COMMAND cp -r ${CMAKE_SOURCE_DIR}/config ${PROJECT_BINARY_DIR})
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

message(STATUS "Setting RPATH ${OPENNI2_ROOT}/Redist:${Boost_LIBRARY_DIRS}")  
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "${OPENNI2_ROOT}/Redist;${Boost_LIBRARY_DIRS}")

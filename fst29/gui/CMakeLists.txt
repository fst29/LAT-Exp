# Project's name
project(fst29_LAT)

# Specify the minimum version for CMake

cmake_minimum_required(VERSION 3.1)
set(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/build)
include_directories("${CMAKE_SOURCE_DIR}/include")
add_executable(guiDriver ${CMAKE_SOURCE_DIR}/guiDriver.cpp)


set(PHOENIX_LIBS_DIR ${CMAKE_SOURCE_DIR}/include/lib/raspberry)

target_link_libraries(guiDriver ${PHOENIX_LIBS_DIR}/libCTRE_Phoenix.so)
target_link_libraries(guiDriver ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixCCI.so)

find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
target_link_libraries(guiDriver ${WIRINGPI_LIBRARIES})

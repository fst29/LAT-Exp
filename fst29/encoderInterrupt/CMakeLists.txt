# Project's name
project(fst29_LAT)

# Specify the minimum version for CMake

cmake_minimum_required(VERSION 3.1)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/build)
add_executable(encoderInterrupt ${CMAKE_SOURCE_DIR}/encoderInterrupt.cpp)

find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
target_link_libraries(encoderInterrupt ${WIRINGPI_LIBRARIES})
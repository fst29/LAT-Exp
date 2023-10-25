# Project's name
project(fst29_LAT)

# Specify the minimum version for CMake

cmake_minimum_required(VERSION 3.1)
set(PROJECT_SOURCE_DIR home/pi/fst29)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/build)
include_directories("${CMAKE_SOURCE_DIR}/include")
add_executable(motorDriver ${CMAKE_SOURCE_DIR}/motorDriver.cpp)

set(PHOENIX_LIBS_DIR ${CMAKE_SOURCE_DIR}/include/lib/raspberry)

target_link_libraries(motorDriver ${PHOENIX_LIBS_DIR}/libCTRE_Phoenix.so)
target_link_libraries(motorDriver ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixCCI.so)


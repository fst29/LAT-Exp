# Project's name
project(fst29_LAT)

# Specify the minimum version for CMake

cmake_minimum_required(VERSION 3.1)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/build)
add_executable(cppreader ${CMAKE_SOURCE_DIR}/reader.cpp)
add_executable(cppwriter ${CMAKE_SOURCE_DIR}/writer.cpp)

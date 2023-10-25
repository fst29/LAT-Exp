# Project's name
PROJECT ( init C CXX )

# Specify the minimum version for CMake

cmake_minimum_required(VERSION 3.1)

add_definitions(-std=c++11)

# Set the output folder where your program will be created
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package (Threads REQUIRED)
find_package(SDL2 REQUIRED)

# The following folder will be included
include_directories("${PROJECT_SOURCE_DIR}/include")
# Encoder
add_executable(example ${PROJECT_SOURCE_DIR}/example.cpp)

if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf") # Jetson TK1 / Pi
    set(PHOENIX_LIBS_DIR ${CMAKE_SOURCE_DIR}/lib/raspberry)
elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu") # Jetson TX2
    set(PHOENIX_LIBS_DIR ${CMAKE_SOURCE_DIR}/lib/jetsontx)
else()
    set(PHOENIX_LIBS_DIR ${CMAKE_SOURCE_DIR}/lib/x86-64)
endif()

target_link_libraries(example ${PHOENIX_LIBS_DIR}/libCTRE_Phoenix.so)
target_link_libraries(example ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixCCI.so)

target_link_libraries(example Threads::Threads)
target_link_libraries(example ${SDL2_LIBRARIES})

find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
target_link_libraries(example ${WIRINGPI_LIBRARIES})

add_custom_command(TARGET example POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${PHOENIX_LIBS_DIR}
        $<TARGET_FILE_DIR:example>)

# Velocity control
add_executable(example_vel ${PROJECT_SOURCE_DIR}/example_vel.cpp)

if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf") # Jetson TK1 / Pi
    set(PHOENIX_LIBS_DIR ${CMAKE_SOURCE_DIR}/lib/raspberry)
elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu") # Jetson TX2
    set(PHOENIX_LIBS_DIR ${CMAKE_SOURCE_DIR}/lib/jetsontx)
else()
    set(PHOENIX_LIBS_DIR ${CMAKE_SOURCE_DIR}/lib/x86-64)
endif()

target_link_libraries(example_vel ${PHOENIX_LIBS_DIR}/libCTRE_Phoenix.so)
target_link_libraries(example_vel ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixCCI.so)

target_link_libraries(example_vel Threads::Threads)
target_link_libraries(example_vel ${SDL2_LIBRARIES})

find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
target_link_libraries(example_vel ${WIRINGPI_LIBRARIES})

add_custom_command(TARGET example_vel POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${PHOENIX_LIBS_DIR}
        $<TARGET_FILE_DIR:example_vel>)

#add_executable(example_cal ${CMAKE_SOURCE_DIR}/example_cal.cpp)

# find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
# target_link_libraries(example_cal ${WIRINGPI_LIBRARIES})

# target_link_libraries(example_cal ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_Phoenix.a)
# target_link_libraries(example_cal ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixCCI.a)
# target_link_libraries(example_cal ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixDiagnostics.a)
# target_link_libraries(example_cal ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixPlatform_socketcan.a)
# target_link_libraries(example_cal ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixCanutils.a)
# target_link_libraries(example_cal ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixCore.a)

# target_link_libraries(example_cal Threads::Threads)
# target_link_libraries(example_cal ${SDL2_LIBRARIES})

# add_executable(example_pos ${CMAKE_SOURCE_DIR}/example_pos.cpp)

# find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
# target_link_libraries(example_pos ${WIRINGPI_LIBRARIES})

# target_link_libraries(example_pos ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_Phoenix.a)
# target_link_libraries(example_pos ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixCCI.a)
# target_link_libraries(example_pos ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixDiagnostics.a)
# target_link_libraries(example_pos ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixPlatform_socketcan.a)
# target_link_libraries(example_pos ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixCanutils.a)
# target_link_libraries(example_pos ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixCore.a)

# target_link_libraries(example_pos Threads::Threads)
# target_link_libraries(example_pos ${SDL2_LIBRARIES})

# add_executable(example_vel ${CMAKE_SOURCE_DIR}/example_vel.cpp)

# find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
# target_link_libraries(example_vel ${WIRINGPI_LIBRARIES})

# target_link_libraries(example_vel ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_Phoenix.a)
# target_link_libraries(example_vel ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixCCI.a)
# target_link_libraries(example_vel ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixDiagnostics.a)
# target_link_libraries(example_vel ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixPlatform_socketcan.a)
# target_link_libraries(example_vel ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixCanutils.a)
# target_link_libraries(example_vel ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixCore.a)

# target_link_libraries(example_vel Threads::Threads)
# target_link_libraries(example_vel ${SDL2_LIBRARIES})

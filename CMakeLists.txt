cmake_minimum_required(VERSION 3.3.1)
project (adc)
set(CMAKE_BUILD_TYPE Release)

# Using c++11
list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}" )

# main adc program
include_directories(sensors)
set(SOURCES "altFilter.cpp"  "config.cpp"  "debug.cpp"  "ESCController.cpp"
    "main.cpp"  "pid.cpp")
add_subdirectory(sensors)
add_executable(adc ${SOURCES})
target_link_libraries(adc sensors pigpio)

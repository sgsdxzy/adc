cmake_minimum_required(VERSION 3.3.1)
project (adc)

list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}" )

include_directories(sensors)
file(GLOB SOURCES "sensors/*.cpp")
 
add_executable(adc main.cpp pid.cpp debug.cpp altFilter.cpp ${SOURCES})
target_link_libraries(adc bcm2835 pigpio pthread)

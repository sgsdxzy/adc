cmake_minimum_required(VERSION 3.3.1)
project (adc)

include_directories(sensors)
file(GLOB SOURCES "sensors/*.cpp")
 
add_executable(adc ${SOURCES})

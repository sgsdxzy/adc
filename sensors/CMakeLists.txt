cmake_minimum_required(VERSION 3.3.1)
project(sensors)

# Using c++11
#list( APPEND CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}" )

# libsensors
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(SOURCES "BMP085.cpp"  "GY87.cpp"  "HMC5883L.cpp"  "I2Cdev.c"
    "MPU6050.cpp")

add_library(sensors STATIC ${SOURCES})

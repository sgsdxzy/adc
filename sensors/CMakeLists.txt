cmake_minimum_required(VERSION 3.3.1)
project(sensors)
set(CMAKE_BUILD_TYPE Release)

# Using c++11
list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}" )

# libsensors
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(SOURCES "BMP085.cpp"  "GY87.cpp"  "HMC5883L.cpp"  "I2Cdev.cpp"
    "MPU6050_6Axis_MotionApps20.cpp"  "MPU6050.cpp")

add_library(sensors SHARED ${SOURCES})

target_link_libraries(sensors bcm2835 pigpio pthread)


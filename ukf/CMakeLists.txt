cmake_minimum_required(VERSION 2.8.7 FATAL_ERROR)
project(ukf)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(SOURCES "state.cpp" "dynamics.cpp" "sensors.cpp" "ukf.cpp")

add_library(ukf STATIC ${SOURCES})

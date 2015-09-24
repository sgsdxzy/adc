cmake_minimum_required(VERSION 3.3.1)
project (adc)

# Using c++11
list( APPEND CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}" )

include_directories(sensors)
add_subdirectory(sensors)

include_directories(ukf)
add_subdirectory(ukf)

set(SOURCES "Calibrator.cpp"  "Configuration.cpp"  "Debug.cpp"
    "ESCController.cpp" "PID.cpp" "adc.cpp")

# main adc program
add_executable(adc ${SOURCES})
target_link_libraries(adc sensors ukf pigpio pthread rt)

# MPU6050 calibration program
add_executable(calibrate "config.cpp" "debug.cpp" "gy87Calibrate.cpp")
target_link_libraries(calibrate sensors)

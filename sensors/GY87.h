#ifndef _GY87_H_
#define _GY87_H_

#include <atomic>
#include <pthread.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "HMC5883L.h"
#include "BMP085.h"

class GY87 {
    public:
        GY87() {};

        void initialize();
        void calibrate(int samples=100, int tolerance=10);
        void setOffset();
        bool testConnection();
        void startDMP();

        void updateMPU();
        void updateBMP(float seaLevelPressure);

        std::atomic<float> yaw, pitch, roll;           // [yaw, pitch, roll]
        std::atomic<float> heading;          // Magnetic heading.
        std::atomic<int16_t> gx, gy, gz; // Gyroscope data
        std::atomic<int16_t> axRelative, ayRelative, azRelative;
        std::atomic<int16_t> axAbsolute, ayAbsolute, azAbsolute;
        std::atomic<float> temperature;
        std::atomic<float> pressure;
        std::atomic<float> altitude;
        

    private:
        // Devices
        MPU6050 mpu;
        BMP085 bmp;

        // MPU6050+HMC5883L
        uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
        uint16_t fifoCount;     // count of all bytes currently in FIFO
        uint8_t fifoBuffer[64]; // FIFO storage buffer
        uint8_t BMPCounter;     // Only sample temperature at 1Hz

        pthread_mutex_t i2cMutex = PTHREAD_MUTEX_INITIALIZER; // Fix the lockup
};

#endif // _GY87_H_

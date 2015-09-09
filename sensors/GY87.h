#ifndef _GY87_H_
#define _GY87_H_

#include <pthread.h>
#include <pigpio.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "HMC5883L.h"
#include "BMP085.h"
#include "../debug.h"

class GY87 {
    public:
        GY87() {};

        void initialize();
        void setOffset(int16_t* gy87Offset);
        bool testConnection();
        void startDMP();
        void reset();

        void updateMPU()
        void updateBMP(float seaLevelPressure)

        // MPU6050
        Quaternion q;
        VectorInt16 aa;
        VectorInt16 aaReal;
        VectorInt16 aaWorld;
        VectorInt16 gyro;
        VectorFloat gravity;
        float ypr[3];
        int16_t mx, my, mz;
        float mh;
        // BMP180
        float temperature;
        float pressure;
        float baroAltitude;
        float sonicVelocity;

    private:
        // Devices
        MPU6050 mpu;
        BMP085 bmp;

        // MPU6050+HMC5883L
        uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
        uint16_t fifoCount;     // count of all bytes currently in FIFO
        uint8_t fifoBuffer[64]; // FIFO storage buffer
        uint8_t BMPCounter = 0;     // Only sample temperature at 1Hz

        pthread_mutex_t i2cMutex = PTHREAD_MUTEX_INITIALIZER; // Fix the lockup
};

#endif // _GY87_H_

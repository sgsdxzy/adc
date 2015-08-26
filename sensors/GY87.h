#ifndef _GY87_H_
#define _GY87_H_

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
        void updateHMC();
        void updateBMP();

        // Devices
        MPU6050 mpu;
        HMC5883L hmc;
        BMP085 bmp;

        // MPU6050
        uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
        uint16_t fifoCount;     // count of all bytes currently in FIFO
        uint8_t fifoBuffer[64]; // FIFO storage buffer
        Quaternion q;           // [w, x, y, z]         quaternion container
        VectorInt16 aa;         // [x, y, z]            accel sensor measurements
        int16_t gyro[3];        //To store gyro's measures
        VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
        VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
        VectorFloat gravity;    // [x, y, z]            gravity vector
        float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

        // HMC5883L
        int16_t mx, my, mz;     //To store magnetometer readings
        float heading;          // Simple magnetic heading.

        // BMP085 
        //TODO

};

#endif // _GY87_H_

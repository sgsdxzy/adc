#ifndef _GY87_H_
#define _GY87_H_

#include <pigpio.h>
#include "MPU6050.h"
#include "HMC5883L.h"
#include "BMP085.h"
#include "../config.h"
#include "../debug.h"
#include "../status.h"

class GY87 {
    public:
        GY87() {};

        void initialize(configuration* conf);
        bool testConnection();
        void reset();

        void getData(statusContainer* status);

        float seaLevelPressure;

    private:
        // Devices
        MPU6050 mpu;
        BMP085 bmp;
};

#endif // _GY87_H_

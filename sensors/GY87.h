#ifndef _GY87_H_
#define _GY87_H_

#include "MPU6050.h"
#include "HMC5883L.h"
#include "BMP085.h"
#include "../Configuration.h"
#include "../Status.h"
#include "../Debug.h"

class GY87 {
    public:
        GY87() {};

        void initialize(Configuration* config);
        bool testConnection();
        void reset();

        void getData(Status* status);

        float seaLevelPressure;

    private:
        // Devices
        MPU6050 mpu;
        BMP085 bmp;
};

#endif // _GY87_H_

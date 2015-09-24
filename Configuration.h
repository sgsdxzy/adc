#ifndef _CONFIGURATION_ADC_H_
#define _CONFIGURATION_ADC_H_

#include <iostream>
#include <Eigen/Dense>
#include "sensors/GY87.h"

using namespace Eigen;

class Configuration
{
    public:
        // System
        float dt = 0.005; // in seconds
        float g = 9.80151; // Beijing
        float seaLevelPressure = 101500;

        // GY87
        uint8_t MPU6050DLPFMode = MPU6050_DLPF_BW_42;
        int16_t MPU6050Rate = 200;
        uint8_t MPU6050GyroFsr = MPU6050_GYRO_FS_2000;
        uint8_t MPU6050AccelFSr = MPU6050_ACCEL_FS_2;
        int16_t MPU6050Offsets[6] = {-1685, 3937, 1803, 0, -17, 109};

        uint8_t HMC5883LSampleAveraging = HMC5883L_AVERAGING_8;
        uint8_t HMC5883LDataRate = HMC5883L_RATE_75;

        // Caliberator
        Matrix3f axisRotationMatrix;

        bool calibrateCompass = true;
        Vector3f calibrateCompassMax;
        Vector3f calibrateCompassMin;
        bool calibrateCompassEllipsoid = true;
        Vector3f calibrateCompassEllipsoidOffset;
        Matrix3f calibrateCompassEllipsoidMatrix;

        bool calibrateAccel = true;
        float calibrateAccelMin[3];
        float calibrateAccelMax[3];

        // UKF
        Vector3f accOffset;
        Vector3f magneticField;


        // ESC controller
        int controlled_esc[4] = {6, 13, 19, 26};
        int ESCSanity[4] = {12, 12, 12, 12};
        int ESCFrequency = 400; // Hz
        int ESCOutMin = 1200;
        int ESCOutMax = 1800;

        // PID system
        float ratePIDSystemConfig[3][7] =
        {
            {100, 1, 0, 200, -200, 400, -400},
            {300, 1, 0, 200, -200, 400, -400},
            {300, 1, 0, 200, -200, 400, -400}
        };
        float attitudePIDSystemConfig[3][7] = 
        {
            {100, 1, 0.03, 200, -200, 400, -400},
            {300, 1, 0.02, 200, -200, 400, -400},
            {300, 1, 0.02, 200, -200, 400, -400}
        };
        float ZPIDSystemConfig[4][7] =
        {
            {20, 20, 20, 25, 0, 600, 0}, 
            {20, 20, 20, 25, 0, 600, 0}, 

            {20, 20, 20, 25, 0, 600, 0}, 
            {20, 20, 20, 25, 0, 600, 0}, 
        };
        int yprtSanity[4] = {8, 8, 8, 3};
};

using std::ostream;
using std::istream;
using std::endl;

std::ostream& operator<<(std::ostream& stream, configuration const& data);
std::istream& operator>>(std::istream& stream, configuration& data);

#endif // _CONFIGURATION_ADC_H_

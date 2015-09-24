#ifndef _STATUS_H_
#define _STATUS_H_

#include <Eigen/Dense>

using namespace Eigen;

struct Status
{
        // GY87 raw data
        Vector3f accRaw;
        Vector3f gyroRaw;
        Vector3f compassRaw;
        float temperature;
        float pressure;
        // GY87 caulculated data
        float sonicVelocity;
        float baroAltitude;

        // Calibrated data from calibrator
        Vector3f accCal;
        Vector3f gyroCal;
        Vector3f compassCal;

        // TODO get these data
        Vector3f GPSPosition;
        Vector3f GPSVelocity;
        float sonarDistance;

        // Device data sanity checker and validator
        bool acc = true;
        bool gyro = true;
        bool compass = true;
        bool baro = true;
        bool GPSP = false;
        bool GPSV = false;
        bool sonar = false;

        // Optical flow
        // std::atomic<float> opticalVelocity[2]; // [vx, vy]

        // Starting conditions
        float startAttitude[3]; // [Yaw, pitch, roll]
        float startAltitude;
};

#endif // _STATUS_H_

#ifndef _STATUS_H_
#define _STATUS_H_

#include "RTMath.h"

struct statusContainer
{
        // GY87 raw data
        float accRaw[3]; // [ax, ay, az]
        float gyroRaw[3]; // [gx, gy, gz]
        float compassRaw[3]; // [mx, my, mz]
        float temperature;
        float pressure;
        // GY87 caulculated data
        float sonicVelocity;
        float baroAltitude;

        // Calibrated data from calibrator
        RTVector3 accCal;
        RTVector3 gyroCal;
        RTVector3 compassCal;



        float attitude[3]; // [Yaw, pitch, roll]
        float heading;          // Magnetic heading.
        float gyroscope[3]; // Gyroscope data [gx, gy, gz]
        float gravity[3]; // Gravity's component along each axis, normalized to 1
        float accRelative[3]; // axRelative, ayRelative, azRelative;
        float accAbsolute[3]; // axAbsolute, ayAbsolute, azAbsolute;

        // HC-SR04+
        float sonarAltitude;
        bool sonar; // Indicate whether sonar is in working range

        // Optical flow
        // std::atomic<float> opticalVelocity[2]; // [vx, vy]

        // Starting conditions
        float startAttitude[3]; // [Yaw, pitch, roll]
        float startHeading;
        float startAltitude;
};

#endif // _STATUS_H_

#ifndef _STATUS_H_
#define _STATUS_H_

#include <atomic>

// All in SI
class statusContainer
{
    public:
        // GY87
        std::atomic<float> attitude[3]; // [Yaw, pitch, roll]
        std::atomic<float> heading;          // Magnetic heading.
        std::atomic<float> gyroscope[3]; // Gyroscope data [gx, gy, gz]
        std::atomic<float> accRelative[3]; // axRelative, ayRelative, azRelative;
        std::atomic<float> accAbsolute[3]; // axAbsolute, ayAbsolute, azAbsolute;
        float temperature;
        float pressure;
        float sonicVelocity;
        float baroAltitude;

        // HC-SR04+
        float sonarAltitude;
        bool sonar; // Indicate whether sonar is in working range

        // Baro filter
        float baroFilterAltitude;
        float baroFilterVelocityZ;

        // Sonar filter 
        float sonarFilterAltitude;
        float sonarFilterVelocityZ;

        // Optical flow
        // std::atomic<float> opticalVelocity[2]; // [vx, vy]

        // Starting conditions
        float startAttitude[3]; // [Yaw, pitch, roll]
        float startHeading;
        float startAltitude;
};

#endif // _STATUS_H_

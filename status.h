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
        std::atomic<float> temperature;
        std::atomic<float> pressure;
        std::atomic<float> baroAltitude;

        // HC-SR04+
        std::atomic<float> sonarAltitude;

        // Baro filter
        std::atomic<float> baroFilterAltitude;
        std::atomic<float> baroFilterVelocityZ;

        // Sonar filter 
        std::atomic<float> sonarFilterAltitude;
        std::atomic<float> sonarFilterVelocityZ;

        // Optical flow
        std::atomic<float> opticalVelocity[2]; // [vx, vy]

        // Starting conditions
        float startAttitude[3]; // [Yaw, pitch, roll]
        float startHeading;
        float startAltitude;
};

#endif // _STATUS_H_

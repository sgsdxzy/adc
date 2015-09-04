#ifndef _PID_H_
#define _PID_H_

#include <pigpio.h>
#include "sensors/sensors.h"
#include "altFilter.h"

// All-in-one 4 axis PID
class PID
{
    public:
        PID();
        void initialize();
        // void newTarget(float yaw, float pitch, float roll, float height);
        void updateESC();

        int esc[4] = {0, 0, 0, 0}; // Output

        float yawPID[3] = {0, 0, 0}; // [Kp, Ki, Kd]
        float pitchPID[3] = {0, 0, 0};
        float rollPID[3] = {0, 0, 0};
        float heightPID[3] = {0, 0, 0};

        float target[4] = {0, 0, 0, 0}; // [yaw, pitch, roll, height]
        float errorI[4] = {0, 0, 0, 0}; // I part;
        // float prevTarget[4] = {0, 0, 0, 0};
        // float prevPosition[4] = {0, 0, 0, 0}; // Used to calculate D part. Using gyro data instead

        // float targetTime = 0; // Time elasped since target set, to dampen D effect
        // uint32_t prevTickTime; // Microsecond time at previous tick, to calculate I. Using system frequency should be enough. 
        float dt = 0.01; // in seconds

        int outMin = 1200;
        int outMax = 1800;
};

#endif // _PID_H_


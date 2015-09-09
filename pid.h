#ifndef _PID_H_
#define _PID_H_

#include <atomic>
#include "status.h"
#include "debug.h"

#define PID_RATE 0
#define PID_ATTITUDE 1

#define ALTITUDE_BARO 0
#define ALTITUDE_SONAR 1

// 1 axis PID
class PID
{
    public:
        void initialize(float pidConfig[7]); // [Kp, Ki, Kd, errorIMax, errorIMin, outMax, outMin]
        float update(float error, float errorD, float dt); // Get PID result by providing errorD
        //float update(float error, float dt); // Get PID result by internal errorD calculation

        float pid[3]; // [Kp, Ki, Kd]
        float errorI = 0;
        //float prevError = 0;
        float errorIMax, errorIMin;
        float outMax, outMin;
};

class PIDSystem
{
    public:
        void initialize(float ratePIDSystemConfig[3][7], float attitudePIDSystemConfig[3][7], float ZPIDSystemConfig[4][7]);
        void setRateTarget(int index, float target); // Setting ratePID[index]'s target
        void setRateTargets(float targets[3]); // Setting ratePIDs' target
        void setAttitudeTarget(int index, float target); // Setting attitudePID[index]'s target
        void setAttitudeTargets(float targets[3]); // Setting attitudePIDs' target
        void setVzTarget(float target, uint8_t type);
        void setAltitudeTarget(float target, uint8_t type);
        void update(statusContainer& status, float dt); // Update the whole PID system to get yprt results

        PID ratePID[3]; // [-gz, -gy, gx]
        PID attitudePID[3]; // [yaw, pitch, roll]
        PID VzPID[2]; // [baro, sonar]
        PID altitudePID[2];
        // PID velocityPID[2]; // [vx,vy]

        std::atomic<float> rateTargets[3];
        std::atomic<float> attitudeTargets[3];
        std::atomic<float> VzTarget; 
        std::atomic<float> altitudeTarget[2]; // [baro, sonar]

        std::atomic<uint8_t> PIDTypes[3] = {PID_ATTITUDE, PID_ATTITUDE, PID_ATTITUDE}; // What enabled PID is controlling YPR
        std::atomic<uint8_t> altPIDType = PID_ATTITUDE;
        std::atomic<uint8_t> altitudeType = ALTITUDE_BARO;

        int yprt[4]; // The final result
};

#endif // _PID_H_


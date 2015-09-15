#ifndef _PID_H_
#define _PID_H_

#include <atomic>
#include <cmath>
#include <pthread.h>
#include "status.h"

#define PID_RATE 0
#define PID_ATTITUDE 1

#define ALTITUDE_BARO 0
#define ALTITUDE_SONAR 1

float minusError(float target, float current);
float circularError(float target, float current);

// 1 axis PID
class PID
{
    public:
        void initialize(float dt, float pidConfig[7]); // [Kp, Ki, Kd, errorIMax, errorIMin, outMax, outMin]
        float update(); // Get PID result
        void reset();

        float target;
        float current;
        float errorD;

        float errorI = 0;

        float dt;
        float pid[3]; // [Kp, Ki, Kd]
        //float prevError = 0;
        float errorIMax, errorIMin;
        float outMax, outMin;
        float (*getError)(float target, float current) = minusError;

        PID* next = nullptr; // PID chain
};

class PIDSystem
{
    public:
        void initialize(float dt, int yprtSanity[4], float PIDSystemConfig[5][7]);
        void reset();
        void setAttitudeTarget(int index, float target); // Setting attitudePID[index]'s target
        void setAttitudeTargets(float targets[3]); // Setting attitudePIDs' target
        void setVzTarget(float target, uint8_t type);
        void setAltitudeTarget(float target, uint8_t type);
        void update(statusContainer& status); // Update the whole PID system to get yprt results

        int sanity[4]; // Control max trasition rate
        float dt;

        PID attitudePID[3]; // [yaw, pitch, roll]
        PID altitudePID[2]; // [baro, sonar]

        PID* enabled[4];

        int oldYPRT[4];
        int yprt[4]; // The final result

        pthread_mutex_t updating = PTHREAD_MUTEX_INITIALIZER;
};

#endif // _PID_H_


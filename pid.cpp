#include <algorithm>
#include <cmath>
#include "pid.h"

PID::PID()
{
    initialize();
}

void PID::initialize()
{
    for (int i=0;i<4;i++) {
        esc[i] = outMin;
        errorI[i] = 0;
    }
}

void PID::updateESC()
{
    int i;
    float error[4];
    error[0] = target[0] - gy87.yaw;
    if (error[0] > M_PI) {
        error[0] -= 2*M_PI;
    }
    if (error[0] < -M_PI) {
        error[0] += 2*M_PI;
    }
    error[1] = target[1] - gy87.pitch;
    error[2] = target[2] - gy87.roll;
    error[3] = target[3] - filter.altitude;

    for (i=0;i<4;i++) {
        errorI[i] += error[i] * dt;
    }

    float errorD[4];
    errorD[0] = -gy87.gz;
    errorD[1] = -gy87.gy;
    errorD[2] = gy87.gx;
    errorD[3] = filter.velocity;
    
    int output[4];

    output[0] = yawPID[0]*error[0] + yawPID[1]*errorI[0] - yawPID[2]*errorD[0];
    output[1] = pitchPID[0]*error[1] + pitchPID[1]*errorI[1] - pitchPID[2]*errorD[1];
    output[2] = rollPID[0]*error[2] + rollPID[1]*errorI[2] - rollPID[2]*errorD[2];
    output[3] = heightPID[0]*error[3] + heightPID[1]*errorI[3] - heightPID[2]*errorD[3];

    esc[0] = 1200+output[3] - output[0] + output[1] + output[2];
    esc[1] = 1200+output[3] + output[0] + output[1] - output[2];
    esc[2] = 1200+output[3] - output[0] - output[1] - output[2];
    esc[3] = 1200+output[3] + output[0] - output[1] + output[2];

    // Limit to range
    int min,max;
    int diff;
    max = *std::max_element(esc, esc+4);
    min = *std::min_element(esc, esc+4);
    if (max > outMax) {
        diff = outMax - max;
        for (i=0;i<4;i++) {
            esc[i] += diff;
        }
        for (i=0;i<4;i++) {
            if (esc[i] < outMin) {
                esc[i] = outMin;
            }
        }
        return;
    }
    if (min < outMin) {
        diff = outMin - min;
        for (i=0;i<4;i++) {
            esc[i] += diff;
        }
        for (i=0;i<4;i++) {
            if (esc[i] > outMax) {
                esc[i] = outMax;
            }
        }
        return;
    }
}

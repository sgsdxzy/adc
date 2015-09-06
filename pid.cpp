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
    errorIMax[0] = singleIMax/yawPID[1];
    errorIMin[0] = -singleIMax/yawPID[1];
    errorIMax[1] = singleIMax/pitchPID[1];
    errorIMin[1] = -singleIMax/pitchPID[1];
    errorIMax[2] = singleIMax/rollPID[1];
    errorIMin[2] = -singleIMax/rollPID[1];
    errorIMax[3] = heightIMax/heightPID[1];
    errorIMin[3] = 0;
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
        if (errorI[i] > errorIMax[i]) {
            errorI[i] = errorIMax[i];
        }
        if (errorI[i] < errorIMin[i]) {
            errorI[i] = errorIMin[i];
        }
    }

    float errorD[4];
    errorD[0] = -gy87.gz;
    errorD[1] = -gy87.gy;
    errorD[2] = gy87.gx;
    errorD[3] = filter.velocity;
    
    int output[4];
}

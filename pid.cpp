#include "pid.h"

void PID::initialize(float pidConfig[7])
{
    for (int i=0;i<3;i++) {
        pid[i] = pidConfig[i];
    }
    errorIMax = pidConfig[3];
    errorIMin = pidConfig[4];
    outMax = pidConfig[5];
    outMin = pidConfig[6];
}

float PID::update(float error, float errorD, float dt)
{
    errorI += error*dt;
    if (errorI > errorIMax) {
        errorI = errorIMax;
    }
    if (errorI < errorIMin) {
        errorI = errorIMin;
    }

    // prevError = error;

    return pid[0] * error + pid[1] * errorI + pid[2] * errorD;
}

/*
float PID::update(float error, float dt)
{
    float errorD = (error - prevError)/dt;
    return update(error, errorD, dt);
}
*/

void PIDSystem::initialize(float ratePIDSystemConfig[3][7], float attitudePIDSystemConfig[3][7], float ZPIDSystemConfig[4][7])
{
    int i;
    for (i=0;i<3;i++) {
        ratePID[i].initialize(ratePIDSystemConfig[i]);
        attitudePID[i].initialize(attitudePIDSystemConfig[i]);
    }
    for (i=0;i<2;i++) {
        VzPID[i].initialize(ZPIDSystemConfig[i]);
        altitudePID[i].initialize(ZPIDSystemConfig[i+2]);
    }
}

void PIDSystem::setRateTarget(int index, float target)
{
    // Setting target first than enabled PID status to get good result in multithreading
    rateTargets[index] = target;

    PIDTypes[index] = PID_RATE;
}

void PIDSystem::setRateTargets(float targets[3])
{
    for (int i=0;i<3;i++) {
        rateTargets[i] = targets[i];
        PIDTypes[i] = PID_RATE;
    }
}

void PIDSystem::setAttitudeTarget(int index, float target)
{
    // Setting target first than enabled PID status to get good result in multithreading
    attitudeTargets[index] = target;

    PIDTypes[index] = PID_ATTITUDE;
}

void PIDSystem::setAttitudeTargets(float targets[3])
{
    for (int i=0;i<3;i++) {
        attitudeTargets[i] = targets[i];
        PIDTypes[i] = PID_ATTITUDE;
    }
}

void PIDSystem::setVzTarget(float target, uint8_t type)
{
    // Some subtle sequence
    VzTarget = target;
    altPIDType = PID_RATE;
    altitudeType = type;
}

void PIDSystem::setAltitudeTarget(float target, uint8_t type)
{
    // Some subtle sequence
    altitudeTarget[type] = target;
    altitudeType = type;
    altPIDType = PID_ATTITUDE;
}

// Updating yprt[4], so ESC controller must read it right after the update.
void PIDSystem::update(statusContainer& status, float dt)
{
    float rates[3];
    rates[0] = -status.gyroscope[2]; // -gz
    rates[1] = -status.gyroscope[1]; // -gy
    rates[2] = status.gyroscope[0]; // +gx
    int i;
    for (i=0;i<3;i++) {
        switch(PIDTypes[i]) {
            case PID_RATE:
                yprt[i] = ratePID[i].update(rateTargets[i] - rates[i], 0, dt);
                break;
            case PID_ATTITUDE:
                yprt[i] = attitudePID[i].update(attitudeTargets[i] - status.attitude[i], rates[i], dt);
                break;
            default:
                //Wrong type, do nothing
                warn("Wrong PID type!");
                break;
        }
    }
    // first PID type then altitude type
    switch(altPIDType) {
        case PID_RATE:
            switch(altitudeType) {
                case ALTITUDE_BARO:
                    yprt[3] = VzPID[0].update(VzTarget - status.baroFilterVelocityZ, status.accAbsolute[2], dt);
                    break;
                case ALTITUDE_SONAR:
                    yprt[3] = VzPID[1].update(VzTarget - status.sonarFilterVelocityZ, status.accAbsolute[2], dt);
                    break;
                default:
                    //Wrong type, do nothing
                    warn("Wrong PID type!");
                    break;
            }
            break;
        case PID_ATTITUDE:
            switch(altitudeType) {
                case ALTITUDE_BARO:
                    yprt[3] = altitudePID[0].update(altitudeTarget[0] - status.baroFilterAltitude, status.baroFilterVelocityZ, dt);
                    break;
                case ALTITUDE_SONAR:
                    yprt[3] = altitudePID[1].update(altitudeTarget[1] - status.sonarFilterAltitude, status.sonarFilterVelocityZ, dt);
                    break;
                default:
                    //Wrong type, do nothing
                    warn("Wrong PID type!");
                    break;
            }
            break;
        default:
            //Wrong type, do nothing
            warn("Wrong PID type!");
            break;
    }
}

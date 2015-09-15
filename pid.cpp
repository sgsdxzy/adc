#include "pid.h"

float minusError(float target, float current)
{
    return target - current;
}

float circularError(float target, float current)
{                   
    float error = target - current;
    if (error > M_PI) {
        error -= 2*M_PI;
        return error;
    }
    if (error < -M_PI) {
        error += 2*M_PI;
        return error;
    }
}


void PID::initialize(float dt, float pidConfig[7])
{
    this->dt = dt;
    for (int i=0;i<3;i++) {
        pid[i] = pidConfig[i];
    }
    errorIMax = pidConfig[3];
    errorIMin = pidConfig[4];
    outMax = pidConfig[5];
    outMin = pidConfig[6];
}

float PID::update()
{
    float error = getError(target, current);
    errorI += error*dt;
    if (errorI > errorIMax) {
        errorI = errorIMax;
    }
    if (errorI < errorIMin) {
        errorI = errorIMin;
    }

    float result = pid[0] * error + pid[1] * errorI + pid[2] * errorD;
    if (next) {
        next->target = result;
        return next->update();
    }

    return result;
}

/*
float PID::update(float error, float dt)
{
    float errorD = (error - prevError)/dt;
    return update(error, errorD, dt);
}
*/

void PID::reset()
{
    errorI = 0;
    if (next) {
        next->reset();
    }
}

void PIDSystem::initialize(float dt, int yprtSanity[4], float PIDSystemConfig[5][7])
{
    int i;

    this->dt = dt;

    for (i=0;i<4;i++) {
        sanity[i] = yprtSanity[i];
    }

    for (i=0;i<3;i++) {
        attitudePID[i].initialize(dt, PIDSystemConfig[i]);
    }
    for (i=3;i<5;i++) {
        altitudePID[i].initialize(dt, PIDSystemConfig[i]);
    }

    // Yaw requires circularError
    attitudePID[0].getError = circularError;
}

void PIDSystem::setAttitudeTarget(int index, float target)
{
    // Setting target first than enabled PID status to get good result in multithreading
    attitudePID[index].target = target;

    enabled[index] = &attitudePID[index];
}

void PIDSystem::setAttitudeTargets(float targets[3])
{
    for (int i=0;i<3;i++) {
        attitudePID[i].target = targets[i];
        enabled[i] = &attitudePID[i];
    }
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
                if (i==0) {
                    // Yaw round up
                    float error = attitudeTargets[0] - status.attitude[0];
                }
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

#include "altFilter.h"

void altFilter::initialize(float startAltitude, float k[3])
{
    filterAltitude = startAltitude;
    filterVelocityZ = 0;
    altErrorI = 0;
    kp1 = k[0];
    kp2 = k[1];
    ki = k[2];
}

void altFilter::updateAltFilter(float altitude, float acceleration, float dt)
{
    float altError = altitude - filterAltitude;
    altErrorI += altError * dt;

    float compAcc = acceleration + altErrorI * ki;

    float delta = (compAcc + kp1 * altError) * dt;
    filterAltitude += (filterVelocityZ*2 + delta) * dt/2 + altError * kp2 * dt;
    filterVelocityZ += delta;
}

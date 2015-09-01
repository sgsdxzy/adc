#include "altFilter.h"

void altFilter::updateAltFilter()
{
    float altError = gy87.altitude - altitude;
    altErrorI += altError*dt;

    float compAcc = gy87.azAbsolute/8192.0*g + altErrorI*ki; // m/s^2

    float delta = (compAcc + kp1*altError)*dt;
    altitude = altitude + (velocity*2 + delta)*dt/2 + altError*kp2*dt;
    velocity = velocity + delta;
}

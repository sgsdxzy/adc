#ifndef _ALT_FILTER_H_
#define _ALT_FILTER_H_

#include <atomic>
#include "sensors/sensors.h"

class altFilter
{
    public:
        altFilter() {};
        void updateAltFilter();

        float dt = 0.01;
        float altErrorI = 0; // Integrated altitude error
        std::atomic<float> altitude;
        std::atomic<float> velocity;

        float kp1 = 0.55; // PI observer velocity gain 
        float kp2 = 1.0;  // PI observer position gain
        float ki = 0.01; // PI observer integral gain (bias cancellation)

        float g = 9.80151; // Beijing
};

extern altFilter filter;

#endif // _ALT_FILTER_H_

#ifndef _ALT_FILTER_H_
#define _ALT_FILTER_H_

class altFilter
{
    public:
        void initialize(float startAltitude, float k[3]);
        void updateAltFilter(float altitude, float acceleration, float dt);

        float filterAltitude = 0;
        float filterVelocityZ = 0;
        float altErrorI = 0; // Integrated altitude error

        float kp1; // PI observer velocity gain 
        float kp2;  // PI observer position gain
        float ki; // PI observer integral gain (bias cancellation)
};

#endif // _ALT_FILTER_H_

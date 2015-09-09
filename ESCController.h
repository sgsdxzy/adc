#ifndef _ESC_CONTROLLER_H_
#define _ESC_CONTROLLER_H_ 

#include <algorithm>
#include <pigpio.h>

class ESCController
{
    public:
        void initialize(int controlled_esc[4], int frequency, int min, int max);
        void allSetTo(int width);
        void arming();
        void startMotor();
        void stopMotor();
        void YPRT(int yprt[4]); // [yaw, pitch, roll, throttle], auto limit into range

        int ESC[4]; // Controlled ESCs
        int outMin;
        int outMax;
};

#endif // _ESC_CONTROLLER_H_

#include "ESCController.h"

void ESCController::initialize(int controlled_esc[4], int ESCSanity[4], int frequency, int min, int max)
{
    for (int i=0;i<4;i++) {
        ESC[i] = controlled_esc[i];
        gpioSetPWMfrequency(ESC[i], frequency);
        gpioSetPWMrange(ESC[i], 1000000/frequency);
        sanity[i] = ESCSanity[i];
    }
    outMin = min;
    outMax = max;
}

void ESCController::allSetTo(int width)
{
    for (int i=0;i<4;i++) {
        gpioPWM(ESC[i], width);
    }
}

void ESCController::arming()
{
    if (!armed) {
        allSetTo(1000);
        gpioSleep(PI_TIME_RELATIVE, 5, 0);
        armed = true;
    }
}

void ESCController::stopMotor()
{
    if (armed) {
        allSetTo(1000);
    }
}

void ESCController::reset()
{
    for (int i=0;i<4;i++) {
        oldESC[i] = outMin;
    }
}

void ESCController::YPRT(int yprt[4])
{
    int i;
    int ESCOutput[4];
    ESCOutput[0] = outMin+yprt[3] + yprt[0] - yprt[1] + yprt[2];
    ESCOutput[1] = outMin+yprt[3] - yprt[0] + yprt[1] + yprt[2];
    ESCOutput[2] = outMin+yprt[3] + yprt[0] + yprt[1] - yprt[2];
    ESCOutput[3] = outMin+yprt[3] - yprt[0] - yprt[1] - yprt[2];

    // Limit to range
    int min,max;
    int diff;
    max = *std::max_element(ESCOutput, ESCOutput+4);
    if (max > outMax) {
        diff = outMax - max;
        for (i=0;i<4;i++) {
            ESCOutput[i] += diff;
        }
        for (i=0;i<4;i++) {
            if (ESCOutput[i] < outMin) {
                ESCOutput[i] = outMin;
            }
        }
    }
    min = *std::min_element(ESCOutput, ESCOutput+4);
    if (min < outMin) {
        diff = outMin - min;
        for (i=0;i<4;i++) {
            ESCOutput[i] += diff;
        }
        for (i=0;i<4;i++) {
            if (ESCOutput[i] > outMax) {
                ESCOutput[i] = outMax;
            }
        }
    }

    // Sanity checker
    for (i=0;i<4;i++) {
        if (ESCOutput[i] - oldESC[i] > sanity[i]) {
            ESCOutput[i] = oldESC[i] + sanity[i];
        }
        if (ESCOutput[i] - oldESC[i] < -sanity[i]) {
            ESCOutput[i] = oldESC[i] - sanity[i];
        }
        oldESC[i] = ESCOutput[i];
    }

    // Output
    for (i=0;i<4;i++) {
        gpioPWM(ESC[i], ESCOutput[i]);
    }
}    

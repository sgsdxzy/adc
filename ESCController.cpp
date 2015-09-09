#include "ESCController.h"

void ESCController::initialize(int controlled_esc[4], int frequency, int min, int max)
{
    for (int i=0;i<4;i++) {
        ESC[i] = controlled_esc[i];
        gpioSetPWMfrequency(ESC[i], frequency);
        gpioSetPWMrange(ESC[i], 1000000/frequency)
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
    allSetTo(1000);
}

void ESCController::startMotor()
{
    allSetTo(outMin);
}
    
void ESCController::stopMotor()
{
    allSetTo(1000);
}

void ESCController::YPRT(int yprt[4])
{
    int i;
    int ESCOutput[4];
    ESCOutput[0] = outMin+yprt[3] - yprt[0] - yprt[1] + yprt[2];
    ESCOutput[1] = outMin+yprt[3] + yprt[0] + yprt[1] + yprt[2];
    ESCOutput[2] = outMin+yprt[3] - yprt[0] + yprt[1] - yprt[2];
    ESCOutput[3] = outMin+yprt[3] + yprt[0] - yprt[1] - yprt[2];

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

    // Output
    for (i=0;i<4;i++) {
        gpioPWM(ESC[i], ESCOutput[i]);
    }
}    

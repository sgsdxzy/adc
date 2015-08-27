#include "sensors/GY87.h"
#include <stdio.h>
#include <pigpio.h>

GY87 gy;

using namespace std;
void alerter(int gpio, int level, uint32_t tick)
{
    if (level==1) {
        printf("Interrupt!\n");
        gy.updateMPU();
        printf("Interrupt done\n");
    }
}

void updater()
{
    printf("BMP!");
    gy.updateBMP();
    printf("BMP done!");
}

int main()
{
    float yaw, pitch, roll, heading, alt;
    I2Cdev::initialize();
    printf("here1\n");
    gpioInitialise();
    printf("here2\n");
    gy.initialize();
    printf("here3\n");
    gpioSetMode(23, PI_OUTPUT);
    gpioWrite(23, 0);
    gpioSetMode(23, PI_INPUT);

    gpioSetAlertFunc(23, alerter);
    //gpioSetTimerFunc(0, 1000, updater);

    printf("Waiting...\n");
    gy.startDMP();
    while (true) {
        printf("BMP!");
        gy.updateBMP();
        printf("BMP done!");
        gpioDelay(100000);
        yaw=gy.yaw;
        pitch=gy.pitch;
        roll=gy.roll;
        heading=gy.heading/M_PI*180;
        alt=gy.altitude;
        printf(" %.2f \t %.2f \t %.2f \t %.0f \t %.1f\r", yaw, pitch, roll, heading, alt);
        fflush(stdout);
    }
    printf("Quit\n");
    gpioTerminate();

    return 0;
}

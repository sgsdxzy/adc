#include "sensors/sensors.h"
#include <stdio.h>
#include <pigpio.h>

using namespace std;
void alerter(int gpio, int level, uint32_t tick)
{
    if (level==1) {
        //pthread_mutex_lock(&i2cmutex);
        gy.updateMPU();
        //pthread_mutex_unlock(&i2cmutex);
    }
}

void updater()
{
    //pthread_mutex_lock(&i2cmutex);
    gy.updateBMP();
    //pthread_mutex_unlock(&i2cmutex);
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
    gpioSetTimerFunc(0, 100, updater);

    printf("Waiting...\n");
    gy.startDMP();
    while (true) {
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

#include "sensors/sensors.h"
#include "altFilter.h"
#include <stdio.h>
#include <pigpio.h>

GY87 gy87;
altFilter filter;

using namespace std;
void alerter(int gpio, int level, uint32_t tick)
{
    if (level==1) {
        //pthread_mutex_lock(&i2cmutex);
        gy87.updateMPU();
        //pthread_mutex_unlock(&i2cmutex);
    }
}

void updater()
{
    //pthread_mutex_lock(&i2cmutex);
    gy87.updateBMP(101500);
    //pthread_mutex_unlock(&i2cmutex);
}

void filterUpdater()
{
    filter.updateAltFilter();
}

int main()
{
    float yaw, pitch, roll, heading, alt, falt;
    I2Cdev::initialize();
    printf("here1\n");
    gpioInitialise();
    printf("here2\n");
    gy87.initialize();
    printf("here3\n");
    gpioSetMode(4, PI_OUTPUT);
    gpioWrite(4, 0);
    gpioSetMode(4, PI_INPUT);

    int az;
    int i=0;
    int gyro;

    printf("Waiting...\n");
    gpioSetAlertFunc(4, alerter);
    gpioSetTimerFunc(0, 10, updater);
    gy87.startDMP();
    while (true) {
        gpioDelay(100000);
        yaw=gy87.yaw;
        pitch=gy87.pitch;
        roll=gy87.roll;
        heading=gy87.heading/M_PI*180;
        alt=gy87.altitude;
        az=gy87.azAbsolute;
        gyro=gy87.gz;
        i++;
        if (i==100) {
            filter.altitude=alt;
            filter.velocity=0;
            gpioSetTimerFunc(1, 10, filterUpdater);
        }
        falt=filter.altitude;
        printf(" %.2f \t %.2f \t %.2f \t %i \t %.1f \t %.1f \t %i\n", yaw, pitch, roll, gyro, alt, falt, az);
        fflush(stdout);
    }
    printf("Quit\n");
    gpioTerminate();

    return 0;
}

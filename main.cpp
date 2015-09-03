// ----------------------------------------------------------------------------
#include <pigpio.h>
#include "sensors/sensors.h"
#include "altFilter.h"
#include "pid.h"
#include "debug.h"

// GPIOs
#define ESC_OUT_0 6
#define ESC_OUT_1 13
#define ESC_OUT_2 19
#define ESC_OUT_3 26

#define CAMERA_STATION_OUT //TODO
#define PARACHUTE_OUT // No parachute yet

#define GY87_INTERRUPT_GPIO 4

// Global variables to be shared in threads
GY87 gy87;
altFilter filter;
PID pid;

// Starting conditions
float startYaw;
float startHeading;
float startAltitude;

// ----------------------------------------------------------------------------
// Callbacks
// Handling MPU6050 data ready interrupt at 100Hz
void gy87InterruptUpdater(int gpio, int level, uint32_t tick)
{
    if (level==1) {
        gy87.updateMPU();
    }
}

// BMP180 barometer altitude updater, work at 100Hz, Clock 0
void BMPUpdater()
{
    gy87.updateBMP();
}

// Altitude complementary filter updater, work at 100Hz, Clock 1
void altitudeFilterUpdater()
{
    filter.update();
}

// Update ESC and motor using values calculated by pid subsystem, work at 100Hz, Clock 2
void PIDESCUpdater()
{
    pid.updateESC();
    gpioPWM(ESC_OUT_0, esc[0]);
    gpioPWM(ESC_OUT_1, esc[1]);
    gpioPWM(ESC_OUT_2, esc[2]);
    gpioPWM(ESC_OUT_3, esc[3]);
}

// ----------------------------------------------------------------------------
// Routines
// Motor control routines
void arming()
{
    info("Arming...");
    gpioPWM(ESC_OUT_0, 1000);
    gpioPWM(ESC_OUT_1, 1000);
    gpioPWM(ESC_OUT_2, 1000);
    gpioPWM(ESC_OUT_3, 1000);
}

void startMotor()
{
    info("Starting Motors...");
    gpioPWM(ESC_OUT_0, 1200);
    gpioPWM(ESC_OUT_1, 1200);
    gpioPWM(ESC_OUT_2, 1200);
    gpioPWM(ESC_OUT_3, 1200);
}
    
void stopMotor()
{
    info("Stopping PID...");
    gpioSetTimerFunc(2, 10, NULL);

    info("Stopping Motors...");
    gpioPWM(ESC_OUT_0, 1000);
    gpioPWM(ESC_OUT_1, 1000);
    gpioPWM(ESC_OUT_2, 1000);
    gpioPWM(ESC_OUT_3, 1000);
}

// Initialize all sensors and motors
void systemInitialize()
{
    // Initialize sensors
    info("Initializing sensors...");
    I2Cdev::initialize();
    gpioInitialise();
    gy87.initialize();

    // Start gathering data
    info("Starting GY87 data gathering...");
    gpioSetAlertFunc(GY87_INTERRUPT_GPIO, gy87InterruptUpdater);
    gpioSetTimerFunc(0, 10, BMPUpdater);
    gy87.startDMP();

    // Set ESC update frequency to 400Hz
    info("Setting PWM frequencies...");
    gpioSetPWMfrequency(ESC_OUT_0, 400)
    gpioSetPWMfrequency(ESC_OUT_1, 400)
    gpioSetPWMfrequency(ESC_OUT_2, 400)
    gpioSetPWMfrequency(ESC_OUT_3, 400)

    gpioSetPWMrange(ESC_OUT_0, 2500);
    gpioSetPWMrange(ESC_OUT_1, 2500);
    gpioSetPWMrange(ESC_OUT_2, 2500);
    gpioSetPWMrange(ESC_OUT_3, 2500);

    // Centering camera station
    info("Centering camera station...");
    gpioServo(CAMERA_STATION_OUT, 1500);
}

// Initialization of altitude filter and starting conditions when GY87's data is stable
void startingConditionInitialize()
{
    info("Recording starting yaw, magnetic heading and altitude...")
    startYaw = gy87.yaw;
    startHeading = gy87.heading;
    startAltitude = gy87.altitude;

    info("Starting altitude filter...")
    filter.altitude = startAltitude;
    filter.velocity = 0;
    filter.altErrorI = 0;
    gpioSetTimerFunc(1, 10, altitudeFilterUpdater);

    // Wait for 2 seconds
    gpioSleep(PI_TIME_RELATIVE, 2, 0);
    info("Getting starting altitude from altitude filter reading...");
    startAltitude = filter.altitude;
    info("Altitude = "+to_string(startAltitude));
}

void pidInitialize()
{
    info("Initializing PID system...");
    pid.Initialize();

    //TODO PID tune

    // Recording starting attitude, in case ground is not level
    pid.target[0] = gy87.yaw;
    pid.target[1] = gy87.pitch;
    pid.target[2] = gy87.roll;
    pid.target[3] = filter.altitude;

    gpioSetTimerFunc(2, 10, PIDESCUpdater);
}

void cleanup()
{
    info("Cleaning up...");
    gy87.reset();
    gpioTerminate();
}

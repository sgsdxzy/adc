// ----------------------------------------------------------------------------
#include <pigpio.h>
#include "status.h"
#include "config.h"
#include "altFilter.h"
#include "pid.h"
#include "debug.h"

// GPIOs
#define CAMERA_STATION_OUT 5
#define PARACHUTE_OUT // No parachute yet

#define GY87_INTERRUPT_GPIO 4

// Global variables to be shared in threads
statusContainer status;
configuration config;
ESCController escControl;

GY87 gy87;
altFilter filter;
PID pid;

// ----------------------------------------------------------------------------
// Callbacks
// Handling MPU6050 data ready interrupt at 100Hz
void gy87InterruptUpdater(int gpio, int level, uint32_t tick)
{
    if (level==1) {
        gy87.updateMPU();

        //Atomic value assign
        for (int i=0;i<3;i++) {
            status.attitude[i] = gy87.ypr[i]; // rad
        }
        status.heading = gy87.mh; // rad
        status.gyroscope[0]   = (float)gy87.gyro.x/32768*1000*M_PI/180; // rad/s
        status.gyroscope[1]   = (float)gy87.gyro.y/32768*1000*M_PI/180;
        status.gyroscope[2]   = (float)gy87.gyro.z/32768*1000*M_PI/180;
        status.accRelative[0] = (float)gy87.aaReal.x/8192*config.g; // m/s^2
        status.accRelative[1] = (float)gy87.aaReal.y/8192*config.g;
        status.accRelative[2] = (float)gy87.aaReal.z/8192*config.g;
        status.accAbsolute[0] = (float)gy87.aaWorld.x/8192*config.g;
        status.accAbsolute[1] = (float)gy87.aaWorld.y/8192*config.g;
        status.accAbsolute[2] = (float)gy87.aaWorld.z/8192*config.g;
    }
}

// BMP180 barometer altitude updater, work at 100Hz, Clock 0
void BMPUpdater()
{
    gy87.updateBMP(config.seaLevelPressure);

    status.temperature = gy87.temperature; // C
    status.pressure = gy87.pressure; // Pa
    status.baroAltitude = gy87.baroAltitude; // m
}

// Altitude complementary filter updater, work at 100Hz, Clock 1
void altitudeFilterUpdater()
{
    filter.updateAltFilter(status.baroAltitude, status.accAbsolute[2], config.dt);

    status.filterAltitude = filter.filterAltitude;
    status.filterVelocityZ = filter.filterVelocityZ;
}

// Update ESC and motor using values calculated by pid subsystem, work at 100Hz, Clock 2
void PIDESCUpdater()
{
    pid.updateESC();
    gpioPWM(ESC_OUT_0, pid.esc[0]);
    gpioPWM(ESC_OUT_1, pid.esc[1]);
    gpioPWM(ESC_OUT_2, pid.esc[2]);
    gpioPWM(ESC_OUT_3, pid.esc[3]);
}

// ----------------------------------------------------------------------------
// Routines
// Initialize all sensors and motors
void systemInitialize()
{
    // Initialize sensors
    info("Initializing system...");
    I2Cdev::initialize();
    gpioInitialise();

    // Initialize ESC controller
    info("Initializing ESC controller...");
    escControl.initialize(config.controlled_esc, config.ESCFrequency, config.outMin, config.outMax);

    // Centering camera station
    info("Centering camera station...");
    gpioServo(CAMERA_STATION_OUT, 1500);

    // Start GY87
    info("Initializing GY87 10-DOF IMU...")
    gy87.initialize();
    gy87.setOffset(config.gy87Offset);
    info("Starting GY87 data gathering...");
    gpioSetAlertFunc(GY87_INTERRUPT_GPIO, gy87InterruptUpdater);
    gy87.startDMP();
    gpioSetTimerFunc(0, 10, BMPUpdater);

    // Wait 10s
    info("Waiting for DMP data to stablize...");
    gpioSleep(PI_TIME_RELATIVE, 10, 0);

    // Initialization of altitude filter and starting conditions when GY87's data is stable
    info("Recording starting yaw and magnetic heading...");
    for (int i=0;i<3;i++) {
        status.startAttitude[i] = status.attitude[i];
    }
    status.startHeading = status.heading;
    status.startAltitude = status.baroAltitude;

    info("Starting altitude filter...");
    filter.initialize(status.startAltitude, config.k);
    gpioSetTimerFunc(1, 10, altitudeFilterUpdater);

    // Wait for 1 seconds
    gpioSleep(PI_TIME_RELATIVE, 1, 0);
    info("Getting starting altitude from altitude filter reading...");
    status.startAltitude = status.filterAltitude;
    info("Altitude = "+to_string(startAltitude));
}

void pidInitialize()
{
    info("Initializing PID system...");

    //PID tune
    pid.yawPID[0] = 100;
    pid.yawPID[1] = 1;
    pid.yawPID[2] = 0.03;

    pid.pitchPID[0] = 100;
    pid.pitchPID[1] = 1;
    pid.pitchPID[2] = 0.02;

    pid.rollPID[0] = 100;
    pid.rollPID[1] = 1;
    pid.rollPID[2] = 0.02;

    pid.heightPID[0] = 20;
    pid.heightPID[1] = 20;
    pid.heightPID[2] = 20;

    pid.initialize();
    // Recording starting attitude, in case ground is not level
    for (int i=0;i<3;i++) {
        pid.target[i] = status.startAttitude[i];
    }
    pid.target[3] = status.startAltitude;

    gpioSetTimerFunc(2, 10, PIDESCUpdater);
}

void cleanup()
{
    info("Cleaning up...");
    gy87.reset();
    gpioTerminate();
}

// ----------------------------------------------------------------------------
// Temporary
using namespace std;
void statusDisplayer()
{
    cout << "Yaw: " << status.attitude[0] << " ";
    cout << "Pitch: " << status.attitude[1] << " ";
    cout << "Roll: " << status.attitude[2] << " ";
    cout << "Baro: " << status.baroAltitude << " ";
    cout << "Altitdue: " << status.filterAltitude << " ";
    cout << "ESC 0: " << gpioGetPWMdutycycle(ESC_OUT_0) << " ";
    cout << "ESC 1: " << gpioGetPWMdutycycle(ESC_OUT_1) << " ";
    cout << "ESC 2: " << gpioGetPWMdutycycle(ESC_OUT_2) << " ";
    cout << "ESC 3: " << gpioGetPWMdutycycle(ESC_OUT_3) << " ";
    cout << endl;
}

// Main program
int main(int argc, char** argv)
{
    systemInitialize();
    gpioSetTimerFunc(3, 200, statusDisplayer);

    string command;
    while (true) {
        cin >> command;
        if (command == "con") { 
            startingConditionInitialize();
            continue;
        }
        if (command == "arm") {
            arming();
            continue;
        }
        if (command == "start") {
            startMotor();
            continue;
        }
        if (command == "pid") {
            pidInitialize();
            continue;
        }
        if (command == "stop") {
            stopMotor();
            continue;
        }
        if (command == "quit") {
            break;
        }

        if (command[0] == 'c') {
            gpioServo(CAMERA_STATION_OUT, std::stoi(command.substr(1)));
            continue;
        }
        if (command[0] == 'y') {
            pid.target[0] = std::stof(command.substr(1));
            continue;
        }
        if (command[0] == 'p') {
            pid.target[1] = std::stof(command.substr(1));
            continue;
        }
        if (command[0] == 'r') {
            pid.target[2] = std::stof(command.substr(1));
            continue;
        }
        if (command[0] == 'a') {
            pid.target[3] = std::stof(command.substr(1));
            continue;
        }
        err("Unknow command: "+command);
    }

    cleanup();
    return 0;
}

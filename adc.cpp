// ----------------------------------------------------------------------------
#include <fstream>
#include <pigpio.h>
#include "status.h"
#include "config.h"
#include "GY87.h"
#include "altFilter.h"
#include "ESCController.h"
#include "pid.h"
#include "debug.h"

// GPIOs
#define CAMERA_STATION_OUT 18
// #define PARACHUTE_OUT // No parachute yet

#define GY87_INTERRUPT_GPIO 4
#define HCSR04_TRIG 20
#define HCSR04_ECHO 16
#define HCSR04_VCC 21

// Global variables to be shared in threads
statusContainer status;
configuration config;
ESCController escControl;

GY87 gy87;
altFilter baroFilter;
altFilter sonarFilter;
PIDSystem pids;

// For HC-SR04+
uint32_t start = 0;
bool gotEcho = false;
int hcsr04pDelay = 0; // Due to limitations, update HC-SR04+ only at 10Hz

bool PIDEnabled = false;


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
        status.gravity[0]     = gy87.gravity.x;
        status.gravity[1]     = gy87.gravity.y;
        status.gravity[2]     = gy87.gravity.z;
        status.accRelative[0] = (float)gy87.aaReal.x/8192*config.g; // m/s^2
        status.accRelative[1] = (float)gy87.aaReal.y/8192*config.g;
        status.accRelative[2] = (float)gy87.aaReal.z/8192*config.g;
        status.accAbsolute[0] = (float)gy87.aaWorld.x/8192*config.g;
        status.accAbsolute[1] = (float)gy87.aaWorld.y/8192*config.g;
        status.accAbsolute[2] = (float)gy87.aaWorld.z/8192*config.g;
    }
}

// HC-SR04+ 
// Handle echo
void getEcho (int gpio, int level, uint32_t tick)
{
    if (level == 1) {
        start = tick;
    } else {
        status.sonarAltitude = ((tick-start) * 1e-6 * status.sonicVelocity/2) * status.gravity[2];
        gotEcho = true;
    }
}

// Updater thread, works at 100Hz, Clock 0
void updater()
{
    // -------------------------------------------------------------------------
    // Sensors
    // BMP180 barometer 
    gy87.updateBMP(config.seaLevelPressure);

    status.temperature = gy87.temperature; // C
    status.pressure = gy87.pressure; // Pa
    status.baroAltitude = gy87.baroAltitude; // m
    status.sonicVelocity = gy87.sonicVelocity; // m/s

    // baroFilter
    baroFilter.updateAltFilter(gy87.baroAltitude, status.accAbsolute[2], config.dt);
    status.baroFilterAltitude = baroFilter.filterAltitude;
    status.baroFilterVelocityZ = baroFilter.filterVelocityZ;


    // HC-SR04+ trigger at 10Hz
    if (hcsr04pDelay == 0) {
        status.sonar = gotEcho;
        gotEcho = false;
        gpioTrigger(HCSR04_TRIG, 40, 1);
    }
    hcsr04pDelay += 1;
    hcsr04pDelay = hcsr04pDelay % 10;

    // sonarFilter, only when sonar is avaliable.
    if (status.sonar) {
        sonarFilter.updateAltFilter(status.sonarAltitude, status.accAbsolute[2], config.dt);
        if (sonarFilter.filterAltitude < 0) {
            // Not possible to < 0
            status.sonarFilterAltitude = 0;
        } else {
            status.sonarFilterAltitude = sonarFilter.filterAltitude;
        }
        status.sonarFilterVelocityZ = sonarFilter.filterVelocityZ;
    }


    // -------------------------------------------------------------------------
    // PID system
    if (PIDEnabled) {
        pids.update(status, config.dt);
        escControl.YPRT(pids.yprt);
    }
}



// ----------------------------------------------------------------------------
// Routines
// Initialize all sensors and motors
void systemInitialize()
{
    info("Loading configuration file...");
    std::ifstream configFile("config.txt");
    configFile >> config;
    configFile.close();

    info("Initializing system...");
    I2Cdev::initialize();
    gpioInitialise();

    // Initialize ESC controller
    info("Initializing ESC controller...");
    escControl.initialize(config.controlled_esc, config.ESCFrequency, config.ESCOutMin, config.ESCOutMax);

    // Centering camera station
    info("Centering camera station...");
    gpioServo(CAMERA_STATION_OUT, 1500);
}

void initialize()
{
    // Power sonar up
    gpioWrite(HCSR04_VCC, 1);

    // Start GY87
    info("Initializing GY87 10-DOF IMU...");
    gy87.initialize();
    gy87.setOffset(config.gy87Offset);
    info("Starting GY87 data gathering...");
    gpioSetAlertFunc(GY87_INTERRUPT_GPIO, gy87InterruptUpdater);
    gy87.startDMP();

    // Wait 10s
    info("Waiting for DMP data to stablize...");
    gpioSleep(PI_TIME_RELATIVE, 10, 0);

    // Initialization of altitude filter and starting conditions when GY87's data is stable
    info("Recording starting yaw and magnetic heading...");
    for (int i=0;i<3;i++) {
        status.startAttitude[i] = status.attitude[i];
    }
    status.startHeading = status.heading;

    info("Initializing altitude filters...");
    // Run twice to get temperature and altitude
    gy87.updateBMP(config.seaLevelPressure);
    gy87.updateBMP(config.seaLevelPressure);
    baroFilter.initialize(gy87.baroAltitude, config.baroFilterConfig);
    sonarFilter.initialize(0, config.sonarFilterConfig);

    info("Initializing PID system...");
    pids.initialize(config.ratePIDSystemConfig, config.attitudePIDSystemConfig, config.ZPIDSystemConfig);
    pids.setAttitudeTargets(status.startAttitude);
    //pids.setVzTarget(0.028, ALTITUDE_BARO);
    pids.setAltitudeTarget(0, ALTITUDE_SONAR);

    info("Starting updater thread...");
    gpioSetMode(HCSR04_ECHO, 0);
    gpioSetMode(HCSR04_TRIG, 1);
    gpioSetAlertFunc(HCSR04_ECHO, getEcho);
    gpioSetTimerFunc(0, 10, updater);

    // Wait for 2 seconds
    gpioSleep(PI_TIME_RELATIVE, 2, 0);
    info("Getting starting altitude from altitude filter reading...");
    status.startAltitude = status.baroFilterAltitude;
    info("Altitude = "+to_string(status.startAltitude));
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
    //cout << "Y: " << status.attitude[0] << "\t";
    //cout << "P: " << status.attitude[1] << "\t";
    //cout << "R: " << status.attitude[2] << "\t";
    cout << "ZA: " << status.accAbsolute[2] << "\t";
    //cout << "GZ: " << status.gravity[2] << "\t";
    //cout << "Baro: " << status.baroAltitude << "\t";
    //cout << "Sonar: " << status.sonarAltitude << "\t";
    cout << "BA: " << status.baroAltitude << "\t";
    cout << "BF: " << status.baroFilterAltitude << "\t";
    cout << "BV: " << status.baroFilterVelocityZ << "\t";
    cout << "SA: " << status.sonarAltitude << "\t";
    cout << "SF: " << status.sonarFilterAltitude << "\t";
    cout << "SV: " << status.sonarFilterVelocityZ << "\t";
    //cout << "E0: " << gpioGetPWMdutycycle(config.controlled_esc[0]) << "\t";
    //cout << "E1: " << gpioGetPWMdutycycle(config.controlled_esc[1]) << "\t";
    //cout << "E2: " << gpioGetPWMdutycycle(config.controlled_esc[2]) << "\t";
    //cout << "E3: " << gpioGetPWMdutycycle(config.controlled_esc[3]) << "\t";
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
        if (command == "arm") {
            escControl.arming();
            continue;
        }
        if (command == "start") {
            escControl.startMotor();
            PIDEnabled = true;
            continue;
        }
        if (command == "init") {
            initialize();
            continue;
        }
        if (command == "stop") {
            PIDEnabled = false;
            gpioDelay(100000);
            escControl.stopMotor();
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
            pids.setAttitudeTarget(0, std::stof(command.substr(1)));
            continue;
        }
        if (command[0] == 'p') {
            pids.setAttitudeTarget(1, std::stof(command.substr(1)));
            continue;
        }
        if (command[0] == 'r') {
            pids.setAttitudeTarget(2, std::stof(command.substr(1)));
            continue;
        }
        if (command[0] == 'a') {
            pids.setAltitudeTarget(std::stof(command.substr(1)), ALTITUDE_BARO);
            continue;
        }
        if (command[0] == 'n') {
            pids.setAltitudeTarget(std::stof(command.substr(1)), ALTITUDE_SONAR);
            continue;
        }
        err("Unknow command: "+command);
    }

    cleanup();
    return 0;
}

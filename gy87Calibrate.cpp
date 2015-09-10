#include <iostream>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <bcm2835.h>
#include "debug.h"
#include "config.h"
#include "MPU6050.h"
#include "I2Cdev.h"

using namespace std;
int main(int argc, char *argv[])
{
    I2Cdev::initialize();
    MPU6050 mpu ;
    int16_t motions[6];
    int average[6];
    int targets[6] = {0, 0, 16384, 0, 0, 0};
    int errors[6];
    int maxError;
    int i,j;
    int times = 1000;
    int tolerence = 10;
    if (argc == 3) {
        times = atoi(argv[1]);
        tolerence = atoi(argv[2]);
    }

    info("GY87 calibration program");

    info("Loading configuration file...");
    ifstream iconfigFile("config.txt");
    configuration config;
    iconfigFile >> config;
    iconfigFile.close();

    info("Initializing MPU6050...");
    mpu.reset();
    bcm2835_delay(200);
    mpu.initialize();
    mpu.setFullScaleAccelRange(0);
    mpu.setFullScaleGyroRange(0);
    mpu.setRate(0);
    mpu.setDLPFMode(MPU6050_DLPF_BW_42);


    while (true) {
        // Loop until meet tolerence
        info("Offsets:");
        for (i=0;i<6;i++) {
            cout << config.gy87Offset[i] << " ";
        }
        cout << endl;

        info("Setting offsets...");
        mpu.setXAccelOffset(config.gy87Offset[0]);
        mpu.setYAccelOffset(config.gy87Offset[1]);
        mpu.setZAccelOffset(config.gy87Offset[2]);
        mpu.setXGyroOffset(config.gy87Offset[3]);
        mpu.setYGyroOffset(config.gy87Offset[4]);
        mpu.setZGyroOffset(config.gy87Offset[5]);

        // Getting average
        info("Getting average...");
        for (i=0;i<6;i++) {
            average[i] = 0;
        }
        for (i=0;i<times;i++) {
            mpu.getMotion6(&(motions[0]), &(motions[1]), &(motions[2]), &(motions[3]), &(motions[4]), &(motions[5]));
            for (j=0;j<6;j++) {
                average[j] += motions[j];
            }
            bcm2835_delay(10);
        }

        // Getting errors
        cout << "Errors:" << endl;
        for (i=0;i<6;i++) {
            average[i] /= times;
            cout << average[i] - targets[i] << " ";
            errors[i] = abs(targets[i] - average[i]);
        }
        cout << endl;
        maxError = *max_element(errors, errors+6);
        if (maxError <= tolerence) {
            break;
        }

        // New offsets
        for (i=0;i<6;i++) {
            config.gy87Offset[i] += (targets[i]-average[i])/8;
        }
    }

    info("MPU6050 Calibration finished, writing to configuration file...");
    ofstream oconfigFile("config.txt");
    oconfigFile << config;
    oconfigFile.close();

    return 0; 
}

#include "Calibrator.h"

void Calibrator::initialize(Configuration* config)
{
    // Copy configs
    g = config->g;

    axisRotationMatrix = config->axisRotationMatrix;

    calibrateCompass = config->calibrateCompass;
    calibrateCompassEllipsoid = config->calibrateCompassEllipsoid;
    compassCalEllipsoidOffset = config->calibrateCompassEllipsoidOffset;
    compassCalEllipsoidMarix = config->calibrateCompassEllipsoidMatrix;

    calibrateAccel = config->calibrateAccel;

    if (calibrateAccel) {
        for (int i=0;i<3;i++) {
            accCalMin[i] = config->calibrateAccelMin[i];
            accCalMax[i] = config->calibrateAccelMax[i];
        }
    }

    sampleRateCount = config->MPU6050Rate * 5;

    if (calibrateCompass) {
        Vector3f delta;
        float maxDelta;
        //  find biggest range
        delta = config->calibrateCompassMax - config->calibrateCompassMin;
        maxDelta = delta.maxCoeff();

        compassCalOffset = (config->calibrateCompassMax + config->calibrateCompassMin) / 2.0;

        for (int i = 0; i < 3; i++) {
            compassCalScale[i] = maxDelta / delta(i);            // makes everything the same range
        }
    }

    gyroLearningAlpha = 2.0f / config->MPU6050Rate;
    gyroContinuousAlpha = 0.001f / config->MPU6050Rate;
}

void Calibrator::calibrate(Status* status)
{
    int i;

    // Rotate axis
    status->accCal = axisRotationMatrix * status->accRaw;
    status->gyroCal = axisRotationMatrix * status->gyroRaw;
    status->compassCal = axisRotationMatrix * status->compassRaw;
 
    // Calibrate acceleration
    if (calibrateAccel) {
        for (i=0;i<3;i++) {
            if (status->accCal(i) >= 0) {
                status->accCal(i) /= accCalMax[i];
            } else {
                status->accCal(i) /= -accCalMin[i];
            }
        }
    }
    // Get results in m/s^2
    status->accCal *= g;
   
    // Handle gyro bias
    Vector3f deltaAccel = status->accCal - previousAccel;
    previousAccel = status->accCal;

    if ((deltaAccel.norm() < 0.05) && (status->gyroCal.norm() < 0.20)) { // Numbers recommanded by RTIMULib
        // what we are seeing on the gyros should be bias only so learn from this

        if (gyroSampleCount < sampleRateCount) {
            gyroBias = (1.0 - gyroLearningAlpha) * gyroBias + gyroLearningAlpha * status->gyroCal;

            gyroSampleCount++;
        } else {
            gyroBias = (1.0 - gyroContinuousAlpha) * gyroBias + gyroContinuousAlpha * status->gyroCal;
        }
    }

    status->gyroCal -= gyroBias;

    // Calibrate compass
    if (calibrateCompass) {
        status->compassCal -= compassCalOffset;
        for (i=0;i<3;i++) {
            status->compassCal(i) *= compassCalScale[i];
        }

        if (calibrateCompassEllipsoid) {
            status->compassCal -= compassCalEllipsoidOffset;
            status->compassCal = compassCalEllipsoidMarix * status->compassCal;
        }
    }
    // update running average
    // compassAverage = status->compassCal * 0.2f + compassAverage * (1.0 - 0.2f); // 0.2 recommanded by RTIMULib
    // status->compassCal = compassAverage;
}

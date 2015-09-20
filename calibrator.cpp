#include "calibrator.h"
#include "helper_3dmath.h"

void Calibrator::initialize(configuration* config)
{
    axisRotationMatrix = (float[3][3])axisRotation[config->axisRotation];
    sampleRate = config->MPU6050Rate;

    float maxDelta = -1;
    float delta[3];

    if (config->calibrateCompass) {
        //  find biggest range

        for (int i = 0; i < 3; i++) {
            delta[i] = config->calibrateCompassMax[i] - config->calibrateCompassMin[i];
            if (delta[i] > maxDelta) {
                maxDelta = delta[i];
            }
        }
        if (maxDelta < 0) {
            err("Error in compass calibration data!");
            return;
        }

        for (int i = 0; i < 3; i++) {
            compassCalScale[i] = maxDelta / delta[i];            // makes everything the same range
            compassCalOffset[i] = (config->calibrateCompassMax[i] + config->calibrateCompassMin[i]) / 2.0f;
        }
    }

    gyroLearningAlpha = 2.0f / config->MPU6050Rate;
    gyroContinuousAlpha = 0.01f / config->MPU6050Rate;
    gyroSampleCount = 0;
}

void Calibrator::calibrate(statusContainer* status)
{
    int i,j;
    // Rotate axis
    status->accCal.zero();
    status->gyroCal.zero();
    status->compassCal.zero();
    for (i=0;i<3;i++) {
        for (j=0;j<3;j++) {
            status->accCal.data[i] += axisRotationMatrix[i][j] * status->accRaw[j];
            status->gyroCal.data[i] += axisRotationMatrix[i][j] * status->gyroRaw[j];
            status->compassCal.data[i] += axisRotationMatrix[i][j] * status->compassRaw[j];
        }
    }
    
    // Handle gyro bias
    RTVector3 deltaAccel = previousAccel;
    deltaAccel -= status->accCal;   // compute difference
    previousAccel = status->accel;

    if ((deltaAccel.length() < 0.05) && (status->gyroCal.length() < 0.20)) { // Numbers recommanded by RTIMULib
        // what we are seeing on the gyros should be bias only so learn from this

        if (gyroSampleCount < (5 * sampleRate)) {
            gyroBias.data[0] = (1.0 - gyroLearningAlpha) * gyroBias.data[0] + gyroLearningAlpha * status->gyroCal.data[0];
            gyroBias.data[1] = (1.0 - gyroLearningAlpha) * gyroBias.data[1] + gyroLearningAlpha * status->gyroCal.data[1];
            gyroBias.data[2] = (1.0 - gyroLearningAlpha) * gyroBias.data[2] + gyroLearningAlpha * status->gyroCal.data[2];

            gyroSampleCount++;
        } else {
            gyroBias.data[0] = (1.0 - gyroContinuousAlpha) * gyroBias.data[0] + gyroContinuousAlpha * status->gyroCal.data[0];
            gyroBias.data[1] = (1.0 - gyroContinuousAlpha) * gyroBias.data[1] + gyroContinuousAlpha * status->gyroCal.data[1];
            gyroBias.data[2] = (1.0 - gyroContinuousAlpha) * gyroBias.data[2] + gyroContinuousAlpha * status->gyroCal.data[2];
        }
    }

    status->gyroCal -= gyroBias;

    // Calibrate compass
    // Calibrate acceleration
}

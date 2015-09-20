#ifndef _CALIBRATOR_
#define _CALIBRATOR_

#include "config.h"
#include "status.h"
#include "debug.h"
#include "RTMath.h"


class Calibrator
{
    public:
        static const float axisRotation[24][9] = {
            {1, 0, 0, 0, 1, 0, 0, 0, 1},                    // RTIMU_XNORTH_YEAST
            {0, -1, 0, 1, 0, 0, 0, 0, 1},                   // RTIMU_XEAST_YSOUTH
            {-1, 0, 0, 0, -1, 0, 0, 0, 1},                  // RTIMU_XSOUTH_YWEST
            {0, 1, 0, -1, 0, 0, 0, 0, 1},                   // RTIMU_XWEST_YNORTH

            {1, 0, 0, 0, -1, 0, 0, 0, -1},                  // RTIMU_XNORTH_YWEST
            {0, 1, 0, 1, 0, 0, 0, 0, -1},                   // RTIMU_XEAST_YNORTH
            {-1, 0, 0, 0, 1, 0, 0, 0, -1},                  // RTIMU_XSOUTH_YEAST
            {0, -1, 0, -1, 0, 0, 0, 0, -1},                 // RTIMU_XWEST_YSOUTH

            {0, 1, 0, 0, 0, -1, -1, 0, 0},                  // RTIMU_XUP_YNORTH
            {0, 0, 1, 0, 1, 0, -1, 0, 0},                   // RTIMU_XUP_YEAST
            {0, -1, 0, 0, 0, 1, -1, 0, 0},                  // RTIMU_XUP_YSOUTH
            {0, 0, -1, 0, -1, 0, -1, 0, 0},                 // RTIMU_XUP_YWEST

            {0, 1, 0, 0, 0, 1, 1, 0, 0},                    // RTIMU_XDOWN_YNORTH
            {0, 0, -1, 0, 1, 0, 1, 0, 0},                   // RTIMU_XDOWN_YEAST
            {0, -1, 0, 0, 0, -1, 1, 0, 0},                  // RTIMU_XDOWN_YSOUTH
            {0, 0, 1, 0, -1, 0, 1, 0, 0},                   // RTIMU_XDOWN_YWEST

            {1, 0, 0, 0, 0, 1, 0, -1, 0},                   // RTIMU_XNORTH_YUP
            {0, 0, -1, 1, 0, 0, 0, -1, 0},                  // RTIMU_XEAST_YUP
            {-1, 0, 0, 0, 0, -1, 0, -1, 0},                 // RTIMU_XSOUTH_YUP
            {0, 0, 1, -1, 0, 0, 0, -1, 0},                  // RTIMU_XWEST_YUP

            {1, 0, 0, 0, 0, -1, 0, 1, 0},                   // RTIMU_XNORTH_YDOWN
            {0, 0, 1, 1, 0, 0, 0, 1, 0},                    // RTIMU_XEAST_YDOWN
            {-1, 0, 0, 0, 0, 1, 0, 1, 0},                   // RTIMU_XSOUTH_YDOWN
            {0, 0, -1, -1, 0, 0, 0, 1, 0}                   // RTIMU_XWEST_YDOWN
        };

        void initialize(configuration* config);

        /* All-in-one calibrator, doing the following four:
         * First thing to call after getting data from IMU
        void axisRotate(statusContainer* status);
        void calibrateAverageCompass(statusContainer* status);
        void calibrateGyro(statusContainer* status)
        void calibrateAcceleration(statusContainer* status);
        */
        void calibrate(statusContainer* status);


    private:
        float axisRotationMatrix[3][3];
        int16_t sampleRate;

        float compassCalScale[3];
        float compassCalOffset[3];

        RTVector3 previousAccel;
        RTVector3 gyroBias;
        float gyroSampleCount = 0;
        float gyroLearningAlpha;
        float gyroContinuousAlpha;
};

#endif // _CALIBRATOR_

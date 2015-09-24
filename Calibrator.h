#ifndef _CALIBRATOR_H_
#define _CALIBRATOR_H_

#include <Eigen/Dense>
#include "Configuration.h"
#include "Status.h"

using namespace Eigen;

class Calibrator
{
    public:
         void initialize(Configuration* config);

        /* All-in-one calibrator, doing the following four:
         * First thing to call after getting data from IMU
        void axisRotate(statusContainer* status);
        void calibrateAverageCompass(statusContainer* status);
        void calibrateGyro(statusContainer* status)
        void calibrateAcceleration(statusContainer* status);
        */
        void calibrate(Status* status);


    private:
        float g;
        Matrix3f axisRotationMatrix;
        int16_t sampleRateCount;

        // Vector3f compassAverage = Vector3f::Zero();
        bool calibrateCompass = false;
        float compassCalScale[3];
        Vector3f compassCalOffset = Vector3f::Zero();
        bool calibrateCompassEllipsoid = false;
        Vector3f compassCalEllipsoidOffset = Vector3f::Zero();
        Matrix3f compassCalEllipsoidMarix = Matrix3f::Identity();

        bool calibrateAccel = false;
        float accCalMin[3];
        float accCalMax[3];

        Vector3f previousAccel = Vector3f::Zero();
        Vector3f gyroBias = Vector3f::Zero();
        float gyroSampleCount = 0;
        float gyroLearningAlpha;
        float gyroContinuousAlpha;
};

#endif // _CALIBRATOR_H_

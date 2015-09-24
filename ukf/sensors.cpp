/*
Copyright (C) 2013 Daniel Dyer

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <cmath>

#include "types.h"
#include "sensors.h"
#include "state.h"
#include "debug.h"

SensorModel::~SensorModel() {}


/*
Takes the mean of a sigma point distribution given a column vector of weights.
*/
MeasurementVector SensorModel::calculate_mean(
const MatrixXr &in, const VectorXr &weights) {
    return in * weights;
}


/*
Calculates the difference between a sigma point distribution and the supplied
mean vector.
*/
MatrixXr SensorModel::calculate_deltas(
const MatrixXr &in,
const MeasurementVector &mean) {
    return in.colwise() - mean;
}


/* Calculates the current size of the measurement vector. */
size_t AriaModel::size() const {
    size_t i = 0;

    i += status->acc ? 6 : 0;
    i += status->gyro ? 3 : 0;
    i += status->compass ? 3 : 0;
    i += status->GPSP ? 3 : 0;
    i += status->gpsv ? 3 : 0;
    i += status->baro ? 1 : 0;
    i += status->sonar ? 1 : 0;

    return i;
}


/*
Takes all populated sensor data and puts it into a vector for use by the UKF.
*/
MeasurementVector AriaModel::collate() const {
    MeasurementVector::Index max_size = (MeasurementVector::Index)size();
    MeasurementVector::Index i = 0;
    if(status->acc) {
        max_size -= 3;
    }
    MeasurementVector measurement(max_size);

    if(status->acc) {
        measurement.segment<3>(i) << status->accCal;
        i += 3;
    }

    if(status->gyro) {
        measurement.segment<3>(i) << status->gyroCal;
        i += 3;
    }

    if(status->compass) {
        measurement.segment<3>(i) << status->compassCal;
        i += 3;
    }

    if(status->GPSP) {
        measurement.segment<3>(i) << status->GPSPposition;
        i += 3;
    }

    if(status->GPSV) {
        measurement.segment<3>(i) << status->GPSVelocity;
        i += 3;
    }

    if(status->baro) {
        measurement[i] = status->baroAltitude;
        i++;
    }

    if(status->sonar) {
        measurement[i] = status->sonarDistance;
        i++;
    }


    return measurement;
}


/*
Takes a state vector and creates a predicted measurement vector containing
only the sensor values which have been supplied.
*/
MeasurementVector AriaModel::predict(const State &in) const {
    MeasurementVector::Index max_size = (MeasurementVector::Index)size();
    MeasurementVector::Index i = 0;
    MeasurementVector predicted(max_size);
    Quaternionr attitude = Quaternionr(in.attitude());

    AssertNormalized(attitude);

    /*
    The accelerometer reading is predicted by adding the expected gravity
    vector (transformed by orientation) to the state vector acceleration.
    Also, if the offset vector is non-zero, need to include the angular
    acceleration term and the centripetal acceleration term.
    */
    if(status->acc) {
        predicted.segment<3>(i) << in.acceleration() +
            in.angular_acceleration().cross(accelerometer_offset) +
            in.angular_velocity().cross(in.angular_velocity().cross(
                        accelerometer.offset));
        predicted.segment<3>(i+3) << attitude * Vector3r(0, 0, -g);
        i += 6;
    }

    /*
    The gyroscope reading is simply the angular velocity transformed by the
    sensor orientation.
    */
    if(status->gyro) {
        predicted.segment<3>(i) << in.angular_velocity() + in.gyro_bias();
        i += 3;
    }

    /*
    Predicted magnetometer measurement is easy, as long as the supplied
    magnetic field vector is in the NED frame of reference. Just need to
    convert the magnetic field vector into the body frame, and also adjust by
    the orientation vector.
    */
    if(status->compass) {
        predicted.segment<3>(i) << attitude * magnetic_field;
        i += 3;
    }

    /*
    GPS position prediction is the previous estimate's lat/lon/alt.
    */
    if(status->GPSP) {
        predicted.segment<3>(i) << in.position();
        i += 3;
    }

    /*
    GPS velocity prediction is just the previous estimate's NED velocity.
    */
    if(status->GPSV) {
        predicted.segment<3>(i) << in.velocity();
        i += 3;
    }

    /*
    The barometric altitude prediction is just the height from the state
    vector position.
    */
    if(status->baro) {
        predicted[i] = in.position()[2];
        i++;
    }

    /*
    The sonar distance is the previous estimate's distance projected into body frame
    */
    if(status->sonar) {
        Vector3r unitZ(0, 0, 1);
        predicted[i] = in.sonar_ground() / (attitude * unitZ)(2);
        i++;
    }

    return predicted;
}

MeasurementVector AriaModel::get_covariance() const {
    MeasurementVector::Index max_size = (MeasurementVector::Index)size();
    MeasurementVector::Index i = 0;
    if(status->acc) {
        max_size -= 3;
    }
    MeasurementVector out_covariance(max_size);

    if(status->acc) {
        out_covariance.segment<3>(i) << covariance.segment<3>(0);
        i += 3;
    }

    if(status->gyro) {
        out_covariance.segment<3>(i) << covariance.segment<3>(3);
        i += 3;
    }

    if(status->compass) {
        out_covariance.segment<3>(i) << covariance.segment<3>(6);
        i += 3;
    }

    if(status->GPSP) {
        out_covariance.segment<3>(i) << covariance.segment<3>(9);
        i += 3;
    }

    if(status->GPSV) {
        out_covariance.segment<3>(i) << covariance.segment<3>(12);
        i += 3;
    }

    if(status->baro) {
        out_covariance[i] = covariance[15];
        i++;
    }

    if(status->sonar) {
        out_covariance[i] = covariance[16];
        i++;
    }

    return out_covariance;
}


/*
The mean is calculated differently because we first have to normalise both the
gravitational acceleration and the magnetometer value, then add the
gravitational acceleration to the kinematic acceleration to get the expected
accelerometer measurement.
*/
MeasurementVector AriaModel::calculate_mean(
const MatrixXr &in,
const VectorXr &weights) {
    MeasurementVector::Index max_size = (MeasurementVector::Index)size();
    MeasurementVector::Index i = 0, j = 0;
    if(status->acc) {
        max_size -= 3;
    }
    MeasurementVector mean(max_size);

    /* Calculate the mean. */
    MeasurementVector initial_mean = in * weights;

    if(status->acc) {
        /*
        Normalise the gravitational acceleration, and add the gravitational
        and kinematic accelerations.
        */
        mean.segment<3>(i) <<
            (initial_mean.segment<3>(j+3).normalized() * g) +
            initial_mean.segment<3>(j);

        i += 3;
        j += 6;
    }

    if(status->gyro) {
        mean.segment<3>(i) << initial_mean.segment<3>(j);
        i += 3;
        j += 3;
    }

    if(status->compass) {
        /* Normalise the magnetometer. */
        mean.segment<3>(i) << initial_mean.segment<3>(j).normalized() *
            magnetometer.field.norm();
        i += 3;
        j += 3;
    }

    if(status->GPSP) {
        mean.segment<3>(i) << initial_mean.segment<3>(j);
        i += 3;
        j += 3;
    }

    if(status->GPSV) {
        mean.segment<3>(i) << initial_mean.segment<3>(j);
        i += 3;
        j += 3;
    }

    if(status->baro) {
        mean[i] = initial_mean[j];
        i++;
        j++;
    }

    if(status->sonar) {
        mean[i] = initial_mean[j];
        i++;
        j++;
    }

    return mean;
}

/*
This is specialised because we have to add the gravitational acceleration to
the kinematic acceleration in the sigma points before subtracting the mean
from them.
*/
MatrixXr AriaModel::calculate_deltas(
const MatrixXr &in,
const MeasurementVector &mean) {
    MatrixXr deltas;
    MeasurementVector::Index max_size = (MeasurementVector::Index)in.rows();

    if(config->acc) {
        max_size -= 3;
        deltas = MatrixXr(max_size, in.cols());

        deltas.block(0, 0, 3, in.cols()) =
            in.block(0, 0, 3, in.cols()) +
            in.block(3, 0, 3, in.cols());

        if(max_size > 3) {
            deltas.block(3, 0, max_size-3, in.cols()) =
                in.block(6, 0, max_size-3, in.cols());
        }

        deltas.colwise() -= mean;
    } else {
        deltas = in.colwise() - mean;
    }

    return deltas;
}

////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014-2015, richards-tech, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "RTMath.h"

//----------------------------------------------------------
//
//  The RTVector3 class

RTVector3::RTVector3()
{
    zero();
}

RTVector3::RTVector3(float x, float y, float z)
{
    data[0] = x;
    data[1] = y;
    data[2] = z;
}

RTVector3& RTVector3::operator =(const RTVector3& vec)
{
    if (this == &vec)
        return *this;

    data[0] = vec.data[0];
    data[1] = vec.data[1];
    data[2] = vec.data[2];

    return *this;
}


const RTVector3& RTVector3::operator +=(RTVector3& vec)
{
    for (int i = 0; i < 3; i++)
        data[i] += vec.data[i];
    return *this;
}

const RTVector3& RTVector3::operator -=(RTVector3& vec)
{
    for (int i = 0; i < 3; i++)
        data[i] -= vec.data[i];
    return *this;
}

void RTVector3::zero()
{
    for (int i = 0; i < 3; i++)
        data[i] = 0;
}


float RTVector3::dotProduct(const RTVector3& a, const RTVector3& b)
{
    return a.data[0] * b.data[0] + a.data[1] * b.data[1] + a.data[2] * b.data[2];
}

void RTVector3::crossProduct(const RTVector3& a, const RTVector3& b, RTVector3& d)
{
    d.data[0] = a.data[1] * b.data[2] - a.data[2] * b.data[1];
    d.data[1] = a.data[2] * b.data[0] - a.data[0] * b.data[2];
    d.data[2] = a.data[0] * b.data[1] - a.data[1] * b.data[0];
}


void RTVector3::accelToEuler(RTVector3& rollPitchYaw) const
{
    RTVector3 normAccel = *this;

    normAccel.normalize();

    rollPitchYaw.data[0] = atan2(normAccel.data[1], normAccel.data[2]);
    rollPitchYaw.data[1] = -atan2(normAccel.data[0], sqrt(normAccel.data[1] * normAccel.data[1] + normAccel.data[2] * normAccel.data[2]));
    rollPitchYaw.data[2] = 0;
}


void RTVector3::accelToQuaternion(RTQuaternion& qPose) const
{
    RTVector3 normAccel = *this;
    RTVector3 vec;
    RTVector3 z(0, 0, 1.0);

    normAccel.normalize();

    float angle = acos(RTVector3::dotProduct(z, normAccel));
    RTVector3::crossProduct(normAccel, z, vec);
    vec.normalize();

    qPose.fromAngleVector(angle, vec);
}


void RTVector3::normalize()
{
    float length = sqrt(data[0] * data[0] + data[1] * data[1] +
            data[2] * data[2]);

    if (length == 0)
        return;

    data[0] /= length;
    data[1] /= length;
    data[2] /= length;
}

float RTVector3::length()
{
    return sqrt(data[0] * data[0] + data[1] * data[1] +
            data[2] * data[2]);
}

//----------------------------------------------------------
//
//  The RTQuaternion class

RTQuaternion::RTQuaternion()
{
    zero();
}

RTQuaternion::RTQuaternion(float scalar, float x, float y, float z)
{
    data[0] = scalar;
    data[1] = x;
    data[2] = y;
    data[3] = z;
}

RTQuaternion& RTQuaternion::operator =(const RTQuaternion& quat)
{
    if (this == &quat)
        return *this;

    data[0] = quat.data[0];
    data[1] = quat.data[1];
    data[2] = quat.data[2];
    data[3] = quat.data[3];

    return *this;
}



RTQuaternion& RTQuaternion::operator +=(const RTQuaternion& quat)
{
    for (int i = 0; i < 4; i++)
        data[i] += quat.data[i];
    return *this;
}

RTQuaternion& RTQuaternion::operator -=(const RTQuaternion& quat)
{
    for (int i = 0; i < 4; i++)
        data[i] -= quat.data[i];
    return *this;
}

RTQuaternion& RTQuaternion::operator -=(const float val)
{
    for (int i = 0; i < 4; i++)
        data[i] -= val;
    return *this;
}

RTQuaternion& RTQuaternion::operator *=(const RTQuaternion& qb)
{
    RTQuaternion qa;

    qa = *this;

    data[0] = qa.data[0] * qb.data[0] - qa.data[1] * qb.data[1] - qa.data[2] * qb.data[2] - qa.data[3] * qb.data[3];
    data[1] = qa.data[0] * qb.data[1] + qa.data[1] * qb.data[0] + qa.data[2] * qb.data[3] - qa.data[3] * qb.data[2];
    data[2] = qa.data[0] * qb.data[2] - qa.data[1] * qb.data[3] + qa.data[2] * qb.data[0] + qa.data[3] * qb.data[1];
    data[3] = qa.data[0] * qb.data[3] + qa.data[1] * qb.data[2] - qa.data[2] * qb.data[1] + qa.data[3] * qb.data[0];

    return *this;
}


RTQuaternion& RTQuaternion::operator *=(const float val)
{
    data[0] *= val;
    data[1] *= val;
    data[2] *= val;
    data[3] *= val;

    return *this;
}


const RTQuaternion RTQuaternion::operator *(const RTQuaternion& qb) const
{
    RTQuaternion result = *this;
    result *= qb;
    return result;
}

const RTQuaternion RTQuaternion::operator *(const float val) const
{
    RTQuaternion result = *this;
    result *= val;
    return result;
}


const RTQuaternion RTQuaternion::operator -(const RTQuaternion& qb) const
{
    RTQuaternion result = *this;
    result -= qb;
    return result;
}

const RTQuaternion RTQuaternion::operator -(const float val) const
{
    RTQuaternion result = *this;
    result -= val;
    return result;
}


void RTQuaternion::zero()
{
    for (int i = 0; i < 4; i++)
        data[i] = 0;
}

void RTQuaternion::normalize()
{
    float length = sqrt(data[0] * data[0] + data[1] * data[1] +
            data[2] * data[2] + data[3] * data[3]);

    if ((length == 0) || (length == 1))
        return;

    data[0] /= length;
    data[1] /= length;
    data[2] /= length;
    data[3] /= length;
}

void RTQuaternion::toEuler(RTVector3& vec)
{
    vec.data[0] = atan2(2.0 * (data[2] * data[3] + data[0] * data[1]), 1 - 2.0 * (data[1] * data[1] + data[2] * data[2]));

    vec.data[1] = asin(2.0 * (data[0] * data[2] - data[1] * data[3]));

    vec.data[2] = atan2(2.0 * (data[1] * data[2] + data[0] * data[3]), 1 - 2.0 * (data[2] * data[2] + data[3] * data[3]));
}

void RTQuaternion::fromEuler(RTVector3& vec)
{
    float cosX2 = cos(vec.data[0] / 2.0f);
    float sinX2 = sin(vec.data[0] / 2.0f);
    float cosY2 = cos(vec.data[1] / 2.0f);
    float sinY2 = sin(vec.data[1] / 2.0f);
    float cosZ2 = cos(vec.data[2] / 2.0f);
    float sinZ2 = sin(vec.data[2] / 2.0f);

    data[0] = cosX2 * cosY2 * cosZ2 + sinX2 * sinY2 * sinZ2;
    data[1] = sinX2 * cosY2 * cosZ2 - cosX2 * sinY2 * sinZ2;
    data[2] = cosX2 * sinY2 * cosZ2 + sinX2 * cosY2 * sinZ2;
    data[3] = cosX2 * cosY2 * sinZ2 - sinX2 * sinY2 * cosZ2;
    normalize();
}

RTQuaternion RTQuaternion::conjugate() const
{
    RTQuaternion q;
    q.data[0] = data[0];
    q.data[1] = -data[1];
    q.data[2] = -data[2];
    q.data[3] = -data[3];
    return q;
}

void RTQuaternion::toAngleVector(float& angle, RTVector3& vec)
{
    float halfTheta;
    float sinHalfTheta;

    halfTheta = acos(data[0]);
    sinHalfTheta = sin(halfTheta);

    if (sinHalfTheta == 0) {
        vec.data[0] = 1.0;
        vec.data[1] = 0;
        vec.data[2] = 0;
    } else {
        vec.data[0] = data[1] / sinHalfTheta;
        vec.data[1] = data[1] / sinHalfTheta;
        vec.data[2] = data[1] / sinHalfTheta;
    }
    angle = 2.0 * halfTheta;
}

void RTQuaternion::fromAngleVector(const float& angle, const RTVector3& vec)
{
    float sinHalfTheta = sin(angle / 2.0);
    data[0] = cos(angle / 2.0);
    data[1] = vec.data[0] * sinHalfTheta;
    data[2] = vec.data[1] * sinHalfTheta;
    data[3] = vec.data[2] * sinHalfTheta;
}



//----------------------------------------------------------
//
//  The RTMatrix4x4 class

RTMatrix4x4::RTMatrix4x4()
{
    fill(0);
}

RTMatrix4x4& RTMatrix4x4::operator =(const RTMatrix4x4& mat)
{
    if (this == &mat)
        return *this;

    for (int row = 0; row < 4; row++)
        for (int col = 0; col < 4; col++)
            data[row][col] = mat.data[row][col];

    return *this;
}


void RTMatrix4x4::fill(float val)
{
    for (int row = 0; row < 4; row++)
        for (int col = 0; col < 4; col++)
            data[row][col] = val;
}


RTMatrix4x4& RTMatrix4x4::operator +=(const RTMatrix4x4& mat)
{
    for (int row = 0; row < 4; row++)
        for (int col = 0; col < 4; col++)
            data[row][col] += mat.data[row][col];

    return *this;
}

RTMatrix4x4& RTMatrix4x4::operator -=(const RTMatrix4x4& mat)
{
    for (int row = 0; row < 4; row++)
        for (int col = 0; col < 4; col++)
            data[row][col] -= mat.data[row][col];

    return *this;
}

RTMatrix4x4& RTMatrix4x4::operator *=(const float val)
{
    for (int row = 0; row < 4; row++)
        for (int col = 0; col < 4; col++)
            data[row][col] *= val;

    return *this;
}

const RTMatrix4x4 RTMatrix4x4::operator +(const RTMatrix4x4& mat) const
{
    RTMatrix4x4 result = *this;
    result += mat;
    return result;
}

const RTMatrix4x4 RTMatrix4x4::operator *(const float val) const
{
    RTMatrix4x4 result = *this;
    result *= val;
    return result;
}


const RTMatrix4x4 RTMatrix4x4::operator *(const RTMatrix4x4& mat) const
{
    RTMatrix4x4 res;

    for (int row = 0; row < 4; row++)
        for (int col = 0; col < 4; col++)
            res.data[row][col] =
                    data[row][0] * mat.data[0][col] +
                    data[row][1] * mat.data[1][col] +
                    data[row][2] * mat.data[2][col] +
                    data[row][3] * mat.data[3][col];

    return res;
}


const RTQuaternion RTMatrix4x4::operator *(const RTQuaternion& q) const
{
    RTQuaternion res;

    res.data[0] = data[0][0] * q.data[0] + data[0][1] * q.data[1] + data[0][2] * q.data[2] + data[0][3] * q.data[3];
    res.data[1] = data[1][0] * q.data[0] + data[1][1] * q.data[1] + data[1][2] * q.data[2] + data[1][3] * q.data[3];
    res.data[2] = data[2][0] * q.data[0] + data[2][1] * q.data[1] + data[2][2] * q.data[2] + data[2][3] * q.data[3];
    res.data[3] = data[3][0] * q.data[0] + data[3][1] * q.data[1] + data[3][2] * q.data[2] + data[3][3] * q.data[3];

    return res;
}

void RTMatrix4x4::setToIdentity()
{
    fill(0);
    data[0][0] = 1;
    data[1][1] = 1;
    data[2][2] = 1;
    data[3][3] = 1;
}

RTMatrix4x4 RTMatrix4x4::transposed()
{
    RTMatrix4x4 res;

    for (int row = 0; row < 4; row++)
        for (int col = 0; col < 4; col++)
            res.data[col][row] = data[row][col];
    return res;
}

//  Note:
//  The matrix inversion code here was strongly influenced by some old code I found
//  but I have no idea where it came from. Apologies to whoever wrote it originally!
//  If it's you, please let me know at info@richards-tech.com so I can credit it correctly.

RTMatrix4x4 RTMatrix4x4::inverted()
{
    RTMatrix4x4 res;

    float det = matDet();

    if (det == 0) {
        res.setToIdentity();
        return res;
    }

    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            if ((row + col) & 1)
                res.data[col][row] = -matMinor(row, col) / det;
            else
                res.data[col][row] = matMinor(row, col) / det;
        }
    }

    return res;
}

float RTMatrix4x4::matDet()
{
    float det = 0;

    det += data[0][0] * matMinor(0, 0);
    det -= data[0][1] * matMinor(0, 1);
    det += data[0][2] * matMinor(0, 2);
    det -= data[0][3] * matMinor(0, 3);
    return det;
}

float RTMatrix4x4::matMinor(const int row, const int col)
{
    static int map[] = {1, 2, 3, 0, 2, 3, 0, 1, 3, 0, 1, 2};

    int *rc;
    int *cc;
    float res = 0;

    rc = map + row * 3;
    cc = map + col * 3;

    res += data[rc[0]][cc[0]] * data[rc[1]][cc[1]] * data[rc[2]][cc[2]];
    res -= data[rc[0]][cc[0]] * data[rc[1]][cc[2]] * data[rc[2]][cc[1]];
    res -= data[rc[0]][cc[1]] * data[rc[1]][cc[0]] * data[rc[2]][cc[2]];
    res += data[rc[0]][cc[1]] * data[rc[1]][cc[2]] * data[rc[2]][cc[0]];
    res += data[rc[0]][cc[2]] * data[rc[1]][cc[0]] * data[rc[2]][cc[1]];
    res -= data[rc[0]][cc[2]] * data[rc[1]][cc[1]] * data[rc[2]][cc[0]];
    return res;
}


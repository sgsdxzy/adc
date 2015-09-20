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

#ifndef _RTMATH_H_
#define _RTMATH_H_

#include <cmath>

class RTVector3;
class RTMatrix4x4;
class RTQuaternion;

class RTVector3
{
public:
    RTVector3();
    RTVector3(float x, float y, float z);

    const RTVector3&  operator +=(RTVector3& vec);
    const RTVector3&  operator -=(RTVector3& vec);

    RTVector3& operator =(const RTVector3& vec);

    float length();
    void normalize();
    void zero();

    static float dotProduct(const RTVector3& a, const RTVector3& b);
    static void crossProduct(const RTVector3& a, const RTVector3& b, RTVector3& d);

    void accelToEuler(RTVector3& rollPitchYaw) const;
    void accelToQuaternion(RTQuaternion& qPose) const;

    inline void fromArray(float val[3]) { memcpy(data, val, 3 * sizeof(float)); }
    inline void toArray(float val[3]) const { memcpy(val, data, 3 * sizeof(float)); }

    float data[3];
};


class RTQuaternion
{
public:
    RTQuaternion();
    RTQuaternion(float scalar, float x, float y, float z);

    RTQuaternion& operator +=(const RTQuaternion& quat);
    RTQuaternion& operator -=(const RTQuaternion& quat);
    RTQuaternion& operator *=(const RTQuaternion& qb);
    RTQuaternion& operator *=(const float val);
    RTQuaternion& operator -=(const float val);

    RTQuaternion& operator =(const RTQuaternion& quat);
    const RTQuaternion operator *(const RTQuaternion& qb) const;
    const RTQuaternion operator *(const float val) const;
    const RTQuaternion operator -(const RTQuaternion& qb) const;
    const RTQuaternion operator -(const float val) const;

    void normalize();
    void toEuler(RTVector3& vec);
    void fromEuler(RTVector3& vec);
    RTQuaternion conjugate() const;
    void toAngleVector(float& angle, RTVector3& vec);
    void fromAngleVector(const float& angle, const RTVector3& vec);

    void zero();

    inline void fromArray(float val[4]) { memcpy(data, val, 4 * sizeof(float)); }
    inline void toArray(float val[4]) const { memcpy(val, data, 4 * sizeof(float)); }

    float data[4];
};

class RTMatrix4x4
{
public:
    RTMatrix4x4();

    RTMatrix4x4& operator +=(const RTMatrix4x4& mat);
    RTMatrix4x4& operator -=(const RTMatrix4x4& mat);
    RTMatrix4x4& operator *=(const float val);

    RTMatrix4x4& operator =(const RTMatrix4x4& vec);
    const RTQuaternion operator *(const RTQuaternion& q) const;
    const RTMatrix4x4 operator *(const float val) const;
    const RTMatrix4x4 operator *(const RTMatrix4x4& mat) const;
    const RTMatrix4x4 operator +(const RTMatrix4x4& mat) const;

    inline float val(int row, int col) const { return m_data[row][col]; }
    inline void setVal(int row, int col, float val) { m_data[row][col] = val; }
    void fill(float val);
    void setToIdentity();

    RTMatrix4x4 inverted();
    RTMatrix4x4 transposed();

    float data[4][4];                                   // row, column

    float matDet();
    float matMinor(const int row, const int col);
};

#endif /* _RTMATH_H_ */

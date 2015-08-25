#include "../debug.h"
#include "GY87.h"

#include <iostream>

using namespace std;

void GY87::initialize()
{
    // MPU6050
    mpu.initialize();
    if (!mpu.testConnection()) {
        // Test failed
        err("MPU6050 test connection failed!");
    }
    uint8_t mpuStatus = mpu.dmpInitialize();
    if (mpuStatus != 0) {
        err("MPU6050 DMP setup failed!");
    }

    // Setting HMC5883L
    mpu.setI2CMasterModeEnabled(0);
    mpu.setI2CBypassEnabled(1);
    // Now we can see HMC5883L
    HMC5883L hmc(HMC5883L_DEFAULT_ADDRESS);
    hmc.initialize();
    if (!hmc.testConnection()) {
        // Test failed
        err("HMC5883L test connection failed!");
    }
    hmc.setDataRate(HMC5883L_RATE_75);
    hmc.setMode(HMC5883L_MODE_CONTINUOUS);
    mpu.setI2CBypassEnabled(0);

    mpu.setSlaveAddress(0, HMC5883L_DEFAULT_ADDRESS | 0x80); // 0x80 turns 7th bit ON, according to datasheet, 7th bit controls Read/Write direction
    mpu.setSlaveRegister(0, HMC5883L_RA_DATAX_H);
    mpu.setSlaveEnabled(0, true);
    mpu.setSlaveWordByteSwap(0, false);
    mpu.setSlaveWriteMode(0, false);
    mpu.setSlaveWordGroupOffset(0, false);
    mpu.setSlaveDataLength(0, 2);

    // Y axis word
    mpu.setSlaveAddress(1, HMC5883L_DEFAULT_ADDRESS | 0x80);
    mpu.setSlaveRegister(1, HMC5883L_RA_DATAY_H);
    mpu.setSlaveEnabled(1, true);
    mpu.setSlaveWordByteSwap(1, false);
    mpu.setSlaveWriteMode(1, false);
    mpu.setSlaveWordGroupOffset(1, false);
    mpu.setSlaveDataLength(1, 2);

    // Z axis word
    mpu.setSlaveAddress(2, HMC5883L_DEFAULT_ADDRESS | 0x80);
    mpu.setSlaveRegister(2, HMC5883L_RA_DATAZ_H);
    mpu.setSlaveEnabled(2, true);
    mpu.setSlaveWordByteSwap(2, false);
    mpu.setSlaveWriteMode(2, false);
    mpu.setSlaveWordGroupOffset(2, false);
    mpu.setSlaveDataLength(2, 2);

    mpu.setI2CMasterModeEnabled(1);
    //Setting HMC5883L Done
    
    mpu.setFullScaleAccelRange(0);
    mpu.setFullScaleGyroRange(1);

    setOffset();
    
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();


    //BMP085 TODO

}

void GY87::setOffset()
{
    //Calibrated results
    mpu.setXAccelOffset(-1557);
    mpu.setYAccelOffset(3950);
    mpu.setZAccelOffset(-1962);
    mpu.setXGyroOffset(-7);
    mpu.setYGyroOffset(-8);
    mpu.setZGyroOffset(112);
}


void GY87::calibrate(int samples, int tolerance)
{
    // MPU6050
    int xa;
    int ya;
    int za;
    int xg;
    int yg;
    int zg;

    int scale;

    scale = mpu.getFullScaleAccelRange();
    int accRange = pow(2, scale);
    scale = mpu.getFullScaleGyroRange();
    int gyroRange = pow(2, scale);

    int g = 16384/accRange; //How much is 1g

    int i;
    int xaa, yaa, zaa, xga, yga, zga;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int error=32767;


    while (error > tolerance) {
        xa = mpu.getXAccelOffset();
        ya = mpu.getYAccelOffset();
        za = mpu.getZAccelOffset();
        xg = mpu.getXGyroOffset();
        yg = mpu.getYGyroOffset();
        zg = mpu.getZGyroOffset();
        xaa=0, yaa=0, zaa=0, xga=0, yga=0, zga=0;
        for (i=0;i<samples;i++) {
            mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            xaa += ax;
            yaa += ay;
            zaa += az;
            xga += gx;
            yga += gy;
            zga += gz;
            bcm2835_delay(10);
        }

        xaa /= samples;
        yaa /= samples;
        zaa /= samples;
        xga /= samples;
        yga /= samples;
        zga /= samples;

        mpu.setXAccelOffset(xa-xaa/8*accRange);
        mpu.setYAccelOffset(ya-yaa/8*accRange);
        mpu.setZAccelOffset(za+(-g-zaa)/8*accRange);
        mpu.setXGyroOffset(xg-xga/8*gyrpRange);
        mpu.setYGyroOffset(yg-yga/8*gyroRange);
        mpu.setZGyroOffset(zg-zga/8*gyroRange);

        error = abs(xaa);
        if (error < abs(yaa)) error = abs(yaa);
        if (error < abs(-g-zaa)) error = abs(-g-zaa);
        if (error < abs(xga)) error = abs(xga);
        if (error < abs(yga)) error = abs(yga);
        if (error < abs(zga)) error = abs(zga);
        info("Calibrated result:"+to_string(error));
    }

    info("Calibration finished!");
}

bool GY87::testConnection()
{
    return mpu.testConnection() && bmp.testConnection();
}  

void GY87::update()
{
    fifoCount = mpu.getFIFOCount();
    if (fifoCount == 1024) {
        // Overflow, reset so we can continue cleanly
        mpu.resetFIFO();
        warn("FIFO Overflow!");
    } else {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGyro(gyro, fifoBuffer);
        //    mpu.dmpGetAccel(&aa, fifoBuffer);  //Use this if you want accelerometer measures
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        //    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);  //Use this to get linear acceleration apart from gravity.
        //    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);  //NOT RECOMMENDED. Gives you linear acceleration rotated to initial position.

        //Read magnetometer measures
        mx=mpu.getExternalSensorWord(0);
        my=mpu.getExternalSensorWord(2);
        mz=mpu.getExternalSensorWord(4);

        float xh = mx*cos(ypr[1])+my*sin(ypr[1])*sin(ypr[2])+mz*sin(ypr[1])*cos(ypr[2]);
        float yh = my*cos(ypr[2])+mz*sin(ypr[2]);
        heading = atan2(-yh, xh);
        if(heading < 0) heading += 2 * M_PI;
    }
}







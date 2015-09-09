#include "GY87.h"

void GY87::initialize()
{
    // Setting MPU6050
    if (!mpu.testConnection()) {
        // Test failed
        err("MPU6050 test connection failed!");
    }
    mpu.initialize();
    uint8_t mpuStatus = mpu.dmpInitialize();
    if (mpuStatus != 0) {
        err("MPU6050 DMP setup failed! Code: "+to_string(mpuStatus));
    }
    packetSize = mpu.dmpGetFIFOPacketSize();
    // Setting MPU6050 done

    // Setting HMC5883L
    mpu.setI2CMasterModeEnabled(0);
    mpu.setI2CBypassEnabled(1);
    // Now we can see HMC5883L
    HMC5883L hmc(HMC5883L_DEFAULT_ADDRESS);
    if (!hmc.testConnection()) {
        // Test failed
        err("HMC5883L test connection failed!");
    }
    hmc.initialize();
    hmc.setSampleAveraging(HMC5883L_AVERAGING_8);
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
    // Setting HMC5883L Done
    
    // Setting MPU6050 slave sample rate to 100Hz
    mpu.setSlave4MasterDelay(1); // 200/(1+1)=100
    mpu.setSlaveDelayEnabled(0, true);
    mpu.setSlaveDelayEnabled(1, true);
    mpu.setSlaveDelayEnabled(2, true);

    //BMP180
    if (!bmp.testConnection()) {
        // Test failed
        err("BMP180 test connection failed!");
    }
    bmp.initialize();
}

void GY87::setOffset(int16_t gy87Offset[6])
{
    //Calibrated results
    mpu.setXAccelOffset(gy87Offset[0]);
    mpu.setYAccelOffset(gy87Offset[1]);
    mpu.setZAccelOffset(gy87Offset[2]);
    mpu.setXGyroOffset(gy87Offset[3]);
    mpu.setYGyroOffset(gy87Offset[4]);
    mpu.setZGyroOffset(gy87Offset[5]);
}

bool GY87::testConnection()
{
    return mpu.testConnection() && bmp.testConnection();
}  

void GY87::startDMP()
{
    mpu.setDMPEnabled(true);
}    

void GY87::reset()
{
    mpu.reset();
    gpioDelay(30000);
}

void GY87::updateMPU()
{
    // Making I2C operations thread-safe
    pthread_mutex_lock(&i2cMutex);
    fifoCount = mpu.getFIFOCount();
    if (fifoCount == 1024) {
        // Overflow, reset so we can continue cleanly
        mpu.resetFIFO();
        warn("FIFO Overflow!");
        return;
    } 
    if (fifoCount < packetSize) {
        warn("Interrupt received but data not ready!");
        return;
    }
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    //Read magnetometer measures
    mx=mpu.getExternalSensorWord(0);
    my=mpu.getExternalSensorWord(2);
    mz=mpu.getExternalSensorWord(4);
    pthread_mutex_unlock(&i2cMutex);

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGyro(&gyro, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);  //Use this if you want accelerometer measures
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);  //Use this to get linear acceleration apart from gravity.
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);  //NOT RECOMMENDED. Gives you linear acceleration rotated to initial position.

    float xh = mx*cos(ypr[1])+my*sin(ypr[1])*sin(ypr[2])+mz*sin(ypr[1])*cos(ypr[2]);
    float yh = my*cos(ypr[2])+mz*sin(ypr[2]);
    mh = atan2(-yh, xh);
    if (mh < 0) mh += 2*M_PI;
}

void GY87::updateBMP(float seaLevelPressure)
{
    if (BMPCounter == 0) {
        pthread_mutex_lock(&i2cMutex);
        bmp.setControl(BMP085_MODE_TEMPERATURE);
        pthread_mutex_unlock(&i2cMutex);
        gpioDelay(4500); // wait 4.5 ms for conversion 
        pthread_mutex_lock(&i2cMutex);
        temperature = bmp.getTemperatureC();
        pthread_mutex_unlock(&i2cMutex);
        sonicVelocity = 331.3 * sqrt(1 + temperature/273.15); // m/s
    } else {
        pthread_mutex_lock(&i2cMutex);
        bmp.setControl(BMP085_MODE_PRESSURE_1) ; //taking reading in highest accuracy measurement mode
        pthread_mutex_unlock(&i2cMutex);
        gpioDelay(7500);
        pthread_mutex_lock(&i2cMutex);
        pressure = bmp.getPressure();
        pthread_mutex_unlock(&i2cMutex);

        baroAltitude = bmp.getAltitude(pressure, seaLevelPressure);
    }

    BMPCounter += 1;
    BMPCounter = BMPCounter % 100;
}

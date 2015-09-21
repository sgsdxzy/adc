#include "GY87.h"

void GY87::initialize(Configuration* config)
{
    // Setting MPU6050
    if (!mpu.testConnection()) {
        // Test failed
        err("MPU6050 test connection failed!");
    }
    mpu.initialize();
    mpu.setDLPFMode(config->MPU6050DLPFMode);
    uint8_t rateDivider;
    if (config->MPU6050DLPFMode == MPU6050_DLPF_BW_256) {
        rateDivider = 8000/config->MPU6050Rate - 1;
    } else {
        rateDivider = 1000/config->MPU6050Rate - 1;
    }
    mpu.setRate(rateDivider);
    mpu.setOffsets(config->MPU6050Offsets);
    mpu.setFullScaleGyroRange(config->MPU6050GyroFsr);
    mpu.setFullScaleAccelRange(config->MPU6050AccelFSr);

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
    hmc.setSampleAveraging(config->HMC5883LSampleAveraging);
    hmc.setDataRate(config->HMC5883LDataRate);
    hmc.setMode(HMC5883L_MODE_CONTINUOUS);

    mpu.setI2CBypassEnabled(0);
    mpu.setI2CMasterModeEnabled(1);

    // Setting MPU6050 to get HMC5883L data
    mpu.setSlaveAddress(0, HMC5883L_DEFAULT_ADDRESS | 0x80); // 0x80 turns 7th bit ON, according to datasheet, 7th bit controls Read/Write direction
    mpu.setSlaveRegister(0, HMC5883L_RA_DATAX_H);
    mpu.setSlaveEnabled(0, true);
    mpu.setSlaveWordByteSwap(0, false);
    mpu.setSlaveWriteMode(0, false);
    mpu.setSlaveWordGroupOffset(0, false);
    mpu.setSlaveDataLength(0, 6);
    // Setting HMC5883L done
    
    // Setting MPU6050 slave sample rate
    mpu.setSlave4MasterDelay(config->MPU6050Rate/75 - 1);
    mpu.setSlaveDelayEnabled(0, true);

    // Using data ready interrupts for an accurate clock
    mpu.setIntDataReadyEnabled(true);
    // Setting MPU6050+HMC5883L done

    //BMP180
    seaLevelPressure = config->seaLevelPressure;
    if (!bmp.testConnection()) {
        // Test failed
        err("BMP180 test connection failed!");
    }
    bmp.initialize();
}

bool GY87::testConnection()
{
    return mpu.testConnection() && bmp.testConnection();
}  

void GY87::reset()
{
    mpu.reset();
}

void GY87::getData(Status* status)
{
    mpu.getMotion10((float[3])status->accRaw.data(), (float[3])status->gyroRaw.data(), (float[3])status->compassRaw.data(), &status->temperature); 
    status->pressure = bmp.getPressure();

    // Set pressure mesurement here so we can read data in next getData
    bmp.setControl(BMP085_MODE_PRESSURE_0); //taking reading in fastest mode

    status->sonicVelocity = 331.3 * sqrt(1 + status->temperature/273.15);
    status->baroAltitude = bmp.getAltitude(status->pressure, seaLevelPressure);
}

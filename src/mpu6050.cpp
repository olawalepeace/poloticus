#include "mpu6050.h"

#include <stdio.h>
#include "pico/stdlib.h"


/**
 * Binds the sensor driver to the supplied IMU data container
 *
 * @param data The shared data structure that stores the latest accelerometer and gyroscope values.
 */
MPU6050::MPU6050(IMUData& data) : imu_data_(data) {}

/**
 * Initializes the I2C bus and wakes the MPU6050 so it is ready to return sensor data.
 * Strongly not adviced to run in main loop
 *
 * @param device_address The 7-bit address of the MPU6050 device on the bus.
 * @param sda The GPIO pin used for the I2C data line.
 * @param scl The GPIO pin used for the I2C clock line.
 * @param read_timeout_ms The timeout to use for I2C transactions in milliseconds.
 */
void MPU6050::initialize(uint8_t device_address,uint8_t sda, uint8_t scl, uint32_t timeout) {
    this->read_timeout_ms_ = timeout;
    this->device_address_ = device_address;
    I2cDevice::initialize(sda, scl, 400000); // Initialize I2C with SDA=sda, SCL=scl, baudrate=400kHz
    uint8_t data[1] = {0x01}; // Data to write to the power management register, this will prevent it from going to sleep or wake it from sleep, something about that in the datasheet.
    I2cDevice::writeBytes(device_address_, MPU6050_PWR_MGMT_1, data, sizeof(data), read_timeout_ms_);
    // calibrateImu();
}

/**
 * Reads the accelerometer and gyroscope registers from the MPU6050 and updates the internal or parameter IMU struct.
 *
 * @param data A pointer to ImuData struct, can be null in orfer to use the internal `imu_data_`
 * The routine prints a short error message if the read completes with a timeout or an incomplete payload.
 */
IMUData MPU6050::readImuData(IMUData* data) {
    uint8_t raw_data[14]; // I'm doing 6 bytes for accel, 2 bytes for temp, 6 bytes for gyro
    
    IMUData& imu_data = (data != nullptr) ? *data : imu_data_;

    int8_t count = I2cDevice::readBytes(device_address_, MPU6050_ACCEL_XOUT_H, raw_data, sizeof(raw_data), read_timeout_ms_);
    // if (count == PICO_ERROR_GENERIC) {
    //     printf("I2C read error: PICO_ERROR_GENERIC\n");
    //     return;
    // }
    // if (count == PICO_ERROR_TIMEOUT) {
    //     printf("I2C read error: PICO_ERROR_TIMEOUT\n");
    //     return;
    // }
    // if (count != sizeof(raw_data)) {
    //     printf("I2C read error: Incomplete data read\n");
    //     return;
    // }

    imu_data.timestamp_us = static_cast<uint64_t>(to_us_since_boot(get_absolute_time()));
    imu_data.accel_data = readAccelData(&raw_data[0]);
    imu_data.gyro_data = readGyroData(&raw_data[8]);

    return imu_data;

}

AccelData MPU6050::readAccelData(uint8_t* raw_data) {
    AccelData accel_data;
    uint8_t local_data[6];
    if (raw_data == nullptr) {
        int8_t count = I2cDevice::readBytes(device_address_, MPU6050_ACCEL_XOUT_H, local_data, sizeof(local_data), read_timeout_ms_);
        raw_data = local_data;
    }

    accel_data.accel_x = ((static_cast<int16_t>((raw_data[0] << 8) | raw_data[1])) / 16384.0f)*G_TO_MPS2 - accel_offset_.accel_x;
    accel_data.accel_y = ((static_cast<int16_t>((raw_data[2] << 8) | raw_data[3])) / 16384.0f)*G_TO_MPS2 - accel_offset_.accel_y;
    accel_data.accel_z = ((static_cast<int16_t>((raw_data[4] << 8) | raw_data[5])) / 16384.0f)*G_TO_MPS2 - accel_offset_.accel_z;

    return accel_data;
}

GyroData MPU6050::readGyroData(uint8_t* raw_data) {
    GyroData gyro_data;
    uint8_t local_data[6];
    if (raw_data == nullptr) {
        int8_t count = I2cDevice::readBytes(device_address_, MPU6050_GYRO_XOUT_H, local_data, sizeof(local_data), read_timeout_ms_);
        raw_data = local_data;
    }

    gyro_data.gyro_x = ((static_cast<int16_t>((raw_data[0] << 8) | raw_data[1])) / 131.0f)*DEG_TO_RAD - gyro_offset_.gyro_x;
    gyro_data.gyro_y = ((static_cast<int16_t>((raw_data[2] << 8) | raw_data[3])) / 131.0f)*DEG_TO_RAD - gyro_offset_.gyro_y;
    gyro_data.gyro_z = ((static_cast<int16_t>((raw_data[4] << 8) | raw_data[5])) / 131.0f)*DEG_TO_RAD - gyro_offset_.gyro_z;

    return gyro_data;
}

/**
 * Estimates the IMU sensor biases while(assumung that) the sensor is stationary.
 *
 * The estimated accelerometer and gyroscope biases are stored and subtracted from sensor measurements during normal
 * operation.
 */
// void MPU6050::calibrateImu()
// {
//     int size = 1000;
//     ImuData imu_bias;
//     ImuData calibration_data;

//     for(int i=0; i<size; i++){
//         readImuData(&calibration_data);
//         imu_bias += calibration_data;
//         sleep_ms(10);
//     }

//     imu_bias /= size;
//     imu_bias.accel_z -= G_TO_MPS2;

//     imu_offset_ = imu_bias;
// }


GyroData& GyroData::operator+=(const GyroData& other)
{
    gyro_x += other.gyro_x;
    gyro_y += other.gyro_y;
    gyro_z += other.gyro_z;

    return *this;
}

GyroData& GyroData::operator/=(const int divisor)
{
    gyro_x /=divisor;
    gyro_y /=divisor;
    gyro_z /=divisor;

    return *this;
}

AccelData& AccelData::operator+=(const AccelData& other)
{
    accel_x += other.accel_x;
    accel_y += other.accel_y;
    accel_z += other.accel_z;

    return *this;
}

AccelData& AccelData::operator/=(const int divisor)
{
    accel_x /=divisor;
    accel_y /=divisor;
    accel_z /=divisor;

    return *this;
}

#include "mpu6050.h"

#include <stdio.h>
#include "pico/stdlib.h"


/**
 * Binds the sensor driver to the supplied IMU data container
 *
 * @param data The shared data structure that stores the latest accelerometer and gyroscope values.
 */
MPU6050::MPU6050(ImuData& data) : imu_data_(data) {}

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
    
}

/**
 * Reads the accelerometer and gyroscope registers from the MPU6050 and updates the internal or parameter IMU struct.
 *
 * @param data A pointer to ImuData struct, can be null in orfer to use the internal `imu_data_`
 * The routine prints a short error message if the read completes with a timeout or an incomplete payload.
 */
void MPU6050::readImuData(ImuData* data) {
    uint8_t raw_data[14]; // I'm doing 6 bytes for accel, 2 bytes for temp, 6 bytes for gyro
    int8_t count = I2cDevice::readBytes(device_address_, MPU6050_ACCEL_XOUT_H, raw_data, sizeof(raw_data), read_timeout_ms_);
    if (count == PICO_ERROR_GENERIC) {
        printf("I2C read error: PICO_ERROR_GENERIC\n");
        return;
    }
    if (count == PICO_ERROR_TIMEOUT) {
        printf("I2C read error: PICO_ERROR_TIMEOUT\n");
        return;
    }
    if (count != sizeof(raw_data)) {
        printf("I2C read error: Incomplete data read\n");
        return;
    }

    interpretRawData(raw_data, data);
}

/**
 * Decodes a raw MPU6050 register block into signed acceleration and gyroscope values.
 *
 * @param raw_data A pointer to 14 bytes containing the accelerometer, temperature, and gyroscope samples.
 * @param data A pointer to ImuData struct, can be null in orfer to use the internal `imu_data_`
 * @return 0 on success.
 */
int8_t MPU6050::interpretRawData(const uint8_t* raw_data, ImuData* data) {
    ImuData& imu_data = (data != nullptr) ? *data : imu_data_;

    // Convert raw accelerometer data to g's
    imu_data.accel_x = ((static_cast<int16_t>((raw_data[0] << 8) | raw_data[1])) / 16384.0f) - imu_offset_.accel_x;
    imu_data.accel_y = ((static_cast<int16_t>((raw_data[2] << 8) | raw_data[3])) / 16384.0f) - imu_offset_.accel_y;
    imu_data.accel_z = ((static_cast<int16_t>((raw_data[4] << 8) | raw_data[5])) / 16384.0f) - imu_offset_.accel_z;

    // Convert raw gyroscope data to degrees/sec
    imu_data.gyro_x = ((static_cast<int16_t>((raw_data[8] << 8) | raw_data[9])) / 131.0f) - imu_offset_.gyro_x;
    imu_data.gyro_y = ((static_cast<int16_t>((raw_data[10] << 8) | raw_data[11])) / 131.0f) - imu_offset_.gyro_y;
    imu_data.gyro_z = ((static_cast<int16_t>((raw_data[12] << 8) | raw_data[13])) / 131.0f) - imu_offset_.gyro_z;

    // Set the timestamp
    imu_data.timestamp_us = static_cast<uint64_t>(to_us_since_boot(get_absolute_time()));
    return 0;
}

/**
 * Estimates the IMU sensor biases while(assumung that) the sensor is stationary.
 *
 * The estimated accelerometer and gyroscope biases are stored and subtracted from sensor measurements during normal
 * operation.
 */
void MPU6050::calibrateImu()
{
    int size = 10;
    ImuData imu_bias;
    ImuData calibration_data;

    for(int i=0; i<size; i++){
        readImuData(&calibration_data);
        imu_bias.accel_x += calibration_data.accel_x;
        imu_bias.accel_y += calibration_data.accel_y;
        imu_bias.accel_z += calibration_data.accel_z;
        imu_bias.gyro_x += calibration_data.gyro_x;
        imu_bias.gyro_y += calibration_data.gyro_y;
        imu_bias.gyro_z += calibration_data.gyro_z;
        printf(".");
        sleep_ms(20);
    }
    
    imu_bias.accel_x /= size;
    imu_bias.accel_y /= size;
    imu_bias.accel_z /= size;
    imu_bias.gyro_x /= size;
    imu_bias.gyro_y /= size;
    imu_bias.gyro_z /= size;

    imu_bias.accel_z -= 9.8;

    imu_offset_ = imu_bias;
}

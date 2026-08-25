#include "mpu6050.h"
#include "pico/time.h"
#include <stdio.h>

MPU6050::MPU6050(ImuData& data) : imu_data(data) {
    // Constructor implementation
    imu_data.accel_x = 0.0f;
    imu_data.accel_y = 0.0f;
    imu_data.accel_z = 0.0f;
    imu_data.gyro_x = 0.0f;
    imu_data.gyro_y = 0.0f;
    imu_data.gyro_z = 0.0f;
    imu_data.timestamp_us = 0;

}

void MPU6050::initialize(uint8_t device_address,uint8_t sda, uint8_t scl, uint32_t read_timeout_ms) {
    // Wake up the MPU6050 by writing 0 to the power management register
    this->read_timeout_ms = read_timeout_ms;
    this->device_address = device_address;
    I2cDevice::initialize(sda, scl, 400000); // Initialize I2C with SDA=sda, SCL=scl, baudrate=400kHz
    uint8_t data[1] = {0x01}; // Data to write to the power management register]
    I2cDevice::writeBytes(device_address, MPU6050_PWR_MGMT_1, data, sizeof(data), read_timeout_ms);

}

void MPU6050::read_imu_data() {
    uint8_t raw_data[14]; // 6 bytes for accel, 2 bytes for temp, 6 bytes for gyro
    int8_t count = I2cDevice::readBytes(device_address, MPU6050_ACCEL_XOUT_H, raw_data, sizeof(raw_data), read_timeout_ms);
    if (count == PICO_ERROR_GENERIC) {
        printf("I2C read error: PICO_ERROR_GENERIC\n");
    }
    if (count == PICO_ERROR_TIMEOUT) {
        printf("I2C read error: PICO_ERROR_TIMEOUT\n");
    }
    if (count != sizeof(raw_data)) {
        printf("I2C read error: Incomplete data read\n");
        return;
    }
    interpret_raw_data(raw_data);
}

int8_t MPU6050::interpret_raw_data(uint8_t* raw_data) {
    // Convert raw accelerometer data to g's
    imu_data.accel_x = (static_cast<int16_t>((raw_data[0] << 8) | raw_data[1])) / 16384.0f;
    imu_data.accel_y = (static_cast<int16_t>((raw_data[2] << 8) | raw_data[3])) / 16384.0f;
    imu_data.accel_z = (static_cast<int16_t>((raw_data[4] << 8) | raw_data[5])) / 16384.0f;

    // Convert raw gyroscope data to degrees/sec
    imu_data.gyro_x = (static_cast<int16_t>((raw_data[8] << 8) | raw_data[9])) / 131.0f;
    imu_data.gyro_y = (static_cast<int16_t>((raw_data[10] << 8) | raw_data[11])) / 131.0f;
    imu_data.gyro_z = (static_cast<int16_t>((raw_data[12] << 8) | raw_data[13])) / 131.0f;

    // Set the timestamp
    imu_data.timestamp_us = static_cast<uint64_t>(to_us_since_boot(get_absolute_time()));
    return 0; // Success
}

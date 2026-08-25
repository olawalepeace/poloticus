#ifndef POLOTICUS_MPU6050_H
#define POLOTICUS_MPU6050_H

#define MPU6050_ADDRESS 0x68
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_PWR_MGMT_1 0x6B

#include <cstdint>
#include <cstddef>
#include "I2cDevice.h"

struct ImuData {
    float accel_x;
    float accel_y;
    float accel_z;

    float gyro_x;
    float gyro_y;
    float gyro_z;

    uint64_t timestamp_us;
};

class MPU6050 {
public:
    MPU6050(ImuData& data);
    ~MPU6050();

    void initialize(uint8_t device_address, uint8_t sda, uint8_t scl, uint32_t timeout=1000000);
    void read_imu_data();

private:
    uint64_t read_timeout_ms;
    ImuData& imu_data;
    uint8_t device_address;
    int8_t interpret_raw_data(uint8_t* raw_data);
};

#endif // POLOTICUS_MPU6050_H

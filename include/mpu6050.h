#ifndef POLOTICUS_MPU6050_H
#define POLOTICUS_MPU6050_H

#define MPU6050_ADDRESS 0x68 //Mostly this value(or 0X69), but I will not pass it directly, pass appropriate during initialization
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_PWR_MGMT_1 0x6B

#include <cstdint>
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
    void readImuData();

private:
    uint64_t read_timeout_ms_;
    ImuData& imu_data_;
    uint8_t device_address_;
    int8_t interpretRawData(uint8_t* raw_data);
};

#endif // POLOTICUS_MPU6050_H

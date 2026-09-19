#ifndef POLOTICUS_MPU6050_H
#define POLOTICUS_MPU6050_H

#define MPU6050_ADDRESS 0x68 //Mostly this value(or 0X69), but I will not pass it directly, pass appropriate during initialization
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_PWR_MGMT_1 0x6B

#include <cstdint>
#include "I2cDevice.h"

constexpr double G_TO_MPS2 = 9.81;
constexpr float DEG_TO_RAD = 1.7453292e-2f;

struct GyroData {
    float gyro_x{0.0f};
    float gyro_y{0.0f};
    float gyro_z{0.0f};
    
    GyroData& operator+=(const GyroData& other);
    GyroData& operator/=(const int divisor);
};

struct AccelData {
    float accel_x{0.0f};
    float accel_y{0.0f};
    float accel_z{0.0f};
    
    AccelData& operator+=(const AccelData& other);
    AccelData& operator/=(const int divisor);
};

struct IMUData {
    uint64_t timestamp_us{0};

    AccelData accel_data{};
    GyroData gyro_data{};
};


class MPU6050 {
public:
    MPU6050(IMUData& data);
    ~MPU6050();

    void initialize(uint8_t device_address, uint8_t sda, uint8_t scl, uint32_t timeout=1e6);

    IMUData readImuData(IMUData* data = nullptr);
    GyroData readGyroData(uint8_t* raw_data = nullptr);
    AccelData readAccelData(uint8_t* raw_data = nullptr);

    void calibrateImu();
    void calibrateGyro();
    void calibrateAccel();

private:
    uint64_t read_timeout_ms_;
    IMUData& imu_data_;
    AccelData accel_offset_;
    GyroData gyro_offset_;
    uint8_t device_address_;
};

#endif // POLOTICUS_MPU6050_H

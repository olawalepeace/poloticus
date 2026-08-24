#ifndef POLOTICUS_MPU6050_H
#define POLOTICUS_MPU6050_H

#include <cstdint>
#include <cstddef>

namespace poloticus {

struct ImuData {
    float accel_x;
    float accel_y;
    float accel_z;

    float gyro_x;
    float gyro_y;
    float gyro_z;

    uint64_t timestamp_us;
};

class Imu {
public:
    Imu();
    ~Imu();

    void initialize();
    void read(ImuData& data);

private:
    uint8_t read_register(uint8_t reg, uint8_t* data, size_t len) const;
};

}

#endif // POLOTICUS_MPU6050_H
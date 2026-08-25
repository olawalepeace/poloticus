#include <stdio.h>
#include "pico/stdlib.h"
#include "mpu6050.h"


int main()
{
    stdio_init_all();
    ImuData data;
    MPU6050 imu(data);
    imu.initialize(0x68, 4, 5, 1000); // Initialize with default address and I2C pins

    while (true) {
        imu.read_imu_data();
        printf("Accel: X=%.2f, Y=%.2f, Z=%.2f | Gyro: X=%.2f, Y=%.2f, Z=%.2f | Timestamp: %llu us\n",
               data.accel_x, data.accel_y, data.accel_z,
               data.gyro_x, data.gyro_y, data.gyro_z,
               data.timestamp_us);
        sleep_ms(1000); // Sleep for 1 second
    }
}

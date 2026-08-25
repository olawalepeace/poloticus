#ifndef I2C_DEVICE_H
#define I2C_DEVICE_H    

#define DEFAULT_I2C_TIMEOUT_MS 1000

#include <cstdint>

class I2cDevice {
public:
    static void initialize(int8_t sda_pin = 4, int8_t scl_pin = 5, uint32_t baudrate = 400000);
    static int8_t writeBytes(uint8_t device_address, uint8_t register_address, uint8_t* data, uint8_t length, uint32_t timeout_ms= DEFAULT_I2C_TIMEOUT_MS);
    static int8_t readBytes(uint8_t device_address, uint8_t register_address, uint8_t* buffer, uint8_t length, uint32_t timeout_ms= DEFAULT_I2C_TIMEOUT_MS);
};
#endif // I2C_DEVICE_H

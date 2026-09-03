#include "I2cDevice.h"

#include "hardware/gpio.h"
#include "hardware/i2c.h"


/**
 * Initializes the default I2C peripheral and assigns the chosen SDA and SCL pins to the I2C function.
 *
 * @param sda_pin The GPIO pin used for the I2C data line.
 * @param scl_pin The GPIO pin used for the I2C clock line.
 * @param baudrate The bus speed in bits per second.
 */
void I2cDevice::initialize(int8_t sda_pin, int8_t scl_pin, uint32_t baudrate)
{
    i2c_init(i2c_default, baudrate);

    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);

    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
}

/**
 * Writes the target register address and then reads a block of bytes from the selected device.
 *
 * @param device_address The 7-bit I2C address of the slave device.
 * @param register_address The register to read from inside the slave device.
 * @param buffer A buffer that receives the bytes.
 * @param length The number of bytes to read.
 * @param timeout_ms The maximum wait time for the transaction in milliseconds.
 * @return The number of bytes read on success, or a negative value indicating fails.
 */
int8_t I2cDevice::readBytes(uint8_t device_address, uint8_t register_address, uint8_t* buffer, uint8_t length, uint32_t timeout_ms) {
    // Write the register address to the device
    int response = i2c_write_timeout_us(i2c_default, device_address, &register_address, 1, true, timeout_ms*1000);
    if (response < 0) {
        return response; // Return error code if write fails
    }

    // Read the data from the device
    int count = i2c_read_timeout_us(i2c_default, device_address, buffer, length, false, timeout_ms*1000);
    return count; // Return the number of bytes read or error code
}

/**
 * Writes a register address and a payload to the selected I2C device in one transaction.
 *
 * @param device_address The I2C address of the target device.
 * @param register_address The internal register to write to.
 * @param data A pointer to the bytes to transmit.
 * @param length The number of payload bytes.
 * @param timeout_ms The maximum wait time for the transaction in milliseconds.
 * @return The number of bytes written on success, or a negative value indicating fails.
 */
int8_t I2cDevice::writeBytes(uint8_t device_address, uint8_t register_address, uint8_t* data, uint8_t length, uint32_t timeout_ms) {
    uint8_t write_buffer[length+1] = {register_address};
    for(uint8_t i = 0; i < length; ++i) {
        write_buffer[i + 1] = data[i]; // Copy each byte from the input data array
    }
    int response = i2c_write_timeout_us(i2c_default, device_address, write_buffer, length + 1, false, timeout_ms*1000);
    return response; // Return the number of bytes written or error code
}

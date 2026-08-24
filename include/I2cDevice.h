#pragma once
#include <cstdint>
#include <cstddef>

class I2c
{
    public:
        static bool initialize();

        static bool write(uint8_t address, const uint8_t *data, size_t length);

        static bool read(uint8_t address, uint8_t *data, size_t length);

        static bool write_read(uint8_t address, const uint8_t *tx_data, size_t tx_length, uint8_t *rx_data, size_t rx_length);
};

#ifndef I2C_FUNCTIONS_H
#define I2C_FUNCTIONS_H

#include "driver/i2c_master.h"
#include "esp_err.h"

esp_err_t i2c_master_init(i2c_master_bus_handle_t *bus_handle, int sda_io, int scl_io, uint32_t freq_hz);
esp_err_t i2c_scanner(i2c_master_bus_handle_t bus_handle);
esp_err_t i2c_bus_recover(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle, uint8_t device_address, int sda_io, int scl_io, uint32_t freq_hz);

#endif // I2C_FUNCTIONS_H

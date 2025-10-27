#ifndef MPU6050_FUNCTIONS_H
#define MPU6050_FUNCTIONS_H

#include "esp_err.h"
#include "driver/i2c_master.h"

typedef struct {
    int16_t ax, ay, az;    // Données brutes (16-bit)
    int16_t gx, gy, gz;
} mpu6050_raw_t;

typedef struct {
    float ax, ay, az;     // Données converties (float)
    float gx, gy, gz;
} mpu6050_converted_t;


esp_err_t mpu6050_init(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t *dev_handle, uint8_t addr);
esp_err_t mpu6050_wake_and_configure(i2c_master_dev_handle_t dev_handle, i2c_master_bus_handle_t bus_handle);
esp_err_t mpu6050_read_data(i2c_master_dev_handle_t dev_handle, i2c_master_bus_handle_t bus_handle, mpu6050_raw_t *data);
esp_err_t convert_mpu6050_data(const mpu6050_raw_t *raw, mpu6050_converted_t *converted);

#endif // MPU6050_FUNCTIONS_H

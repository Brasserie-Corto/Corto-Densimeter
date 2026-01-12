#ifndef DENSIMETER_FUNCTIONS_H
#define DENSIMETER_FUNCTIONS_H

#include "esp_err.h"
#include "Hardware/MPU6050/mpu6050_functions.h"

esp_err_t calculate_inclination(mpu6050_raw_t *raw_data, float *angle_x, float *angle_y);
float estimate_density(float angle_x, float angle_y);
float estimate_alcohol_percentage(float initial_density, float final_density);

float measure_immersion_depth(mpu6050_raw_t *raw_data);
float estimate_density_from_depth(float depth);
float estimate_alcohol_percentage_from_depth(float initial_density, float final_density);

#endif // DENSIMETER_FUNCTIONS_H

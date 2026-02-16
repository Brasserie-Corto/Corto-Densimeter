#ifndef DENSIMETER_FUNCTIONS_H
#define DENSIMETER_FUNCTIONS_H

#include "esp_err.h"
#include "Hardware/MPU6050/mpu6050_functions.h"

// Mesure principale : enfoncement via axe Z
float get_az_value(mpu6050_raw_t *raw_data);
float calculate_immersion_delta(float az_initial, float az_current);
float estimate_density_from_az(float az);
float estimate_density_from_immersion(float immersion_delta);
float calculate_alcohol_percentage(float initial_density, float final_density);

// Fonctions d'inclinaison conservées pour référence (non utilisées)
esp_err_t calculate_inclination(mpu6050_raw_t *raw_data, float *angle_x, float *angle_y);
float estimate_density_from_angle(float angle_x, float angle_y);

#endif // DENSIMETER_FUNCTIONS_H

#include "mpu6050_functions.h"
#include "esp_log.h"
#include "i2c_functions.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MPU6050";

/**
 * @brief Initialise le MPU6050 sur le bus I2C.
 */
esp_err_t mpu6050_init(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t *dev_handle, uint8_t addr) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,                                                          // Adresse I2C sur 7 bits
        .device_address = addr,                                                                         // Adresse I2C du MPU6050
        .scl_speed_hz = 100000,                                                                         // Vitesse SCL (Clock à 100kHz)
    };
    return i2c_master_bus_add_device(bus_handle, &dev_cfg, dev_handle);                                 // Ajouter le périphérique MPU6050 au bus I2C
}

/**
 * @brief Réveille le MPU6050 et le configure.
 */
esp_err_t mpu6050_wake_and_configure(i2c_master_dev_handle_t dev_handle, i2c_master_bus_handle_t bus_handle) {
    // 1. Réveiller le MPU6050 (PWR_MGMT_1)
    uint8_t pwr_mgmt_1_reg = 0x6B;
    uint8_t pwr_mgmt_1_data = 0x00;
    esp_err_t ret = i2c_master_transmit(dev_handle, &pwr_mgmt_1_reg, 1, pdMS_TO_TICKS(1000));           // Envoyer l'adresse du registre PWR_MGMT_1
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send register address for PWR_MGMT_1: %s", esp_err_to_name(ret));
        i2c_bus_recover(&bus_handle, &dev_handle, 0x68, 1, 2, 100000);
        return ret;
    }
    ret = i2c_master_transmit(dev_handle, &pwr_mgmt_1_data, 1, pdMS_TO_TICKS(1000));                    // Écrire 0x00 pour sortir du mode veille
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU6050: %s", esp_err_to_name(ret));
        i2c_bus_recover(&bus_handle, &dev_handle, 0x68, 1, 2, 100000);
        return ret;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // 2. Configurer l'accéléromètre (ACCEL_CONFIG)
    uint8_t accel_config_reg = 0x1C;
    uint8_t accel_config_data = 0x00;  // ±2g
    ret = i2c_master_transmit(dev_handle, &accel_config_reg, 1, pdMS_TO_TICKS(1000));                   // Envoyer l'adresse du registre ACCEL_CONFIG
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send register address for ACCEL_CONFIG: %s", esp_err_to_name(ret));
        i2c_bus_recover(&bus_handle, &dev_handle, 0x68, 1, 2, 100000);
        return ret;
    }
    ret = i2c_master_transmit(dev_handle, &accel_config_data, 1, pdMS_TO_TICKS(1000));                  // Écrire 0x00 pour configurer l'accéléromètre
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure accelerometer: %s", esp_err_to_name(ret));
        i2c_bus_recover(&bus_handle, &dev_handle, 0x68, 1, 2, 100000);
        return ret;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // 3. Configurer le gyroscope (GYRO_CONFIG)
    uint8_t gyro_config_reg = 0x1B;
    uint8_t gyro_config_data = 0x00;  // ±250°/s
    ret = i2c_master_transmit(dev_handle, &gyro_config_reg, 1, pdMS_TO_TICKS(1000));                    // Envoyer l'adresse du registre GYRO_CONFIG
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send register address for GYRO_CONFIG: %s", esp_err_to_name(ret));
        i2c_bus_recover(&bus_handle, &dev_handle, 0x68, 1, 2, 100000);
        return ret;
    }
    ret = i2c_master_transmit(dev_handle, &gyro_config_data, 1, pdMS_TO_TICKS(1000));                   // Écrire 0x00 pour configurer le gyroscope
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure gyroscope: %s", esp_err_to_name(ret));
        i2c_bus_recover(&bus_handle, &dev_handle, 0x68, 1, 2, 100000);
        return ret;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);

    return ESP_OK;
}

/**
 * @brief Lit les données brutes du MPU6050 (accélération et gyroscope).
 */
esp_err_t mpu6050_read_data(i2c_master_dev_handle_t dev_handle, i2c_master_bus_handle_t bus_handle, mpu6050_data_t *data) {
    uint8_t reg_addr = 0x3B;
    uint8_t buffer[14];
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg_addr, 1, buffer, 14, pdMS_TO_TICKS(1000));      // Lire 14 octets à partir du registre 0x3B
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read I2C data: %s", esp_err_to_name(ret));
        i2c_bus_recover(&bus_handle, &dev_handle, 0x68, 1, 2, 100000);                                           // En cas d'erreur, tenter de récupérer le bus I2C en réinitialisant le périphérique
        return ret;
    }
    /*
     * Conversion des données brutes du MPU6050 :
     *
     * Le MPU6050 stocke les valeurs d'accélération et de vitesse angulaire
     * sous forme de paires d'octets (16 bits) en complément à deux (signed 16-bit).
     * Chaque valeur est répartie sur deux octets consécutifs dans le buffer 'data' :
     * - Le premier octet (ex: data[0] pour AX) est le byte le plus significatif (MSB).
     * - Le deuxième octet (ex: data[1] pour AX) est le byte le moins significatif (LSB).
     *
     * Pour reconstituer la valeur 16 bits :
     * 1. On décale le MSB de 8 bits vers la gauche (<< 8) pour le placer en position haute.
     * 2. On combine avec le LSB via un OU binaire (|), ce qui donne la valeur 16 bits brute.
     *
     * Exemple pour l'accélération X (AX) :
     *   - data[0] = MSB (ex: 0x07)
     *   - data[1] = LSB (ex: 0xFF)
     *   - ax = (0x07 << 8) | 0xFF = 0x07FF = 2047 en décimal
     *
     * Les valeurs sont en complément à deux, donc :
     *   - Si le bit 15 (MSB du MSB) est à 1, la valeur est négative.
     *   - Sinon, la valeur est positive.
     *
     * Plage de valeurs possibles :
     *   - Accélération : ±16g (par défaut), donc ±32768 LSB/g (selon la sensibilité configurée).
     *   - Gyroscope : ±250°/s (par défaut), donc ±32768 LSB/(250°/s) (selon la sensibilité configurée).
     *
     * Index des octets dans le buffer 'data' :
     *   - Accélération : X (0-1), Y (2-3), Z (4-5)
     *   - Température : (6-7) [non utilisée ici]
     *   - Gyroscope : X (8-9), Y (10-11), Z (12-13)
     */
    data->ax = (buffer[0] << 8) | buffer[1];                                // Accélération X
    data->ay = (buffer[2] << 8) | buffer[3];                                // Accélération Y
    data->az = (buffer[4] << 8) | buffer[5];                                // Accélération Z
    data->gx = (buffer[8] << 8) | buffer[9];                                // Vitesse angulaire X
    data->gy = (buffer[10] << 8) | buffer[11];                              // Vitesse angulaire Y
    data->gz = (buffer[12] << 8) | buffer[13];                              // Vitesse angulaire Z
    return ESP_OK;
}

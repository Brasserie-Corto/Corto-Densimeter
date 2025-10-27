#include "i2c_functions.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "I2C";

/**
 * @brief Initialise le bus I2C en mode maître.
 */
esp_err_t i2c_master_init(i2c_master_bus_handle_t *bus_handle, int sda_io, int scl_io, uint32_t freq_hz) {
    i2c_master_bus_config_t i2c_bus_config = {                                                                  // Configuration du bus I2C
        .sda_io_num = sda_io,                                                                                   // Broche SDA
        .scl_io_num = scl_io,                                                                                   // Broche SCL
        .clk_source = I2C_CLK_SRC_DEFAULT,                                                                      // Source d'horloge par défaut
        .i2c_port = I2C_NUM_0,                                                                                  // Utiliser le port I2C 0
        .glitch_ignore_cnt = 7,                                                                                 // Filtrage des glitches 
        .flags.enable_internal_pullup = true,                                                                   // Activer les pull-ups internes
    };
    return i2c_new_master_bus(&i2c_bus_config, bus_handle);                                                     // Créer le bus I2C maître
}

/**
 * @brief Scanne le bus I2C pour détecter les périphériques connectés.
 */
esp_err_t i2c_scanner(i2c_master_bus_handle_t bus_handle) {
    ESP_LOGI(TAG, "Scanning I2C bus...");
    for (uint8_t address = 1; address < 127; address++) {                                                       // Parcourir les adresses I2C valides
        i2c_device_config_t dev_cfg = {                                                                         // Configuration du périphérique I2C
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,                                                              // Adresse sur 7 bits
            .device_address = address,                                                                          // Adresse du périphérique
            .scl_speed_hz = 100000,                                                                             // Vitesse SCL à 100kHz
        };
        i2c_master_dev_handle_t dev_handle_scan;                                                                // Handle pour le périphérique détecté
        if (i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle_scan) == ESP_OK) {                      // Tenter d'ajouter le périphérique
            ESP_LOGI(TAG, "Device found at address: 0x%02X", address);                                          // Afficher l'adresse du périphérique trouvé
            i2c_master_bus_rm_device(dev_handle_scan);                                                          // Supprimer le périphérique du bus
        }
    }
    return ESP_OK;
}

/**
 * @brief Réinitialise le bus I2C en cas d'erreur.
 */
esp_err_t i2c_bus_recover(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle, uint8_t device_address, int sda_io, int scl_io, uint32_t freq_hz) {
    // Supprimer le périphérique et le bus existants
    if (*dev_handle != NULL) {
        i2c_master_bus_rm_device(*dev_handle);
        *dev_handle = NULL;
    }
    if (*bus_handle != NULL) {
        i2c_del_master_bus(*bus_handle);
        *bus_handle = NULL;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Réinitialiser le bus I2C en réutilisant i2c_master_init
    esp_err_t ret = i2c_master_init(bus_handle, sda_io, scl_io, freq_hz);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reinitialize I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // Réajouter le périphérique
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_address,
        .scl_speed_hz = freq_hz,
    };
    ret = i2c_master_bus_add_device(*bus_handle, &dev_cfg, dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reinitialize I2C device: %s", esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}


// Définition des macros pour inclure les en-têtes nécessaires
#include "i2c_functions.h"                                                              // Pour les fonctions I2C
#include "mpu6050_functions.h"                                                          // Pour les fonctions MPU6050
#include "led_functions.h"                                                              // Pour les fonctions LED
#include "densimeter_functions.h"                                                       // Pour les fonctions Densimeter
#include "freertos/FreeRTOS.h"                                                          // Pour les définitions FreeRTOS
#include "freertos/task.h"                                                              // Pour les fonctions FreeRTOS
#include "esp_log.h"                                                                    // Pour les fonctions de log
#include "driver/gpio.h"                                                                // Pour les fonctions GPIO

// Définition des broches pour les LEDs et boutons
#define I2C_MASTER_SCL_IO 2                                                             // GPIO 2 pour MPU6050-SCL
#define I2C_MASTER_SDA_IO 1                                                             // GPIO 1 pour MPU6050-SDA
#define LED_GREEN_GPIO 15                                                               // GPIO 15 pour la LED verte
#define LED_RED_GPIO 23                                                                 // GPIO 23 pour la LED rouge
#define BUTTON_GPIO 3                                                                   // GPIO 3 pour le bouton de tarage

static const char *TAG = "MAIN";                                                        // Tag pour les logs

// Constantes pour l'étalonnage (à ajuster selon tes mesures expérimentales)
#define DENSITY_AIR 0.001225f  // Densité de l'air (kg/m³) - pour référence
#define DENSITY_WATER 1.0f     // Densité de l'eau (kg/m³) - pour étalonnage
#define INITIAL_DENSITY 1.050f // Densité initiale du moût (kg/m³)

/**
 * @brief Gère les erreurs en clignotant la LED rouge et en affichant un message.
 */
void handle_error(esp_err_t err, const char *message) {
    ESP_LOGE(TAG, "%s: %s", message, esp_err_to_name(err));                             // Affiche le message d'erreur
    gpio_set_level(LED_RED_GPIO, 1);                                                    // Allume la LED rouge
    vTaskDelay(1000 / portTICK_PERIOD_MS);                                              // Attendre 1 seconde
    gpio_set_level(LED_RED_GPIO, 0);                                                    // Éteindre la LED rouge
}

/**
 * @brief Point d'entrée principal du programme.
 * Initialise le bus I2C, les LEDs, et lit en boucle les données du MPU6050.
 */
void app_main() {
    // Initialisation des handles : ressources abstraites pour I2C et MPU6050
    i2c_master_bus_handle_t i2c_bus = NULL;
    i2c_master_dev_handle_t dev_handle = NULL;

    mpu6050_raw_t raw_data;
    mpu6050_converted_t converted_data;

    // Initialisation des LEDs
    esp_err_t ret = leds_init(LED_GREEN_GPIO, LED_RED_GPIO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erreur d'initialisation des LEDs");
        return;
    }
    gpio_set_level(LED_GREEN_GPIO, 1);                                                  // Allumer la LED verte pendant l'initialisation

    // Initialisation du bus I2C
    ret = i2c_master_init(&i2c_bus, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, 100000);      // 100kHz
    if (ret != ESP_OK) {
        handle_error(ret, "Erreur d'initialisation du bus I2C");
        return;
    }

    // Initialisation du MPU6050
    ret = mpu6050_init(i2c_bus, &dev_handle, 0x68);
    if (ret != ESP_OK) {
        handle_error(ret, "Erreur d'initialisation du MPU6050");
        return;
    }

    // Réveil et configuration du MPU6050
    ret = mpu6050_wake_and_configure(dev_handle, i2c_bus);
    if (ret != ESP_OK) {
        handle_error(ret, "Erreur de configuration du MPU6050");
        return;
    }

    // Éteindre la LED verte après l'initialisation
    gpio_set_level(LED_GREEN_GPIO, 0);

    // Boucle principale
    while (1) {
        gpio_set_level(LED_GREEN_GPIO, 1);
        ret = mpu6050_read_data(dev_handle, i2c_bus, &raw_data);
        if (ret == ESP_OK) {
            // Mesurer la profondeur d'immersion
            float depth = measure_immersion_depth(&raw_data);

            // Estimer la densité à partir de la profondeur
            float current_density = estimate_density_from_depth(depth);

            // Estimer le degré d'alcool
            float alcohol_percentage = estimate_alcohol_percentage_from_depth(INITIAL_DENSITY, current_density);

            // Afficher les résultats
            ESP_LOGI(TAG, "Profondeur d'immersion: %.2f mm", depth);
            ESP_LOGI(TAG, "Densité estimée: %.3f kg/m³", current_density);
            ESP_LOGI(TAG, "Degré d'alcool estimé: %.2f %%", alcohol_percentage);

            // Afficher les données brutes du gyroscope (optionnel)
            ESP_LOGI(TAG, "Raw Gyro: X=%d, Y=%d, Z=%d", raw_data.gx, raw_data.gy, raw_data.gz);
            ret = convert_mpu6050_data(&raw_data, &converted_data);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Gyro (°/s): X=%.2f, Y=%.2f, Z=%.2f", converted_data.gx, converted_data.gy, converted_data.gz);
            } else {
                ESP_LOGE(TAG, "Erreur de conversion: %s", esp_err_to_name(ret));
            }
        } else {
            handle_error(ret, "Erreur de lecture du MPU6050");
        }
        gpio_set_level(LED_GREEN_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

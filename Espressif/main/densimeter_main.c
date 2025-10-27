// Définition des macros pour inclure les en-têtes nécessaires
#include "i2c_functions.h"                                                              // Pour les fonctions I2C
#include "mpu6050_functions.h"                                                          // Pour les fonctions MPU6050
#include "led_functions.h"                                                              // Pour les fonctions LED
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
    mpu6050_data_t mpu_data;

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
        // Allumer la LED verte pendant la lecture
        gpio_set_level(LED_GREEN_GPIO, 1);

        // Lire les données du MPU6050
        ret = mpu6050_read_data(dev_handle, i2c_bus, &mpu_data);                                
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Accel: X=%d, Y=%d, Z=%d", mpu_data.ax, mpu_data.ay, mpu_data.az);
            ESP_LOGI(TAG, "Gyro: X=%d, Y=%d, Z=%d", mpu_data.gx, mpu_data.gy, mpu_data.gz);
        } else {
            handle_error(ret, "Erreur de lecture des données du MPU6050");
        }

        // Éteindre la LED verte après la lecture
        gpio_set_level(LED_GREEN_GPIO, 0);                                                          

        // Attendre 1 seconde avant la prochaine lecture
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

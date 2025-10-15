#include <stdio.h>                                                                      // Pour les fonctions standard d'entrée/sortie
#include "freertos/FreeRTOS.h"                                                          // Pour les définitions FreeRTOS
#include "freertos/task.h"                                                              // Pour les fonctions FreeRTOS
#include "driver/i2c_master.h"                                                          // Nouveau pilote I2C
#include "driver/gpio.h"                                                                // Pour les fonctions GPIO
#include "esp_log.h"                                                                    // Pour les fonctions de log
#include "esp_system.h"                                                                 // Pour esp_restart()  

// Définition des broches et paramètres I2C
#define I2C_MASTER_SCL_IO           2                                                   // GPIO 2 pour SCL
#define I2C_MASTER_SDA_IO           1                                                   // GPIO 1 pour SDA
#define I2C_MASTER_FREQ_HZ          100000                                              // Fréquence I2C (100 kHz)
#define I2C_MASTER_TIMEOUT_MS       1000                                                // Timeout I2C en ms

// Définition des broches pour les LEDs et boutons
#define LED_GREEN_GPIO   15                                                             // GPIO 15 pour la LED verte
#define LED_RED_GPIO     23                                                             // GPIO 23 pour la LED rouge
#define BUTTON_GPIO      3                                                              // GPIO 3 pour le bouton de tarage

// Définition des paramètres spécifiques au MPU6050
#define MPU6050_ADDR                0x68                                                // Adresse I2C du MPU6050 (peut être 0x69 si AD0 est à HIGH)
#define MPU6050_ACCEL_XOUT_H        0x3B                                                // Registre de départ pour les données d'accélération et de gyroscope
static const char *TAG = "MPU6050";                                                     // Tag pour les logs

// Handles pour le bus et le périphérique I2C
i2c_master_bus_handle_t i2c_bus;                                                        // Handle du bus I2C   
i2c_master_dev_handle_t dev_handle;                                                     // Handle du périphérique I2C 

/**
 * @brief Initialise le bus I2C en mode maître.
 * Cette fonction configure les broches SDA et SCL, active les résistances de pull-up,
 * et installe le pilote I2C avec la fréquence spécifiée.
 */
void i2c_master_init() {
    i2c_master_bus_config_t i2c_mst_config = {                                          // Configuration du bus I2C
        .clk_source = I2C_CLK_SRC_DEFAULT,                                              // Source d'horloge par défaut
        .i2c_port = I2C_NUM_0,                                                          // Utiliser le port I2C 0
        .scl_io_num = I2C_MASTER_SCL_IO,                                                // Broche SCL
        .sda_io_num = I2C_MASTER_SDA_IO,                                                // Broche SDA
        .glitch_ignore_cnt = 7,                                                         // Ignorer les glitches
        .flags.enable_internal_pullup = true,                                           // Activer les résistances de pull-up internes
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &i2c_bus));                     // Créer le bus I2C maître

    i2c_device_config_t dev_cfg = {                                                     // Configuration du périphérique I2C
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,                                          // Adresse sur 7 bits
        .device_address = MPU6050_ADDR,                                                 // Adresse du MPU6050
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,                                             // Vitesse du bus I2C
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_cfg, &dev_handle));         // Ajouter le périphérique au bus I2C
}

/**
 * @brief Scanne le bus I2C pour détecter les périphériques connectés.
 */
void i2c_scanner() {
    printf("\nScanning I2C bus...\n");                                                  // Message de début de scan
    for (uint8_t address = 1; address < 127; address++) {                               // Parcourir toutes les adresses I2C possibles
        i2c_device_config_t dev_cfg = {                                                 // Configuration du périphérique I2C pour le scan
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,                                      // Adresse sur 7 bits
            .device_address = address,                                                  // Adresse courante à tester
            .scl_speed_hz = 100000,                                                     // Vitesse du bus I2C
        };
        i2c_master_dev_handle_t dev_handle_scan;                                        // Handle pour le périphérique scanné

        if (i2c_master_bus_add_device(i2c_bus, &dev_cfg, &dev_handle_scan) == ESP_OK) { // Tenter d'ajouter le périphérique au bus I2C
            printf("Device found at address: 0x%02X\n", address);                       // Si succès, afficher l'adresse du périphérique trouvé
            i2c_master_bus_rm_device(dev_handle_scan);                                  // Retirer le périphérique du bus après le test
        }
    }
}

/**
 * @brief Réinitialise le bus I2C en cas d'erreur.
 * Cette fonction désinstalle le périphérique et le bus I2C,
 * puis les réinstalle pour tenter de récupérer la communication.
 */
void i2c_bus_recover() {
    i2c_master_bus_rm_device(dev_handle);                                               // Retirer le périphérique du bus
    i2c_del_master_bus(i2c_bus);                                                        // Désinstaller le bus I2C
    vTaskDelay(100 / portTICK_PERIOD_MS);                                               // Attendre 100 ms avant de réinitialiser

    i2c_master_bus_config_t i2c_mst_config = {                                          // Reconfiguration du bus I2C
        .clk_source = I2C_CLK_SRC_DEFAULT,                                              // Source d'horloge par défaut
        .i2c_port = I2C_NUM_0,                                                          // Utiliser le port I2C 0
        .scl_io_num = I2C_MASTER_SCL_IO,                                                // Broche SCL
        .sda_io_num = I2C_MASTER_SDA_IO,                                                // Broche SDA
        .glitch_ignore_cnt = 7,                                                         // Ignorer les glitches
        .flags.enable_internal_pullup = true,                                           // Activer les résistances de pull-up internes  
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &i2c_bus));                     // Réinstaller le bus I2C maître

    i2c_device_config_t dev_cfg = {                                                     // Reconfiguration du périphérique I2C   
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,                                          // Adresse sur 7 bits
        .device_address = MPU6050_ADDR,                                                 // Adresse du MPU6050
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,                                             // Vitesse du bus I2C
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_cfg, &dev_handle));         // Réinstaller le périphérique au bus I2C
}

/**
 * @brief Réveille le MPU6050 en désactivant le mode veille.
 */
void mpu6050_wake_up() {                    
    uint8_t data = 0;                                                                   // Donnée pour désactiver le mode veille
    uint8_t reg_addr = 0x6B;                                                            // Adresse du registre PWR_MGMT_1
    esp_err_t ret = i2c_master_transmit(dev_handle, &reg_addr, 1, pdMS_TO_TICKS(1000)); // Envoyer l'adresse du registre
    if (ret != ESP_OK) {                                                                // Vérifier les erreurs de communication
        ESP_LOGE(TAG, "Failed to send register address: %s", esp_err_to_name(ret));     // En cas d'erreur, afficher le message et tenter de récupérer le bus I2C
        i2c_bus_recover();                                                              // Tenter de récupérer le bus I2C
        return;                                                                         // Sortir de la fonction en cas d'erreur
    }
    ret = i2c_master_transmit(dev_handle, &data, 1, pdMS_TO_TICKS(1000));               // Envoyer la donnée pour désactiver le mode veille
    if (ret != ESP_OK) {                                                                // Vérifier les erreurs de communication
        ESP_LOGE(TAG, "Failed to wake up MPU6050: %s", esp_err_to_name(ret));           // En cas d'erreur, afficher le message et tenter de récupérer le bus I2C
        i2c_bus_recover();                                                              // Tenter de récupérer le bus I2C
        return;                                                                         // Sortir de la fonction en cas d'erreur
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);                                               // Attendre 100 ms pour que le MPU6050 se stabilise
}

/**
 * @brief Lit les données brutes du MPU6050 (accélération et gyroscope).
 * Cette fonction envoie une commande I2C pour lire 14 octets à partir du registre 0x3B,
 * puis convertit ces octets en valeurs 16 bits pour chaque axe (X, Y, Z) de l'accéléromètre et du gyroscope.
 * Les valeurs sont ensuite affichées dans la console.
 */
void mpu6050_read_data() {
    gpio_set_level(LED_GREEN_GPIO, 1);                                                  // Allumer la LED verte pour indiquer la lecture 

    uint8_t data[14];                                                                   // Buffer pour stocker les données lues
    uint8_t reg_addr = MPU6050_ACCEL_XOUT_H;                                            // Adresse du registre de départ pour la lecture

    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, 14, pdMS_TO_TICKS(1000));   // Lire 14 octets à partir du registre 0x3B
    if (ret != ESP_OK) {                                                                // Vérifier les erreurs de communication
        ESP_LOGE(TAG, "Failed to read I2C data: %s", esp_err_to_name(ret));             // En cas d'erreur, afficher le message et tenter de récupérer le bus I2C         
        gpio_set_level(LED_GREEN_GPIO, 0);                                              // Éteindre la LED verte en cas d'erreur
        i2c_bus_recover();                                                              // Tenter de récupérer le bus I2C
        return;                                                                         // Sortir de la fonction en cas d'erreur
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
    int16_t ax = (data[0] << 8) | data[1];                                              // Accélération X : combine MSB (data[0]) et LSB (data[1])
    int16_t ay = (data[2] << 8) | data[3];                                              // Accélération Y : combine MSB (data[2]) et LSB (data[3])
    int16_t az = (data[4] << 8) | data[5];                                              // Accélération Z : combine MSB (data[4]) et LSB (data[5])
    int16_t gx = (data[8] << 8) | data[9];                                              // Gyroscope X : combine MSB (data[8]) et LSB (data[9])
    int16_t gy = (data[10] << 8) | data[11];                                            // Gyroscope Y : combine MSB (data[10]) et LSB (data[11])
    int16_t gz = (data[12] << 8) | data[13];                                            // Gyroscope Z : combine MSB (data[12]) et LSB (data[13])

    // Affichage des valeurs dans la console
    ESP_LOGI(TAG, "Accel: X=%d, Y=%d, Z=%d", ax, ay, az);                               // Afficher les valeurs d'accélération
    ESP_LOGI(TAG, "Gyro: X=%d, Y=%d, Z=%d", gx, gy, gz);                                // Afficher les valeurs de vitesse angulaire

    gpio_set_level(LED_GREEN_GPIO, 0);                                                  // Éteindre la LED verte après la lecture
}

/**
 * @brief Configure le MPU6050 pour une utilisation optimale.
 */
void mpu6050_configure() {
    uint8_t pwr_mgmt_1_reg = 0x6B;                                                      // Adresse du registre PWR_MGMT_1
    uint8_t pwr_mgmt_1_data = 0x00;                                                     // Désactiver le mode veille

    esp_err_t ret = i2c_master_transmit(dev_handle, &pwr_mgmt_1_reg, 1, pdMS_TO_TICKS(1000));       // Envoyer l'adresse du registre PWR_MGMT_1
    if (ret != ESP_OK) {                                                                            // Vérifier les erreurs de communication
        ESP_LOGE(TAG, "Failed to send register address for PWR_MGMT_1: %s", esp_err_to_name(ret));  // En cas d'erreur, afficher le message et tenter de récupérer le bus I2C
        i2c_bus_recover();                                                                          // Tenter de récupérer le bus I2C
        return;                                                                                     // Sortir de la fonction en cas d'erreur
    }

    ret = i2c_master_transmit(dev_handle, &pwr_mgmt_1_data, 1, pdMS_TO_TICKS(1000));                // Envoyer la donnée pour désactiver le mode veille
    if (ret != ESP_OK) {                                                                            // Vérifier les erreurs de communication
        ESP_LOGE(TAG, "Failed to wake up MPU6050: %s", esp_err_to_name(ret));                       // En cas d'erreur, afficher le message et tenter de récupérer le bus I2C
        i2c_bus_recover();                                                                          // Tenter de récupérer le bus I2C
        return;                                                                                     // Sortir de la fonction en cas d'erreur
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);

    uint8_t accel_config_reg = 0x1C;                                                                // Adresse du registre ACCEL_CONFIG
    uint8_t accel_config_data = 0x00;  // ±2g                                                       // Configurer la sensibilité de l'accéléromètre (±2g)

    ret = i2c_master_transmit(dev_handle, &accel_config_reg, 1, pdMS_TO_TICKS(1000));                   // Envoyer l'adresse du registre ACCEL_CONFIG
    if (ret != ESP_OK) {                                                                                // Vérifier les erreurs de communication
        ESP_LOGE(TAG, "Failed to send register address for ACCEL_CONFIG: %s", esp_err_to_name(ret));    // En cas d'erreur, afficher le message et tenter de récupérer le bus I2C
        i2c_bus_recover();                                                                              // Tenter de récupérer le bus I2C
        return;                                                                                         // Sortir de la fonction en cas d'erreur
    }

    ret = i2c_master_transmit(dev_handle, &accel_config_data, 1, pdMS_TO_TICKS(1000));               // Envoyer la donnée pour configurer la sensibilité de l'accéléromètre
    if (ret != ESP_OK) {                                                                             // Vérifier les erreurs de communication
        ESP_LOGE(TAG, "Failed to configure accelerometer: %s", esp_err_to_name(ret));                // En cas d'erreur, afficher le message et tenter de récupérer le bus I2C
        i2c_bus_recover();                                                                           // Tenter de récupérer le bus I2C
        return;                                                                                      // Sortir de la fonction en cas d'erreur
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);                                                            // Attendre 100 ms pour que la configuration prenne effet

    uint8_t gyro_config_reg = 0x1B;                                                                  // Adresse du registre GYRO_CONFIG
    uint8_t gyro_config_data = 0x00;                                                                 // Configurer la sensibilité du gyroscope (±250°/s)

    ret = i2c_master_transmit(dev_handle, &gyro_config_reg, 1, pdMS_TO_TICKS(1000));                 // Envoyer l'adresse du registre GYRO_CONFIG
    if (ret != ESP_OK) {                                                                             // Vérifier les erreurs de communication
        ESP_LOGE(TAG, "Failed to send register address for GYRO_CONFIG: %s", esp_err_to_name(ret));  // En cas d'erreur, afficher le message et tenter de récupérer le bus I2C
        i2c_bus_recover();                                                                           // Tenter de récupérer le bus I2C
        return;                                                                                      // Sortir de la fonction en cas d'erreur
    }

    ret = i2c_master_transmit(dev_handle, &gyro_config_data, 1, pdMS_TO_TICKS(1000));                // Envoyer la donnée pour configurer la sensibilité du gyroscope  
    if (ret != ESP_OK) {                                                                             // Vérifier les erreurs de communication
        ESP_LOGE(TAG, "Failed to configure gyroscope: %s", esp_err_to_name(ret));                    // En cas d'erreur, afficher le message et tenter de récupérer le bus I2C
        i2c_bus_recover();                                                                           // Tenter de récupérer le bus I2C
        return;                                                                                      // Sortir de la fonction en cas d'erreur
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);                                                            // Attendre 100 ms pour que la configuration prenne effet 
}


/**
 * @brief Initialise les GPIOs pour les LEDs.
 * Configure les broches des LEDs en mode sortie et allume la LED rouge au démarrage.
 */
void leds_init() {
    gpio_config_t io_conf = {
        .pin_bit_mask = ((1ULL << LED_GREEN_GPIO) | (1ULL << LED_RED_GPIO)),            // Sélection des GPIOs
        .mode = GPIO_MODE_OUTPUT,                                                       // Mode sortie
        .pull_up_en = GPIO_PULLUP_DISABLE,                                              // Désactiver les résistances de pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,                                          // Désactiver les résistances de pull-down
        .intr_type = GPIO_INTR_DISABLE,                                                 // Désactiver les interruptions
    };
    gpio_config(&io_conf);                                                              // Appliquer la configuration
    for (int i = 0; i < 20; i++)                                                        // Boucle pour clignoter les LEDs 20 fois
    {
        gpio_set_level(LED_RED_GPIO, 0);                                                // Éteindre la LED rouge
        gpio_set_level(LED_GREEN_GPIO, 1);                                              // Allumer la LED verte
        vTaskDelay(50 / portTICK_PERIOD_MS);                                            // Délai de 500 ms
        gpio_set_level(LED_GREEN_GPIO, 0);                                              // Éteindre la LED verte
        gpio_set_level(LED_RED_GPIO, 1);                                                // Allumer la LED rouge
        vTaskDelay(50 / portTICK_PERIOD_MS);                                            // Délai de 500 ms
    }
    gpio_set_level(LED_RED_GPIO, 1);                                                    // Allumer la LED rouge au démarrage
    gpio_set_level(LED_GREEN_GPIO, 0);                                                  // Éteindre la LED verte
}

void button_init() {                                                                    // Initialise le GPIO pour le bouton de tarage
    gpio_config_t io_conf = {                                                           // Configuration du GPIO
        .pin_bit_mask = (1ULL << BUTTON_GPIO),                                          // Sélection du GPIO du bouton  
        .mode = GPIO_MODE_INPUT,                                                        // Mode entrée
        .pull_up_en = GPIO_PULLUP_DISABLE,                                              // Désactive le pull-up interne
        .pull_down_en = GPIO_PULLDOWN_ENABLE,                                           // Active le pull-down interne
        .intr_type = GPIO_INTR_DISABLE,                                                 // Désactive les interruptions
    };
    gpio_config(&io_conf);                                                              // Appliquer la configuration
}

void check_button() {                                                                   // Vérifie l'état du bouton
    if (gpio_get_level(BUTTON_GPIO) == 1) {                                             // Bouton pressé (HIGH)
        printf("Bouton pressé : reboot dans 1 seconde...\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);                                          // Délai pour éviter les rebonds
        if (gpio_get_level(BUTTON_GPIO) == 1) {                                         // Vérifie à nouveau
            esp_restart();                                                              // Reboot la board
        }
    }
}

/**
 * @brief Point d'entrée principal du programme.
 * Initialise le bus I2C, puis lit et affiche en boucle les données du MPU6050 toutes les secondes.
 */
void app_main() {
    leds_init();                                                                        // Initialiser les LEDs
    i2c_master_init();                                                                  // Initialiser l'I2C
    mpu6050_wake_up();
    i2c_scanner();                                                                      // Scanner le bus I2C pour détecter les périphériques
    while (1) {
        check_button();  // Vérifie l'état du bouton
        mpu6050_read_data();                                                            // Lire et afficher les données du MPU6050
        vTaskDelay(1000 / portTICK_PERIOD_MS);                                          // Attendre 1000 ms avant la prochaine lecture
    }
}

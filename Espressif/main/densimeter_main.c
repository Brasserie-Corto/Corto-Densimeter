#include <stdio.h>                                                                      // Pour les fonctions standard d'entrée/sortie
#include "freertos/FreeRTOS.h"                                                          // Pour les définitions FreeRTOS
#include "freertos/task.h"                                                              // Pour les fonctions FreeRTOS
#include "driver/i2c.h"                                                                 // Pour les fonctions I2C
#include "driver/gpio.h"                                                                // Pour les fonctions GPIO
#include "esp_log.h"                                                                    // Pour les fonctions de log

// Définition des broches et paramètres I2C
#define I2C_MASTER_SCL_IO           2                                                   // GPIO 2 pour SCL
#define I2C_MASTER_SDA_IO           1                                                   // GPIO 1 pour SDA
#define I2C_MASTER_FREQ_HZ          400000                                              // Fréquence I2C (400 kHz)
#define I2C_MASTER_TIMEOUT_MS       1000                                                // Timeout I2C en ms

// Définition des broches pour les LEDs et boutons
#define LED_GREEN_GPIO   15                                                             // GPIO 15 pour la LED verte
#define LED_RED_GPIO     23                                                             // GPIO 23 pour la LED rouge
#define BUTTON_GPIO      3                                                              // GPIO 3 pour le bouton de tarage

// Définition des paramètres spécifiques au MPU6050
#define MPU6050_ADDR                0x68                                                // Adresse I2C du MPU6050 (peut être 0x69 si AD0 est à HIGH)
#define MPU6050_ACCEL_XOUT_H        0x3B                                                // Registre de départ pour les données d'accélération et de gyroscope
static const char *TAG = "MPU6050";                                                     // Tag pour les logs

/**
 * @brief Initialise le bus I2C en mode maître.
 * Cette fonction configure les broches SDA et SCL, active les résistances de pull-up,
 * et installe le pilote I2C avec la fréquence spécifiée.
 */
void i2c_master_init() {
    i2c_config_t conf = {                                                               // Configuration I2C
        .mode = I2C_MODE_MASTER,                                                        // Mode maître
        .sda_io_num = I2C_MASTER_SDA_IO,                                                // GPIO 1 pour SDA
        .scl_io_num = I2C_MASTER_SCL_IO,                                                // GPIO 2 pour SCL
        .sda_pullup_en = GPIO_PULLUP_ENABLE,                                            // Indiquer la présence de la résistance de pull-up pour SDA : le GY-521 a des résistances de pull-up intégrées
        .scl_pullup_en = GPIO_PULLUP_ENABLE,                                            // Indiquer la présence de la résistance de pull-up pour SCL : le GY-521 a des résistances de pull-up intégrées
        .master.clk_speed = I2C_MASTER_FREQ_HZ,                                         // Fréquence I2C
    };
    i2c_param_config(I2C_NUM_0, &conf);                                                 // Configurer le port I2C 0
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);                                  // Installer le pilote I2C
}

/**
 * @brief Lit les données brutes du MPU6050 (accélération et gyroscope).
 * Cette fonction envoie une commande I2C pour lire 14 octets à partir du registre 0x3B,
 * puis convertit ces octets en valeurs 16 bits pour chaque axe (X, Y, Z) de l'accéléromètre et du gyroscope.
 * Les valeurs sont ensuite affichées dans la console.
 */
void mpu6050_read_data() {
    gpio_set_level(LED_GREEN_GPIO, 1);                                                  // Allumer la LED verte pendant la lecture
    
    uint8_t data[14];                                                                   // Buffer pour stocker les données lues

    // Création d'une commande I2C pour lire les données du MPU6050
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();                                       // Créer une commande I2C
    i2c_master_start(cmd);                                                              // Démarrer la communication I2C
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);           // Adresse du MPU6050 avec bit d'écriture
    i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, true);                             // Registre de départ des données
    i2c_master_start(cmd);                                                              // Redémarrer la communication I2C (pour passer en mode lecture)
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);            // Adresse du MPU6050 avec bit de lecture
    i2c_master_read(cmd, data, 14, I2C_MASTER_LAST_NACK);                               // Lire 14 octets de données (accélération + gyroscope)
    i2c_master_stop(cmd);                                                               // Arrêter la communication I2C
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);                    // Exécuter la commande I2C
    i2c_cmd_link_delete(cmd);                                                           // Supprimer la commande I2C

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
    gpio_set_level(LED_RED_GPIO, 1);                                                    // Allumer la LED rouge au démarrage
    gpio_set_level(LED_GREEN_GPIO, 0);                                                  // Éteindre la LED verte
}

/**
 * @brief Point d'entrée principal du programme.
 * Initialise le bus I2C, puis lit et affiche en boucle les données du MPU6050 toutes les secondes.
 */
void app_main() {
    leds_init();                                                                        // Initialiser les LEDs
    i2c_master_init();                                                                  // Initialiser l'I2C
    while (1) {
        mpu6050_read_data();                                                            // Lire et afficher les données du MPU6050
        vTaskDelay(1000 / portTICK_PERIOD_MS);                                          // Attendre 1 seconde avant la prochaine lecture
    }
}

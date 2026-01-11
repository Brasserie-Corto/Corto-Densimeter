// Définition des macros pour inclure les en-têtes nécessaires
#include "i2c_functions.h"                                                              // Pour les fonctions I2C
#include "mpu6050_functions.h"                                                          // Pour les fonctions MPU6050
#include "led_functions.h"                                                              // Pour les fonctions LED
#include "freertos/FreeRTOS.h"                                                          // Pour les définitions FreeRTOS
#include "freertos/task.h"                                                              // Pour les fonctions FreeRTOS
#include "esp_log.h"                                                                    // Pour les fonctions de log
#include "driver/gpio.h"                                                                // Pour les fonctions GPIO
#include <math.h>                                                                       // Pour les fonctions trigonométriques (atan2, M_PI)

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
#define INITIAL_ANGLE 0.0f     // Angle initial (en degrés) - à mesurer dans l'air

// Variables pour l'étalonnage
float reference_angle_x = INITIAL_ANGLE;  // Angle de référence (à mesurer dans l'air)
float reference_angle_y = INITIAL_ANGLE;
float current_density = DENSITY_WATER;    // Densité actuelle (par défaut : eau)

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
 * @brief Calcule l'inclinaison à partir des données de l'accéléromètre.
 */
void calculate_inclination(mpu6050_raw_t *raw_data, float *angle_x, float *angle_y) {
    // Convertir les valeurs brutes en g (suppose une sensibilité de 16384 LSB/g)
    float ax = raw_data->ax / 16384.0f;
    float ay = raw_data->ay / 16384.0f;
    float az = raw_data->az / 16384.0f;

    // Calculer les angles d'inclinaison (en degrés)
    *angle_x = atan2(ax, sqrt(ay * ay + az * az)) * 180.0f / M_PI;
    *angle_y = atan2(ay, sqrt(ax * ax + az * az)) * 180.0f / M_PI;
}

/**
 * @brief Estime la densité en fonction de l'inclinaison.
 * @note : Cette fonction suppose une relation linéaire entre l'inclinaison et la densité.
 *        Il faut étalonner les constantes `m` et `b` expérimentalement.
 */
float estimate_density(float angle_x, float angle_y) {
    // Exemple de relation linéaire : densité = m * angle + b
    // À ajuster selon tes mesures expérimentales
    float m = 0.01f;  // Pente (ex: 0.01 kg/m³ par degré)
    float b = 1.0f;  // Ordonnée à l'origine (densité de l'eau)

    // Utiliser la moyenne des angles X et Y pour plus de stabilité
    float avg_angle = (fabs(angle_x) + fabs(angle_y)) / 2.0f;
    return m * avg_angle + b;
}

/**
 * @brief Estime le degré d'alcool en fonction de la densité.
 * @note : Formule empirique pour estimer le %ABV (Alcohol By Volume).
 */
float estimate_alcohol_percentage(float initial_density, float final_density) {
    // Formule empirique : %ABV = 132.71 * (densité_initiale - densité_finale) / densité_finale
    return 132.71f * (initial_density - final_density) / final_density;
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

    

    // // Boucle principale
    // while (1) {
    //     gpio_set_level(LED_GREEN_GPIO, 1);
    //     ret = mpu6050_read_data(dev_handle, i2c_bus, &raw_data);
    //     if (ret == ESP_OK) {
    //         // ESP_LOGI(TAG, "Raw Accel: X=%d, Y=%d, Z=%d", raw_data.ax, raw_data.ay, raw_data.az);
    //         ESP_LOGI(TAG, "Raw Gyro: X=%d, Y=%d, Z=%d", raw_data.gx, raw_data.gy, raw_data.gz);
    //         ret = convert_mpu6050_data(&raw_data, &converted_data);
    //         if (ret == ESP_OK) {
    //             // ESP_LOGI(TAG, "Accel (g): X=%.2f, Y=%.2f, Z=%.2f", converted_data.ax, converted_data.ay, converted_data.az);
    //             ESP_LOGI(TAG, "Gyro (°/s): X=%.2f, Y=%.2f, Z=%.2f", converted_data.gx, converted_data.gy, converted_data.gz);
    //         } else {
    //             ESP_LOGE(TAG, "Erreur de conversion: %s", esp_err_to_name(ret));
    //         }
    //     } else {
    //         handle_error(ret, "Erreur de lecture du MPU6050");
    //     }
    //     gpio_set_level(LED_GREEN_GPIO, 0);
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }

    // Boucle principale
    while (1) {
        gpio_set_level(LED_GREEN_GPIO, 1);
        ret = mpu6050_read_data(dev_handle, i2c_bus, &raw_data);
        if (ret == ESP_OK) {
            // Calculer l'inclinaison
            float angle_x, angle_y;
            calculate_inclination(&raw_data, &angle_x, &angle_y);

            // Estimer la densité
            current_density = estimate_density(angle_x, angle_y);

            // Estimer le degré d'alcool (suppose une densité initiale de 1.050)
            float alcohol_percentage = estimate_alcohol_percentage(1.050f, current_density);

            // Afficher les résultats
            ESP_LOGI(TAG, "Inclinaison: X=%.2f°, Y=%.2f°", angle_x, angle_y);
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


/**
 * --------------------------------------------------------------------------------
 * THÉORIE : CALCUL DU TAUX D'ALCOOL À PARTIR DE LA DENSITÉ
 * --------------------------------------------------------------------------------
 *
 * 1. PRINCIPE PHYSIQUE :
 *    -------------------
 *    La densité d'un liquide est directement liée à sa composition chimique.
 *    Pendant la fermentation, les sucres sont transformés en alcool et en CO2,
 *    ce qui réduit la densité du moût (liquide initial).
 *
 *    - Avant fermentation : densité élevée (beaucoup de sucres).
 *    - Après fermentation : densité plus faible (moins de sucres, plus d'alcool).
 *
 *    La densité est mesurée en kg/m³ ou en unités spécifiques comme :
 *    - Degrés Plato (°P) : pourcentage de sucres dans le moût.
 *    - Degrés Brix (°Bx) : similaire au Plato, utilisé en œnologie.
 *
 *
 * 2. RELATION DENSITÉ-ALCOOL :
 *    -------------------------
 *    La formule empirique utilisée est basée sur la différence de densité
 *    avant et après fermentation :
 *
 *      %ABV = K * (densité_initiale - densité_finale) / densité_finale
 *
 *    Où :
 *    - %ABV : Pourcentage d'alcool en volume (Alcohol By Volume).
 *    - K : Constante empirique (132.71 pour les bières standard).
 *    - densité_initiale : Densité du moût avant fermentation (ex: 1.050 kg/m³).
 *    - densité_finale : Densité du moût après fermentation (ex: 1.010 kg/m³).
 *
 *    Exemple :
 *      Si densité_initiale = 1.050 et densité_finale = 1.010,
 *      %ABV ≈ 132.71 * (1.050 - 1.010) / 1.010 ≈ 5.2%.
 *
 *
 * 3. MESURE DE LA DENSITÉ :
 *    ---------------------
 *    Dans ce projet, la densité est estimée indirectement via :
 *    - L'inclinaison du capteur (mesurée par l'accéléromètre du MPU6050).
 *    - Une relation linéaire étalonnée expérimentalement :
 *        densité = m * inclinaison + b
 *      Où m et b sont déterminés en mesurant l'inclinaison dans des liquides
 *      de densités connues (ex: eau à 1.0 kg/m³, solution sucrée à 1.050 kg/m³).
 *
 *
 * 4. LIMITES ET PRÉCISION :
 *    ----------------------
 *    - La méthode suppose une relation linéaire entre inclinaison et densité,
 *      ce qui est une approximation. Un étalonnage précis est nécessaire.
 *    - La température affecte la densité. Pour une mesure précise, il faudrait
 *      compenser en fonction de la température (non implémenté ici).
 *    - La forme du capteur et sa position dans le liquide influencent
 *      l'inclinaison mesurée.
 *
 *
 * 5. ÉTALONNAGE RECOMMANDÉ :
 *    -----------------------
 *    Pour obtenir des résultats précis :
 *    1. Mesurer l'inclinaison dans l'air (référence).
 *    2. Mesurer l'inclinaison dans l'eau (densité = 1.0 kg/m³).
 *    3. Mesurer l'inclinaison dans une solution de densité connue (ex: 1.050 kg/m³).
 *    4. Ajuster les constantes m et b dans estimate_density() pour coller aux mesures.
 *
 * --------------------------------------------------------------------------------
 */

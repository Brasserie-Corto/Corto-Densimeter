#include "densimeter_functions.h"
#include <math.h>  // Pour les fonctions trigonométriques (atan2, M_PI)

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

/**
 * @brief Calcule l'inclinaison à partir des données de l'accéléromètre.
 */
esp_err_t calculate_inclination(mpu6050_raw_t *raw_data, float *angle_x, float *angle_y) {
    // Convertir les valeurs brutes en g (16384 LSB/g)
    float ax = raw_data->ax / 16384.0f;
    float ay = raw_data->ay / 16384.0f; 
    float az = raw_data->az / 16384.0f; 

    // Calculer l'angle d'inclinaison autour de X (axe vertical car capteur à la verticale)
    // On utilise ay et az car le capteur est vertical
    *angle_x = atan2(ay, sqrt(ax * ax + az * az)) * 180.0f / M_PI;  // Inclinaison avant/arrière
    *angle_y = atan2(ax, sqrt(ay * ay + az * az)) * 180.0f / M_PI;  // Inclinaison latérale (moins utile ici)

    return ESP_OK;
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
 * --------------------------------------------------------------------------------
 * THÉORIE : CALCUL DU TAUX D'ALCOOL À PARTIR DE LA PROFONDEUR D'IMMERSION
 * --------------------------------------------------------------------------------
 *
 * 1. PRINCIPE PHYSIQUE :
 *    -------------------
 *    La profondeur d'immersion d'un objet dans un liquide dépend de la densité
 *    de ce liquide (principe d'Archimède). Plus le liquide est dense, moins
 *    l'objet s'enfonce, et inversement.
 *
 *    - Avant fermentation : densité élevée (beaucoup de sucres) → immersion faible.
 *    - Après fermentation : densité plus faible (moins de sucres, plus d'alcool) → immersion plus profonde.
 *
 *    La densité est mesurée en kg/m³ ou en unités spécifiques comme :
 *    - Degrés Plato (°P) : pourcentage de sucres dans le moût.
 *    - Degrés Brix (°Bx) : similaire au Plato, utilisé en œnologie.
 *
 *
 * 2. RELATION PROFONDEUR-DENSITÉ :
 *    -----------------------------
 *    La profondeur d'immersion est mesurée indirectement via :
 *    - La composante Z de l'accélération (MPU6050), qui diminue en fonction
 *      de la poussée d'Archimède.
 *    - Une relation empirique : profondeur = k * (1 - az),
 *      où k est une constante d'étalonnage et az est l'accélération mesurée sur Z.
 *
 *    La densité est ensuite estimée par une relation linéaire :
 *      densité = a * profondeur + b
 *    où a et b sont déterminés expérimentalement.
 *
 *
 * 3. RELATION DENSITÉ-ALCOOL :
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
 * 4. LIMITES ET PRÉCISION :
 *    ----------------------
 *    - La méthode suppose une relation linéaire entre profondeur et densité,
 *      ce qui est une approximation. Un étalonnage précis est nécessaire.
 *    - La température affecte la densité. Pour une mesure précise, il faudrait
 *      compenser en fonction de la température (non implémenté ici).
 *    - La forme du capteur et sa position dans le liquide influencent
 *      la profondeur mesurée.
 *
 *
 * 5. ÉTALONNAGE RECOMMANDÉ :
 *    -----------------------
 *    Pour obtenir des résultats précis :
 *    1. Mesurer la profondeur dans l'air (référence : profondeur ≈ 0).
 *    2. Mesurer la profondeur dans l'eau (densité = 1.0 kg/m³).
 *    3. Mesurer la profondeur dans une solution de densité connue (ex: 1.050 kg/m³).
 *    4. Ajuster les constantes k, a, et b pour coller aux mesures.
 *
 * --------------------------------------------------------------------------------
 */

float measure_immersion_depth(mpu6050_raw_t *raw_data) {
    // Convertir la valeur brute de l'accéléromètre Z en g
    float az = raw_data->az / 16384.0f;

    // Dans l'air, az ≈ 1g (9.81 m/s²)
    // Dans le liquide, az = 1g - (poussée d'Archimède)
    // La poussée dépend de la densité et de la profondeur

    // Relation empirique : profondeur = k * (1 - az)
    // k est une constante à étalonner
    float k = 10.0f;  // À ajuster expérimentalement
    return k * (1.0f - az);
}

float estimate_density_from_depth(float depth) {
    // Relation empirique : densité = a * depth + b
    // a et b sont des constantes à étalonner
    float a = -0.01f;  // Exemple : densité diminue de 0.01 kg/m³ par mm d'immersion
    float b = 1.1f;    // Densité maximale (quand depth = 0)

    return a * depth + b;
}

float estimate_alcohol_percentage_from_depth(float initial_density, float final_density) {
    if (final_density >= initial_density) {
        return 0.0f;
    }
    return 132.71f * (initial_density - final_density) / final_density;
}

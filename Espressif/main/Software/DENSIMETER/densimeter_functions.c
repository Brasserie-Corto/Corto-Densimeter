#include "densimeter_functions.h"
#include <math.h>

/**
 * ================================================================================
 * DENSIMÈTRE NUMÉRIQUE - MÉTHODE PAR ENFONCEMENT (POUSSÉE D'ARCHIMÈDE)
 * ================================================================================
 *
 * PRINCIPE :
 * ----------
 * Le densimètre flotte dans la bière comme un hydromètre classique.
 * L'accéléromètre MPU6050 mesure la gravité apparente qui varie selon la poussée.
 *
 *   Liquide DENSE (sucré) → forte poussée → flotte HAUT → az élevé
 *   Liquide PEU DENSE (alcool) → faible poussée → S'ENFONCE → az diminue
 *
 * MESURE SIMPLE :
 * ---------------
 *   1. Mesurer az au DÉBUT de la fermentation (référence) → az_initial
 *   2. Mesurer az EN CONTINU pendant la fermentation → az_actuel
 *   3. Différence = enfoncement = az_initial - az_actuel
 *   4. Convertir enfoncement → densité (étalonnage nécessaire)
 *   5. Calculer %ABV avec densité initiale et finale
 *
 * FORMULE %ABV :
 * --------------
 *   %ABV = 132.71 × (densité_initiale - densité_finale) / densité_finale
 *
 *   Exemple : densité_initiale = 1.050, densité_finale = 1.010
 *            → %ABV ≈ 5.2%
 *
 * ÉTALONNAGE REQUIS :
 * -------------------
 *   1. Mesurer az dans l'eau (densité = 1.000) → point 1
 *   2. Mesurer az dans solution sucrée (densité = 1.050) → point 2
 *   3. Calculer la relation linéaire : densité = a × az + b
 *
 * ================================================================================
 */

/**
 * @brief Lit la valeur brute de l'axe Z et la convertit en g.
 * 
 * @param raw_data Données brutes de l'accéléromètre
 * @return Valeur de l'accélération sur l'axe Z en g (gravités)
 */
float get_az_value(mpu6050_raw_t *raw_data) {
    // Convertir la valeur brute en g (16384 LSB/g pour la plage ±2g)
    return raw_data->az / 16384.0f;
}

/**
 * @brief Calcule l'enfoncement du densimètre par rapport à la référence.
 * 
 * @param az_initial Valeur de az au début de la fermentation (référence)
 * @param az_current Valeur de az actuelle
 * @return Enfoncement relatif (plus le nombre est grand, plus le densimètre s'est enfoncé)
 */
float calculate_immersion_delta(float az_initial, float az_current) {
    // Simple différence : quand le densimètre s'enfonce, az diminue
    return az_initial - az_current;
}

/**
 * @brief Estime la densité à partir de la valeur az (ou de l'enfoncement).
 * 
 * MÉTHODE DIRECTE avec az :
 * -------------------------
 * Cette fonction convertit directement la valeur az en densité.
 * Plus az est élevé, plus le liquide est dense (forte poussée).
 * 
 * ÉTALONNAGE NÉCESSAIRE :
 * -----------------------
 * Mesurer az dans deux liquides de densités connues :
 *   - Eau : densité = 1.000, mesurer az1
 *   - Solution sucrée : densité = 1.050, mesurer az2
 * 
 * Calculer :
 *   a = (1.050 - 1.000) / (az2 - az1) = 0.050 / Δaz
 *   b = 1.000 - a × az1
 * 
 * @param az Valeur de l'accélération sur Z (en g)
 * @return Densité estimée (kg/L ou g/mL)
 */
float estimate_density_from_az(float az) {
    // VALEURS À ÉTALONNER EXPÉRIMENTALEMENT
    // Exemple avec des valeurs hypothétiques :
    //   - Dans l'eau (1.000) : az = 0.95g
    //   - Dans moût (1.050) : az = 1.00g
    //   → a = 0.050 / 0.05 = 1.0
    //   → b = 1.000 - 1.0 × 0.95 = 0.05
    
    float a = 1.0f;   // Pente (à étalonner)
    float b = 0.05f;  // Ordonnée à l'origine (à étalonner)
    
    return a * az + b;
}

/**
 * @brief Estime la densité à partir de l'enfoncement (différence d'az).
 * 
 * MÉTHODE PAR DIFFÉRENCE :
 * ------------------------
 * Cette fonction convertit la différence d'az (enfoncement) en densité.
 * Plus l'enfoncement est grand, moins le liquide est dense.
 * 
 * @param immersion_delta Enfoncement = az_initial - az_current
 * @return Densité estimée (kg/L ou g/mL)
 */
float estimate_density_from_immersion(float immersion_delta) {
    // VALEURS À ÉTALONNER EXPÉRIMENTALEMENT
    // Relation : densité diminue quand l'enfoncement augmente
    
    float a = -0.5f;  // Pente négative (à étalonner)
    float b = 1.05f;  // Densité maximale au début (à étalonner)
    
    return a * immersion_delta + b;
}

/**
 * @brief Calcule le pourcentage d'alcool (ABV) à partir des densités.
 * 
 * Formule empirique standard pour la bière.
 * 
 * @param initial_density Densité au début de la fermentation (ex: 1.050)
 * @param final_density Densité actuelle ou finale (ex: 1.010)
 * @return Pourcentage d'alcool en volume (%ABV)
 */
float calculate_alcohol_percentage(float initial_density, float final_density) {
    // Protection contre les valeurs aberrantes
    if (final_density >= initial_density) {
        return 0.0f;
    }
    
    // Formule empirique : %ABV = 132.71 × (OG - FG) / FG
    // OG = Original Gravity (densité initiale)
    // FG = Final Gravity (densité finale)
    return 132.71f * (initial_density - final_density) / final_density;
}
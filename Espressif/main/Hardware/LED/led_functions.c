#include "led_functions.h"
#include "driver/gpio.h"

esp_err_t leds_init(gpio_num_t green_led, gpio_num_t red_led) {
    gpio_config_t io_conf = {
        .pin_bit_mask = ((1ULL << green_led) | (1ULL << red_led)),          // Sélectionner les broches des LEDs : 1ULL << n crée un masque avec le n-ième bit à 1, ce qui correspond à la broche GPIO n.
        .mode = GPIO_MODE_OUTPUT,                                           // Configurer comme sorties
        .pull_up_en = GPIO_PULLUP_DISABLE,                                  // Désactiver la résistance de tirage vers le haut
        .pull_down_en = GPIO_PULLDOWN_DISABLE,                              // Désactiver la résistance de tirage vers le bas
        .intr_type = GPIO_INTR_DISABLE,                                     // Désactiver les interruptions
    };
    gpio_config(&io_conf);                                                  // Appliquer la configuration
    return ESP_OK;
}

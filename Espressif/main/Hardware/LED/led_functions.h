#ifndef LED_FUNCTIONS_H
#define LED_FUNCTIONS_H

#include "driver/gpio.h"
#include "esp_err.h"

esp_err_t leds_init(gpio_num_t green_led, gpio_num_t red_led);

#endif // LED_FUNCTIONS_H

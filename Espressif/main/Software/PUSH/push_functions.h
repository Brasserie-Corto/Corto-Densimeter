#ifndef PUSH_FUNCTIONS_H
#define PUSH_FUNCTIONS_H

#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

/**
 * @brief Initialise le WiFi en mode station et se connecte au réseau.
 * @return ESP_OK si connexion réussie, ESP_FAIL sinon
 */
esp_err_t wifi_init_sta(void);

/**
 * @brief Envoie les données du densimètre vers l'API via HTTP POST (JSON).
 * @param depth Delta d'enfoncement (immersion_delta)
 * @param density Densité estimée (kg/L)
 * @param alcohol Pourcentage d'alcool estimé (%)
 * @return ESP_OK si code HTTP 200, ESP_FAIL sinon
 */
esp_err_t wifi_push(float depth, float density, float alcohol);

#endif // PUSH_FUNCTIONS_H
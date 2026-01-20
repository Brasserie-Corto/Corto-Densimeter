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

esp_err_t wifi_init_sta(void);
esp_err_t wifi_push(float depth, float density, float alcohol);

#endif // PUSH_FUNCTIONS_H
#include "push_functions.h"
#include <string.h>
#include <stdio.h>

#define WIFI_SSID      "Pierre’s Iphone"
#define WIFI_PASS      "Tititutu"
#define SERVER_URL     "https://www.postb.in/1768894944677-0351539265830"
#define MAX_RETRY      5

static const char *TAG = "WIFI_PUSH";
static EventGroupHandle_t wifi_event_group;
static int retry_count = 0;

#define CONNECTED_BIT BIT0
#define FAILED_BIT    BIT1

static void wifi_event_callback(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            if (retry_count < MAX_RETRY) {
                esp_wifi_connect();
                retry_count++;
                ESP_LOGW(TAG, "Tentative de reconnexion %d/%d", retry_count, MAX_RETRY);
            } else {
                xEventGroupSetBits(wifi_event_group, FAILED_BIT);
                ESP_LOGE(TAG, "Échec de connexion WiFi");
            }
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "IP obtenue: " IPSTR, IP2STR(&event->ip_info.ip));
        retry_count = 0;
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
    }
}

esp_err_t wifi_init_sta(void)
{
    wifi_event_group = xEventGroupCreate();
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_callback,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_callback,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                           CONNECTED_BIT | FAILED_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    if (bits & CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connexion WiFi réussie");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Échec connexion WiFi");
        return ESP_FAIL;
    }
}

esp_err_t wifi_push(float depth, float density, float alcohol)
{
    char post_data[256];
    snprintf(post_data, sizeof(post_data),
             "api_key=tPmAT5Ab3j7F9&sensor=Densimeter&depth=%.2f&density=%.3f&alcohol=%.2f",
             depth, density, alcohol);

    esp_http_client_config_t config = {
        .url = SERVER_URL,
        .method = HTTP_METHOD_POST,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    
    esp_http_client_set_header(client, "Content-Type", "application/x-www-form-urlencoded");
    esp_http_client_set_post_field(client, post_data, strlen(post_data));

    esp_err_t err = esp_http_client_perform(client);
    
    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        ESP_LOGI(TAG, "Code HTTP: %d", status);
        esp_http_client_cleanup(client);
        return (status == 200) ? ESP_OK : ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "Erreur HTTP: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return err;
    }
}
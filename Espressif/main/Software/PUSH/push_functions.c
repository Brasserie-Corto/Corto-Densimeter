#include "push_functions.h"
#include <string.h>
#include <stdio.h>
#include "esp_crt_bundle.h"

#define WIFI_SSID      "WiFi_Coloc_Invite"
#define WIFI_PASS      "TestArduino"
#define SERVER_URL     "http://httpbin.org/post"
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
            ESP_LOGI(TAG, "WiFi démarré, connexion en cours...");
            esp_wifi_connect();
        } else if (event_id == WIFI_EVENT_STA_CONNECTED) {
            ESP_LOGI(TAG, "Connecté au point d'accès, attente IP...");
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*) event_data;
            ESP_LOGW(TAG, "Déconnecté - raison: %d", event->reason);
            
            if (retry_count < MAX_RETRY) {
                vTaskDelay(pdMS_TO_TICKS(1000)); // Attendre 1s avant reconnexion
                esp_wifi_connect();
                retry_count++;
                ESP_LOGW(TAG, "Tentative de reconnexion %d/%d", retry_count, MAX_RETRY);
            } else {
                xEventGroupSetBits(wifi_event_group, FAILED_BIT);
                ESP_LOGE(TAG, "Échec de connexion WiFi après %d tentatives", MAX_RETRY);
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
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
            .scan_method = WIFI_ALL_CHANNEL_SCAN,
            .bssid_set = false,
            .channel = 0,
            .listen_interval = 0,
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
        },
    };
    
    // Copier le mot de passe byte par byte
    memcpy(wifi_config.sta.password, WIFI_PASS, sizeof(WIFI_PASS));

    ESP_LOGI(TAG, "Configuration WiFi - SSID: %s", WIFI_SSID);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    
    // Désactiver le power save pour éviter les déconnexions
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    
    // Limiter la bande passante 20MHz pour compatibilité Freebox
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW20));
    
    // Définir un protocole compatible (b/g/n uniquement)
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N));
    
    ESP_LOGI(TAG, "Démarrage WiFi...");
    ESP_ERROR_CHECK(esp_wifi_start());

    // Attendre max 20 secondes pour connexion et IP
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                           CONNECTED_BIT | FAILED_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           20000 / portTICK_PERIOD_MS);

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
    // Préparer les données JSON
    char json_data[256];
    snprintf(json_data, sizeof(json_data),
             "{\"sensor\":\"Densimeter\",\"immersion_delta\":%.3f,\"density\":%.3f,\"alcohol\":%.2f}",
             depth, density, alcohol);

    // Configuration HTTP client
    esp_http_client_config_t config = {
        .url = SERVER_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 5000,  // Timeout de 5 secondes
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "Échec init client HTTP");
        return ESP_FAIL;
    }
    
    // Définir les headers et le body
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, json_data, strlen(json_data));

    // Exécuter la requête
    esp_err_t err = esp_http_client_perform(client);
    int status_code = 0;
    
    if (err == ESP_OK) {
        status_code = esp_http_client_get_status_code(client);
        ESP_LOGI(TAG, "POST réussi - Code: %d - Données: %s", status_code, json_data);
    } else {
        ESP_LOGE(TAG, "Échec POST: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    return (err == ESP_OK && status_code == 200) ? ESP_OK : ESP_FAIL;
}
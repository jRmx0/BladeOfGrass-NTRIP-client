// TODO
// - No WiFi available (go to energy saving mode)
#include "wifi_connect.h"

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include <netdb.h>

#define WIFI_TAG                        "WIFI CONNECT"

// Connection status
#define BIT_CONNECTED                   BIT0
#define BIT_DISCONNECTED                BIT1
static EventGroupHandle_t wifi_events   = NULL;

// Reconnection
static int reconnection_count           = 0;
#define RECONNECT_DELAY_MS              5000
SemaphoreHandle_t reconnectSemaphore;

// Private Variables 
static esp_netif_t *esp_netif           = NULL;
static bool attempt_reconnect           = true;

// Forward Declarations 
static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void wifi_attempt_reconnect(void *params);
char *get_wifi_disconnection_string(wifi_err_reason_t reason); 

// Event Handler 
static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
        case WIFI_EVENT_STA_START:
        {
            xEventGroupSetBits(wifi_status, WIFI_STATUS_CONNECTING);
            ESP_LOGI(WIFI_TAG, "WIFI_EVENT_STA_START");
            esp_wifi_connect();
            break;
        }

        case WIFI_EVENT_STA_CONNECTED:
        {
            ESP_LOGI(WIFI_TAG, "WIFI_EVENT_STA_CONNECTED");
            reconnection_count  = 0;
            break;
        }
    
        case WIFI_EVENT_STA_DISCONNECTED:
        {
            wifi_event_sta_disconnected_t *event = event_data;
            ESP_LOGW(WIFI_TAG, "DISCONNECTED %d: %s",
                     event->reason,
                     get_wifi_disconnection_string(event->reason));

            if (attempt_reconnect)
            {
                xEventGroupSetBits(wifi_status, WIFI_STATUS_RECONNECTING);
                ESP_LOGI(WIFI_TAG, "Attempting to reconnect... (%d)", ++reconnection_count);
                xSemaphoreGive(reconnectSemaphore);
            }
            break;
        }

        case IP_EVENT_STA_GOT_IP:
        {
            ESP_LOGI(WIFI_TAG, "IP_EVENT_STA_GOT_IP");
            xEventGroupSetBits(wifi_events, BIT_CONNECTED);
            xEventGroupSetBits(wifi_status, WIFI_STATUS_CONNECTED);
            break;
        }    
            
        default:
            break;
    }
}

// Public Functions
void wifi_connect_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, event_handler, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    // Disable cluttering warning messages (IDF v5)
    esp_log_level_set("wifi", ESP_LOG_ERROR);
}

esp_err_t wifi_connect_sta(char *ssid, char *pass, int timeout)
{
    attempt_reconnect = true;
    
    wifi_events = xEventGroupCreate();
    wifi_status = xEventGroupCreate();
    reconnectSemaphore = xSemaphoreCreateBinary();
    xTaskCreate(&wifi_attempt_reconnect, "wifi_attempt_reconnect", 4096, NULL, 5, NULL);

    esp_netif = esp_netif_create_default_wifi_sta();
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    wifi_config_t wifi_config = { 0 };
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, pass, sizeof(wifi_config.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(WIFI_TAG, "Connecting to SSID: %s", ssid);

    EventBits_t result = xEventGroupWaitBits(
        wifi_events,
        BIT_CONNECTED | BIT_DISCONNECTED,
        pdTRUE,           // Clear bits on exit
        pdFALSE,          // Wait for any bit
        pdMS_TO_TICKS(timeout)
    );

    if (result & BIT_CONNECTED)
    {
        ESP_LOGI(WIFI_TAG, "Successfully connected to SSID: %s", ssid);
        return ESP_OK;
    }

    ESP_LOGW(WIFI_TAG, "Failed to connect to SSID: %s", ssid);
    return ESP_FAIL;
}

void wifi_connect_ap(const char *ssid, const char *pass)
{
    esp_netif = esp_netif_create_default_wifi_ap();
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

    wifi_config_t wifi_config = { 0 };
    strncpy((char *)wifi_config.ap.ssid, ssid, sizeof(wifi_config.ap.ssid) - 1);
    strncpy((char *)wifi_config.ap.password, pass, sizeof(wifi_config.ap.password) - 1);

    wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    wifi_config.ap.max_connection = 4;
    wifi_config.ap.beacon_interval = 100;
    wifi_config.ap.channel = 1;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(WIFI_TAG, "Access Point started with SSID: %s", ssid);
}

void wifi_disconnect(void)
{
    attempt_reconnect = false;

    ESP_LOGI(WIFI_TAG, "Disconnecting and stopping WiFi");

    ESP_ERROR_CHECK(esp_wifi_stop());

    if (esp_netif)
    {
        esp_netif_destroy(esp_netif);
        esp_netif = NULL;
    }

    xEventGroupSetBits(wifi_status, WIFI_STATUS_DISCONNECTED);
}

// Private Functions
static void wifi_attempt_reconnect(void *params)
{
    while (true)
    {
        xSemaphoreTake(reconnectSemaphore, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(RECONNECT_DELAY_MS));
        ESP_ERROR_CHECK(esp_wifi_connect());
    }
}
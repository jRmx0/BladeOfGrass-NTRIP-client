#include "ntrip_client.h"
#include "uart_um980.h"
#include "wifi_monitor.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_http_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"

#define NTRIP_BUFFER_SIZE 1024
#define GGA_BUFFER_SIZE 256

#define TAG "NTRIP_CLIENT"

static struct
{
    char host[64];
    int port;
    char mountpoint[64];
    char username[32];
    char password[32];

    esp_http_client_handle_t http_client;
    TaskHandle_t client_task_handle;
    TaskHandle_t ntrip_rtcm_forward_task_handle;
    TaskHandle_t gga_queue_task_handle;

    bool connected;
    bool shutdown_requested;

    QueueHandle_t gga_queue;
    QueueHandle_t rtcm_queue;

} ntrip_client = {0};

esp_err_t http_client_event_handler(esp_http_client_event_t *evt);

static void ntrip_client_task(void *pvParameters);
static esp_err_t ntrip_http_init(void);
static esp_err_t ntrip_connect(void);
static void ntrip_cleanup(void);

static void ntrip_gga_queue_task(void *pvParameters);
static void ntrip_rtcm_forward_task(void *pvParameters);

esp_err_t http_client_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ERROR:
        ESP_LOGW(TAG, "HTTP_EVENT_ERROR");
        ntrip_client.connected = false;
        break;
        
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
        
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
        
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", 
                 evt->header_key, evt->header_value);
        break;
        
    case HTTP_EVENT_ON_DATA:
        // ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, Length=%d", evt->data_len);
        break;
        
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_FINISH");
        break;
        
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
        ntrip_client.connected = false;
        break;
        
    default:
        ESP_LOGW(TAG, "Unhandled HTTP event: %d", evt->event_id);
        break;
    }
    return ESP_OK;
}


esp_err_t ntrip_client_init(const char *host, int port, const char *mountpoint,
                            const char *username, const char *password, 
                            QueueHandle_t gga_queue, QueueHandle_t rtcm_queue)
{
    memset(&ntrip_client, 0, sizeof(ntrip_client));

    strncpy(ntrip_client.host, host, sizeof(ntrip_client.host) - 1);
    ntrip_client.port = port;
    strncpy(ntrip_client.mountpoint, mountpoint, sizeof(ntrip_client.mountpoint) - 1);
    strncpy(ntrip_client.username, username, sizeof(ntrip_client.username) - 1);
    strncpy(ntrip_client.password, password, sizeof(ntrip_client.password) - 1);

    ntrip_client.connected = false;
    ntrip_client.shutdown_requested = false;

    ntrip_client.gga_queue = gga_queue;
    ntrip_client.rtcm_queue = rtcm_queue;

    ESP_LOGI(TAG, "NTRIP client initialized for %s:%d/%s",
             ntrip_client.host, ntrip_client.port, ntrip_client.mountpoint);

    return ESP_OK;
}

esp_err_t ntrip_client_start(void)
{
    if (ntrip_client.client_task_handle != NULL)
    {
        ESP_LOGW(TAG, "NTRIP client already started");
        return ESP_OK;
    }

    ntrip_client.shutdown_requested = false;
    ntrip_client.connected = false;

    xTaskCreate(ntrip_client_task,
                "ntrip_client_task",
                8192,
                NULL,
                8,
                &ntrip_client.client_task_handle);

    ESP_LOGI(TAG, "NTRIP client started");
    return ESP_OK;
}

esp_err_t ntrip_client_stop(void)
{
    if (ntrip_client.client_task_handle == NULL)
    {
        ESP_LOGW(TAG, "NTRIP client not running");
        return ESP_OK;
    }

    ntrip_client.shutdown_requested = true;

    // Wait for tasks to clean up
    vTaskDelay(pdMS_TO_TICKS(1000));

    if (ntrip_client.http_client != NULL)
    {
        esp_http_client_close(ntrip_client.http_client);
        esp_http_client_cleanup(ntrip_client.http_client);
        ntrip_client.http_client = NULL;
    }

    ntrip_client.connected = false;

    ESP_LOGI(TAG, "NTRIP client stopped");
    return ESP_OK;
}

bool ntrip_client_is_connected(void)
{
    return ntrip_client.connected;
}

// Main NTRIP client task - handles connection management
static void ntrip_client_task(void *pvParameters)
{
    while (!ntrip_client.shutdown_requested)
    {
        if (!connected_to_wifi())
        {
            ESP_LOGI(TAG, "Not connected to wifi");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        
        if (ntrip_http_init() != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to initialize HTTP client");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        if (ntrip_connect() != ESP_OK)
        {
            ntrip_cleanup();
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        ntrip_client.connected = true;
        ESP_LOGI(TAG, "Connected to NTRIP caster successfully");

        xTaskCreate(ntrip_gga_queue_task,
                    "ntrip_gga_task",
                    4096,
                    NULL,
                    9,
                    &ntrip_client.gga_queue_task_handle);
        
        xTaskCreate(ntrip_rtcm_forward_task,
                    "ntrip_rtcm_forward_task",
                    8192,
                    NULL,
                    9,
                    &ntrip_client.ntrip_rtcm_forward_task_handle);

        // Simple keep-alive loop - just monitor connection status
        while (ntrip_client.connected && !ntrip_client.shutdown_requested)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        if (ntrip_client.gga_queue_task_handle != NULL)
        {
            vTaskDelete(ntrip_client.gga_queue_task_handle);
            ntrip_client.gga_queue_task_handle = NULL;
        }

        if (ntrip_client.ntrip_rtcm_forward_task_handle != NULL)
        {
            vTaskDelete(ntrip_client.ntrip_rtcm_forward_task_handle);
            ntrip_client.ntrip_rtcm_forward_task_handle = NULL;
        }

        ntrip_cleanup();

        if (ntrip_client.shutdown_requested)
        {
            break;
        }

        // Wait before reconnecting
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    ntrip_client.client_task_handle = NULL;
    vTaskDelete(NULL);
}

// Initialize the HTTP client for NTRIP
static esp_err_t ntrip_http_init(void)
{
    ESP_LOGI(TAG, "Initializing HTTP client for NTRIP connection");

    char path[128];
    snprintf(path, sizeof(path), "/%s", ntrip_client.mountpoint);

    // Initialize HTTP client configuration
    esp_http_client_config_t config = {
        .host = ntrip_client.host,
        .port = ntrip_client.port,
        .path = path,
        .method = HTTP_METHOD_GET,
        .timeout_ms = 120000, // 2 minutes timeout
        .skip_cert_common_name_check = true,
        .transport_type = HTTP_TRANSPORT_OVER_TCP,
        .buffer_size = NTRIP_BUFFER_SIZE * 2,
        .buffer_size_tx = NTRIP_BUFFER_SIZE * 2,
        .auth_type = HTTP_AUTH_TYPE_BASIC,
        .username = ntrip_client.username,
        .password = ntrip_client.password,
        .event_handler = http_client_event_handler
    };

    ntrip_client.http_client = esp_http_client_init(&config);

    // Add NTRIP specific headers
    esp_http_client_set_header(ntrip_client.http_client, "Ntrip-Version", "Ntrip/2.0");
    esp_http_client_set_header(ntrip_client.http_client, "User-Agent", "NTRIP ESP32Client/1.0");
    esp_http_client_set_header(ntrip_client.http_client, "Accept", "*/*");

    return ESP_OK;
}

// Connect to the NTRIP server
static esp_err_t ntrip_connect(void)
{
    ESP_LOGI(TAG, "Opening connection to NTRIP caster %s:%d",
             ntrip_client.host, ntrip_client.port);

    esp_err_t err = esp_http_client_open(ntrip_client.http_client, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open connection with NTRIP caster: %s", esp_err_to_name(err));
        return err;
    }

    // Fetch headers to get HTTP status code
    int content_length = esp_http_client_fetch_headers(ntrip_client.http_client);
    int status_code = esp_http_client_get_status_code(ntrip_client.http_client);

    ESP_LOGI(TAG, "NTRIP connection HTTP status: %d, Content length: %d",
             status_code, content_length);

    // Check if connection was successful
    if (status_code != 200)
    {
        ESP_LOGE(TAG, "NTRIP server returned error code: %d", status_code);
        return ESP_FAIL;
    }

    return ESP_OK;
}

// Clean up HTTP client resources
static void ntrip_cleanup(void)
{
    if (ntrip_client.http_client != NULL)
    {
        ESP_LOGW(TAG, "NTRIP connection lost, cleaning up");
        esp_http_client_close(ntrip_client.http_client);
        esp_http_client_cleanup(ntrip_client.http_client);
        ntrip_client.http_client = NULL;
    }

    ntrip_client.connected = false;
}

// Task to process GGA messages from queue
static void ntrip_gga_queue_task(void *pvParameters)
{   
    gga_report_t gga_report;

    while (!ntrip_client.shutdown_requested)
    {
        xQueueReceive(ntrip_client.gga_queue, &gga_report, pdMS_TO_TICKS(portMAX_DELAY));

        if (ntrip_client.connected)
        {
            ESP_LOGD(TAG, "Sending GGA to NTRIP caster: %.*s", gga_report.size, (const char*) gga_report.data);

            int written = esp_http_client_write(ntrip_client.http_client,
                                                (const char *)gga_report.data,
                                                gga_report.size);

            if (written > 0)
            {
                ESP_LOGD(TAG, "Successfully sent %d bytes to NTRIP caster", written);
            }
            else
            {
                ESP_LOGE(TAG, "Failed to send GGA to NTRIP caster: %d", written);
            }
        }
    }

    ntrip_client.gga_queue_task_handle = NULL;
    vTaskDelete(NULL);
}

// Task for handling NTRIP data reception
static void ntrip_rtcm_forward_task(void *pvParameters)
{
    rtcm_corrections_t rtcm_correction;
    uint8_t correction_data[NTRIP_BUFFER_SIZE];
    TickType_t last_successful_read = xTaskGetTickCount();
    int data_read_len;

    while (ntrip_client.connected && !ntrip_client.shutdown_requested)
    {
        data_read_len = esp_http_client_read_response(ntrip_client.http_client,
                                                      (char *)correction_data,
                                                      NTRIP_BUFFER_SIZE);
        
        if (data_read_len > 0)
        {
            memcpy(rtcm_correction.data, correction_data, data_read_len);
            rtcm_correction.size = data_read_len;
            
            ESP_LOGD(TAG, "Received %d bytes of correction data", data_read_len);

            ESP_LOG_BUFFER_HEXDUMP(TAG, rtcm_correction.data,
                                   (data_read_len > 16) ? 16 : rtcm_correction.size, ESP_LOG_DEBUG);

            if (xQueueSend(ntrip_client.rtcm_queue, &rtcm_correction, pdMS_TO_TICKS(10)) != pdPASS)
            {
                ESP_LOGW(TAG, "Failed to queue RTCM data, queue full");
            }

            // Reset reconnection timer on successful data
            last_successful_read = xTaskGetTickCount();
        }
        else if (data_read_len == 0)
        {
            // Check for timeout - if no data received for a long time
            if ((xTaskGetTickCount() - last_successful_read) > pdMS_TO_TICKS(90000))
            { 
                ESP_LOGW(TAG, "No data received for 90 seconds, reconnecting");
                ntrip_client.connected = false;
                break;
            }
        }
        else if (data_read_len < 0)
        {
            esp_err_t err = esp_http_client_get_errno(ntrip_client.http_client);

            // Handle connection issues
            if (err == ECONNRESET || err == ENOTCONN || err == ECONNABORTED)
            {
                ESP_LOGW(TAG, "Connection reset by NTRIP caster, will reconnect");
            }
            else
            {
                ESP_LOGE(TAG, "Error reading from NTRIP caster: %d", err);
            }

            ntrip_client.connected = false;
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    ntrip_client.ntrip_rtcm_forward_task_handle = NULL;
    vTaskDelete(NULL);
}
